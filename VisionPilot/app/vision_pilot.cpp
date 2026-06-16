// VisionPilot — preprocess → inference → fusion → display
#include <config/vision_pilot_config.hpp>
#include <debug/debug_draw.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <visualization/visualization.hpp>

#include <camera_interface/frame_source.hpp>
#ifdef ENABLE_ROS2_INTERFACE
#include <control/lateral_control.hpp>
#include <control/longitudinal_control.hpp>
#include <control_cmd_publisher/cmd_to_ros2.hpp>
#include <planning/planning.hpp>
#include <vehicle_state_subscriber/ros2_to_can.hpp>
#endif
#ifdef ENABLE_WEBRTC
#include <visualization/visualization_to_webrtc.hpp>
#endif

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <thread>

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

int main(int argc, char** argv)
{
    // ── 1. Config ─────────────────────────────────────────────────────────────
    const std::string cfg_path = resolve_vision_pilot_config_path(argc, argv);
    if (cfg_path.empty()) {
        VP_ERROR("No config — cp config/vision_pilot.conf.example config/vision_pilot.conf");
        return 1;
    }

    VisionPilotConfig cfg;
    try { cfg = load_vision_pilot_config(cfg_path); }
    catch (const std::exception& e) { VP_ERROR("Config: %s", e.what()); return 1; }

    // ── 2. Pipeline (preprocess + ONNX + inference/fusion) ────────────────────
    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine_cfg);
    vm::InferencePipeline pipeline(engine, {
        cfg.autodrive_model, cfg.autosteer_model, cfg.autospeed_model,
        cfg.homography_path, cfg.fusion_debug,
    });

    vd::init_wheel_assets(cfg.wheel_dir);
    vd::init_homography(cfg.homography_path);

    // ── 3. Display output ─────────────────────────────────────────────────────
    bool show_window = true;
#ifdef ENABLE_WEBRTC
    std::unique_ptr<visualization::WebRTCStreamer> webrtc;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--webrtc") show_window = false;
        if (std::string(argv[i]) == "--webrtc-port" && i + 1 < argc) {
            webrtc = std::make_unique<visualization::WebRTCStreamer>();
            if (!webrtc->init(static_cast<uint16_t>(std::stoi(argv[++i])))) return 1;
        }
    }
#endif

    // ── 4. Frame source (video / V4L2 / ROS2) ───────────────────────────────
    auto source = camera_interface::open_frame_source(cfg.source);
    if (!source || !source->is_device_open()) {
        VP_ERROR("Cannot open frame source");
        return 1;
    }

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    const std::string label = source_label(cfg.source);
    cv::Mat frame, warped, resized;

#ifdef ENABLE_ROS2_INTERFACE
    // ── Control / actuation (camera-only drive) ──────────────────────────────
    // Planner is the MPC controller core; the control modules are thin adapters
    // mapping its {acceleration, steering_sequence} to an Ackermann command.
    Planner planner;
    visionpilot::control::LateralController lat(
        visionpilot::control::LateralController::Config{/*max_steering_rad=*/0.6});
    visionpilot::control::LongitudinalController lon(
        visionpilot::control::LongitudinalController::Config{
            cfg.control.min_cruise_mps, cfg.control.max_speed_mps});
    control_interface::ControlCommandPublisher pub(cfg.control.topic);
    vehicle_state_interface::VehicleStateSubscriber vss(cfg.control.vehicle_state_topic);
    bool control_ready_signaled = false;
#endif

    // ── 5. Main loop ────────────────────────────────────────────────────────
    while (true) {
        auto [ok, frame] = source->get_latest_frame();
        if (!ok || frame.empty()) {
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);

        if (const auto r = pipeline.process(warped)) {
            pipeline.latency().print();
#ifdef ENABLE_ROS2_INTERFACE
            // Camera perception → fusion → MPC plan → Ackermann command.
            if (r->lateral.valid) {
                const double ego_v = vss.get_ego_speed_mps();
                // cipo_v is the lead's absolute speed: velocity_ms is relative
                // (−ve = approaching), so add ego speed back.
                auto [accel, steer_seq] = planner.compute_plan(
                    r->lateral.cte_m, r->lateral.yaw_rad, r->lateral.curvature,
                    ego_v, r->cipo.valid, r->cipo.velocity_ms + ego_v, r->cipo.distance_m);
                double steer = lat.compute(steer_seq.empty() ? 0.0 : steer_seq[0]);
                const double speed = lon.compute(accel, ego_v);
                // steering_angle is the real front-wheel angle (rad). Optionally
                // invert the sign for consumers whose convention is opposite
                // (config control.invert_steering).
                if (cfg.control.invert_steering) steer = -steer;
                // Acceleration: agnostic default forwards the planner's computed
                // acceleration so its RSS following-distance reaches the actuator.
                // control.track_speed_only (opt-in, e.g. CARLA's native --ros2
                // controller) instead forwards zero accel and lets the speed command
                // (floored at min_cruise_mps) carry the motion.
                const double accel_cmd = cfg.control.track_speed_only ? 0.0 : accel;
                pub.publish(steer, speed, accel_cmd);
                // Readiness handshake (opt-in): on the first valid control command,
                // signal external orchestration that control is live — e.g. a sim
                // spawn helper holding the ego stationary until VisionPilot drives.
                if (!control_ready_signaled && !cfg.control.ready_sentinel_path.empty()) {
                    std::ofstream(cfg.control.ready_sentinel_path).put('1');
                    control_ready_signaled = true;
                }
            }
#endif
            vd::annotate_frame(warped, vd::debug_view_from(
                *r, label, cfg.wheel_dir, cfg.homography_path));
        }

        if (show_window) visualization::render_frame(warped, "VisionPilot", {});
#ifdef ENABLE_WEBRTC
        if (webrtc) webrtc->push_frame(warped);
#endif
    }

    visualization::close_windows();
    return 0;
}
