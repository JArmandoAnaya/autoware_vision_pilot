// ─────────────────────────────────────────────────────────────────────────────
// VisionPilot — main application
// ─────────────────────────────────────────────────────────────────────────────
#include <config/vision_pilot_config.hpp>
#include <logging/logger.hpp>
#include <debug/debug_draw.hpp>

#include <camera_subscriber/ros2_to_opencv.hpp>
#include <v4l2_interface/v4l2_reader.hpp>
#include <visualization/visualization.hpp>

#include <engine/onnx_engine.hpp>
#include <fusion/longitudinal_fusion.hpp>
#include <fusion/lateral_fusion.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/auto_speed.hpp>

#include <opencv2/opencv.hpp>

#include <array>
#include <chrono>
#include <cstring>
#include <future>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace vm = visionpilot::models;
namespace vf = visionpilot::fusion;
namespace ve = visionpilot::engine;
namespace vd = visionpilot::debug;

// ─────────────────────────────────────────────────────────────────────────────
// Circular frame buffer — last N BGR frames for AutoDrive's 2-frame input
// ─────────────────────────────────────────────────────────────────────────────
template<int N>
class CircularFrameBuffer {
public:
    void push(const cv::Mat& f) {
        buf_[head_] = f.clone();
        head_       = (head_ + 1) % N;
        count_      = std::min(count_ + 1, N);
    }
    const cv::Mat& operator[](int i) const { return buf_[(head_ - 1 - i + N * 2) % N]; }
    bool ready() const { return count_ >= N; }
    void clear()       { count_ = 0; head_ = 0; }
private:
    std::array<cv::Mat, N> buf_;
    int head_ = 0, count_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// LatencyStats — EMA-smoothed per-model timings
// ─────────────────────────────────────────────────────────────────────────────
struct LatencyStats {
    double pre{0}, ad{0}, as{0}, asp{0}, wall{0};
    bool   ok{false};

    void update(double pre_, double ad_, double as_, double asp_, double wall_) {
        auto ema = [this](double& e, double v){ e = ok ? e*0.9 + v*0.1 : v; };
        ema(pre, pre_); ema(ad, ad_); ema(as, as_); ema(asp, asp_); ema(wall, wall_);
        ok = true;
    }
    void print() const {
        if (!ok) return;
        VP_INFO("Latency[EMA]  pre=%.1f  AD=%.1f  AS=%.1f  ASp=%.1f  wall=%.1f ms  (%.0f fps)",
                pre, ad, as, asp, wall, 1000.0 / wall);
    }
    void reset() { *this = {}; }
};

// ─────────────────────────────────────────────────────────────────────────────
// InferencePipeline — model inference + fusion
// ─────────────────────────────────────────────────────────────────────────────
class InferencePipeline {
public:
    static constexpr int NET_W    = vm::AutoDrive::NET_W;
    static constexpr int NET_H    = vm::AutoDrive::NET_H;
    static constexpr int CHW_SIZE = 3 * NET_W * NET_H;

    InferencePipeline(ve::OnnxEngine& engine, const VisionPilotConfig& cfg)
        : auto_drive_(engine, cfg.autodrive_model)
        , auto_steer_(engine, cfg.autosteer_model)
        , auto_speed_(engine, cfg.autospeed_model)
        , wheel_dir_(cfg.wheel_dir)
        , homography_path_(cfg.homography_path)
    {
        vf::LongitudinalFusion::Config lc;
        lc.homography_path = cfg.homography_path;
        lc.debug           = cfg.fusion_debug;
        long_fusion_ = vf::LongitudinalFusion{lc};

        vf::LateralFusion::Config latc;
        latc.homography_path = cfg.homography_path;
        latc.debug           = cfg.fusion_debug;
        lat_fusion_ = vf::LateralFusion{latc};
    }

    // Returns a filled DebugView ready for annotate_frame, or nullopt when
    // the circular buffer hasn't collected 2 frames yet.
    std::optional<vd::DebugView> process(const cv::Mat& preprocessed,
                                          const cv::Mat& /*original*/,
                                          const std::string& src_label)
    {
        using Clock = std::chrono::steady_clock;
        using Ms    = std::chrono::duration<double, std::milli>;

        frames_.push(preprocessed);
        ++frame_count_;
        if (!frames_.ready()) return std::nullopt;

        // ── Build CHW float buffers ───────────────────────────────────────────
        auto t0 = Clock::now();
        auto prev_imn = chw_imagenet(frames_[1]);
        auto curr_imn = chw_imagenet(frames_[0]);
        auto curr_01  = chw_01(frames_[0]);
        const double ms_pre = Ms(Clock::now() - t0).count();

        // ── Parallel inference ────────────────────────────────────────────────
        auto t_wall = Clock::now();
        auto f_drive = std::async(std::launch::async, [&] {
            auto t = Clock::now();
            return std::make_pair(auto_drive_.infer(prev_imn.data(), curr_imn.data()),
                                  Ms(Clock::now() - t).count());
        });
        auto f_steer = std::async(std::launch::async, [&] {
            auto t = Clock::now();
            return std::make_pair(auto_steer_.infer(curr_01.data()),
                                  Ms(Clock::now() - t).count());
        });
        auto f_speed = std::async(std::launch::async, [&] {
            auto t = Clock::now();
            return std::make_pair(auto_speed_.infer(curr_01.data()),
                                  Ms(Clock::now() - t).count());
        });

        auto [res_drive, ms_drive] = f_drive.get();
        auto [res_steer, ms_steer] = f_steer.get();
        auto [res_speed, ms_speed] = f_speed.get();
        const double ms_wall = Ms(Clock::now() - t_wall).count();

        // ── Fusion ────────────────────────────────────────────────────────────
        const auto cipo    = long_fusion_.update(res_drive, res_speed, preprocessed);
        const auto lateral = lat_fusion_.update(res_steer, res_drive);

        stats_.update(ms_pre, ms_drive, ms_steer, ms_speed, ms_wall);

        vd::DebugView view;
        view.frame_id      = frame_count_;
        view.wall_ms       = ms_wall;
        view.pre_ms        = ms_pre;
        view.ad_ms         = ms_drive;
        view.as_ms         = ms_steer;
        view.asp_ms        = ms_speed;
        view.src_label     = src_label;
        view.auto_drive    = res_drive;
        view.auto_steer    = res_steer;
        view.auto_speed    = res_speed;
        view.cipo          = cipo;
        view.lateral       = lateral;
        view.wheel_dir        = wheel_dir_;
        view.homography_path  = homography_path_;

        return view;
    }

    void reset() {
        frames_.clear();
        frame_count_ = 0;
        stats_.reset();
        long_fusion_.reset();
        lat_fusion_.reset();
    }

    const LatencyStats& latency() const { return stats_; }

private:
    static constexpr float MEAN[3] = {0.485f, 0.456f, 0.406f};
    static constexpr float STD[3]  = {0.229f, 0.224f, 0.225f};

    static std::vector<float> chw_imagenet(const cv::Mat& bgr) {
        cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::Mat f32; rgb.convertTo(f32, CV_32FC3, 1.0/255.0);
        std::vector<cv::Mat> ch(3); cv::split(f32, ch);
        std::vector<float> out(static_cast<std::size_t>(CHW_SIZE));
        for (int c = 0; c < 3; ++c) {
            float*       dst = out.data() + c * NET_H * NET_W;
            const float* src = reinterpret_cast<const float*>(ch[c].data);
            for (int i = 0; i < NET_H * NET_W; ++i) dst[i] = (src[i] - MEAN[c]) / STD[c];
        }
        return out;
    }

    static std::vector<float> chw_01(const cv::Mat& bgr) {
        cv::Mat rgb; cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
        cv::Mat f32; rgb.convertTo(f32, CV_32FC3, 1.0/255.0);
        std::vector<cv::Mat> ch(3); cv::split(f32, ch);
        std::vector<float> out(static_cast<std::size_t>(CHW_SIZE));
        for (int c = 0; c < 3; ++c)
            std::memcpy(out.data() + c * NET_H * NET_W, ch[c].data,
                        static_cast<std::size_t>(NET_H * NET_W) * sizeof(float));
        return out;
    }

    vm::AutoDrive          auto_drive_;
    vm::AutoSteer          auto_steer_;
    vm::AutoSpeed          auto_speed_;
    vf::LongitudinalFusion long_fusion_;
    vf::LateralFusion      lat_fusion_;
    CircularFrameBuffer<2> frames_;
    uint64_t               frame_count_ = 0;
    LatencyStats           stats_;
    std::string            wheel_dir_;
    std::string            homography_path_;
};

// ─────────────────────────────────────────────────────────────────────────────
// Preprocessing — center-crop to 50° HFOV, resize to NET_W × NET_H
// (mirrors center_crop_50deg_resize_with_metadata in zod_manual_ground_homography.py)
// ─────────────────────────────────────────────────────────────────────────────
static cv::Mat preprocess(const cv::Mat& raw, float hfov_deg)
{
    static constexpr float TARGET_FOV = 50.f;
    const int img_w = raw.cols, img_h = raw.rows;
    int crop_w = static_cast<int>(std::lround(img_w * TARGET_FOV / hfov_deg));
    int crop_h = crop_w / 2;
    crop_w = std::clamp(crop_w, 1, img_w);
    crop_h = std::clamp(crop_h, 1, img_h);
    const cv::Rect roi((img_w - crop_w) / 2, (img_h - crop_h) / 2, crop_w, crop_h);
    cv::Mat out;
    cv::resize(raw(roi), out,
               cv::Size(InferencePipeline::NET_W, InferencePipeline::NET_H),
               0, 0, cv::INTER_LINEAR);
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Source mode runners — one liners
// ─────────────────────────────────────────────────────────────────────────────

static int run_video(InferencePipeline& pipeline,
                     const VisionPilotConfig& cfg,
                     const std::string& path)
{
    cv::VideoCapture cap(path);
    if (!cap.isOpened()) { VP_ERROR("Cannot open video: %s", path.c_str()); return 1; }

    VP_INFO("Video: %s  %.0f fps  %dx%d  crop_hfov=%.0f→50  realtime=%s  loop=%s",
            path.c_str(), cap.get(cv::CAP_PROP_FPS),
            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT)),
            cfg.source.hfov_deg,
            cfg.source.video_realtime ? "yes" : "no",
            cfg.source.video_loop     ? "yes" : "no");

    VP_INFO("GPU warmup...");
    { cv::Mat f;
      for (int i = 0; i < 4 && cap.read(f); ++i)
          pipeline.process(preprocess(f, cfg.source.hfov_deg), f, "warmup");
      cap.set(cv::CAP_PROP_POS_FRAMES, 0); pipeline.reset(); }

    if (cfg.pipeline.initial_inference_check) {
        cv::Mat f1, f2;
        if (cap.read(f1) && cap.read(f2)) {
            pipeline.process(preprocess(f1, cfg.source.hfov_deg), f1, "check");
            if (const auto r = pipeline.process(preprocess(f2, cfg.source.hfov_deg), f2, "check"); r)
                VP_INFO("Initial check OK — d=%.1f m  v=%.2f m/s  wall=%.1f ms",
                        r->cipo.distance_m, r->cipo.velocity_ms, r->wall_ms);
        }
        cap.set(cv::CAP_PROP_POS_FRAMES, 0); pipeline.reset();
    }

    const double fps    = cap.get(cv::CAP_PROP_FPS);
    const auto   period = (cfg.source.video_realtime && fps > 1.0)
                          ? std::chrono::duration<double>(1.0 / fps)
                          : std::chrono::duration<double>(0);
    cv::Mat frame;
    for (;;) {
        const auto t0 = std::chrono::steady_clock::now();
        if (!cap.read(frame) || frame.empty()) {
            if (cfg.source.video_loop) { cap.set(cv::CAP_PROP_POS_FRAMES, 0); pipeline.reset(); continue; }
            VP_INFO("End of video."); break;
        }

        cv::Mat prep   = preprocess(frame, cfg.source.hfov_deg);
        auto    result = pipeline.process(prep, frame, "video");

        if (result) {
            if (result->frame_id % 30 == 0) pipeline.latency().print();
            vd::annotate_frame(prep, *result);
        }
        visualization::render_frame(prep, "VisionPilot",
            result ? std::vector<std::string>{} : std::vector<std::string>{"warming up..."});

        if (period.count() > 0) {
            const auto rem = period - (std::chrono::steady_clock::now() - t0);
            if (rem.count() > 0) std::this_thread::sleep_for(rem);
        }
    }
    visualization::close_windows();
    return 0;
}

static void run_ros2(InferencePipeline& pipeline,
                     const VisionPilotConfig& cfg,
                     const std::string& topic)
{
    VP_INFO("ROS2 mode | topic: %s", topic.c_str());
    camera_subscriber::ROS2ImageSubscriber sub(topic);
    for (;;) {
        auto [ok, frame] = sub.get_latest_frame();
        if (!ok || frame.empty()) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }

        cv::Mat prep   = preprocess(frame, cfg.source.hfov_deg);
        auto    result = pipeline.process(prep, frame, topic);
        if (!result) continue;

        if (result->frame_id % 30 == 0) pipeline.latency().print();
        vd::annotate_frame(prep, *result);
        visualization::render_frame(prep, "VisionPilot", {});
    }
    visualization::close_windows();
}

static void run_v4l2(InferencePipeline& pipeline,
                     const VisionPilotConfig& cfg,
                     const std::string& device, int fps)
{
    VP_INFO("V4L2 mode | device: %s  fps: %d", device.c_str(), fps);
    v4l2_interface::V4L2Reader reader(device, static_cast<uint32_t>(fps));
    if (!reader.is_device_open()) { VP_ERROR("Failed to open: %s", device.c_str()); return; }
    for (;;) {
        auto [ok, frame] = reader.get_latest_frame();
        if (!ok || frame.empty()) continue;

        cv::Mat prep   = preprocess(frame, cfg.source.hfov_deg);
        auto    result = pipeline.process(prep, frame, device);
        if (!result) continue;

        if (result->frame_id % 30 == 0) pipeline.latency().print();
        vd::annotate_frame(prep, *result);
        visualization::render_frame(prep, "VisionPilot", {});
    }
    visualization::close_windows();
}

// ─────────────────────────────────────────────────────────────────────────────
// main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    const std::string cfg_path = resolve_vision_pilot_config_path(argc, argv);
    if (cfg_path.empty()) {
        VP_ERROR("No config file found.");
        VP_ERROR("  cp config/vision_pilot.conf.example config/vision_pilot.conf");
        VP_ERROR("  or: export VISIONPILOT_CONFIG=/path/to/vision_pilot.conf");
        return 1;
    }

    VisionPilotConfig cfg;
    try { cfg = load_vision_pilot_config(cfg_path); }
    catch (const std::exception& e) { VP_ERROR("Config: %s", e.what()); return 1; }

    VP_INFO("Config    : %s", cfg_path.c_str());
    VP_INFO("AutoDrive : %s", cfg.autodrive_model.c_str());
    VP_INFO("AutoSteer : %s", cfg.autosteer_model.c_str());
    VP_INFO("AutoSpeed : %s", cfg.autospeed_model.c_str());
    VP_INFO("Provider  : %s", cfg.engine_cfg.provider.c_str());
    VP_INFO("Homography: %s", cfg.homography_path.empty() ? "(none)" : cfg.homography_path.c_str());
    VP_INFO("FusionDbg : %s", cfg.fusion_debug ? "on" : "off");

    ve::OnnxEngine    engine(cfg.engine_cfg);
    InferencePipeline pipeline(engine, cfg);
    vd::init_wheel_assets(cfg.wheel_dir);
    vd::init_homography(cfg.homography_path);

    switch (cfg.source.mode) {
        case SourceMode::Video: return run_video(pipeline, cfg, cfg.source.video_path);
        case SourceMode::Ros2:  run_ros2(pipeline, cfg, cfg.source.ros2_topic); break;
        case SourceMode::V4l2:  run_v4l2(pipeline, cfg, cfg.source.v4l2_device, cfg.source.v4l2_fps); break;
    }
    return 0;
}
