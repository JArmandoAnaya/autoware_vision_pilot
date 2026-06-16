#pragma once

#include <engine/onnx_engine.hpp>
#include <string>

namespace vpe = visionpilot::engine;

enum class SourceMode { Ros2 = 0, V4l2 = 1, Video = 2 };

struct SourceConfig {
    SourceMode  mode         = SourceMode::Video;
    std::string video_path;
    bool        video_realtime = true;
    bool        video_loop     = false;
    std::string ros2_topic   = "/camera/image";
    std::string v4l2_device  = "/dev/video0";
    int         v4l2_fps     = 10;
};

struct PipelineConfig {
    bool initial_inference_check = true;
};

// Control / actuation (ROS2 drive path). Topic names are middleware-agnostic
// defaults — point them at whatever consumer is in use (a real vehicle, CARLA,
// another simulator) via the config file. The speed envelope feeds the
// longitudinal controller.
struct ControlConfig {
    std::string topic               = "/control/ackermann_cmd";
    std::string vehicle_state_topic = "/vehicle/odometry";
    double      max_speed_mps       = 60.0 / 3.6;  // 60 km/h
    double      min_cruise_mps      = 0.0;

    // Invert the published steering sign. VisionPilot uses the ROS/REP-103
    // convention (+angle = left, matching +curvature = left). The official
    // carla_ros_bridge negates the angle for CARLA (control.steer = -angle/max),
    // but CARLA's NATIVE --ros2 ackermann path forwards it unchanged
    // (control.steer = +angle => +angle turns RIGHT). So when publishing to the
    // native path, set this true to restore the correct turn direction.
    bool        invert_steering     = false;

    // Publish a target speed only and forward zero acceleration, instead of the
    // planner's computed acceleration. DEFAULT false = agnostic: forward the real
    // acceleration so the planner's RSS following-distance / curve braking reaches
    // the actuator. Set true only for sinks whose Ackermann speed-tracking would
    // otherwise dead-stop the vehicle on the planner's deceleration (e.g. CARLA's
    // native --ros2 controller); the steering then carries the turn and the speed
    // floor (min_cruise_mps) keeps the vehicle moving.
    bool        track_speed_only    = false;

    // Optional readiness handshake. When non-empty, VisionPilot writes this file
    // once, on its first valid control command — a signal that perception+control
    // is live. Default empty = disabled (agnostic). External orchestration (e.g. a
    // simulator spawn helper) can hold the vehicle stationary until this appears,
    // so the actuator is not left uncommanded during model load / engine warm-up.
    std::string ready_sentinel_path = "";
};

struct VisionPilotConfig {
    std::string       autodrive_model;
    std::string       autosteer_model;
    std::string       autospeed_model;
    vpe::EngineConfig engine_cfg;
    SourceConfig      source;
    PipelineConfig    pipeline;
    ControlConfig     control;
    // Path to homography YAML — enables ObjectFinder tracker when non-empty.
    std::string       homography_path;
    // Print per-frame fusion debug logs
    bool              fusion_debug   = false;
    // Directory with wheel_white.png / wheel_green.png for steering HUD
    std::string       wheel_dir;
};

// Load from key=value .conf file. Expands ~ to $HOME.
// Throws std::runtime_error on missing or invalid config.
VisionPilotConfig load_vision_pilot_config(const std::string& config_path);

// Resolve config path from --config <path>, VISIONPILOT_CONFIG env var,
// or default candidates. Returns empty string if nothing found.
std::string resolve_vision_pilot_config_path(int argc, char** argv);

SourceMode parse_source_mode(const std::string& value);

// Short label for debug overlay (video / device path / ROS topic).
std::string source_label(const SourceConfig& source);
