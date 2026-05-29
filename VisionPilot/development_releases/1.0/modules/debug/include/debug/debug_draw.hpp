#pragma once

#include <fusion/longitudinal_fusion.hpp>
#include <fusion/lateral_fusion.hpp>
#include <models/auto_drive.hpp>
#include <models/auto_steer.hpp>
#include <models/auto_speed.hpp>
#include <opencv2/core.hpp>

#include <string>

namespace visionpilot::debug {

// Vehicle / AutoDrive defaults (match Models/visualizations/AutoDrive/image_visualization.py)
struct VehicleParams {
    float wheelbase_m     = 2.984f;
    float steer_ratio     = 16.8f;
    float flag_threshold  = 0.65f;
    float curv_scale      = 0.21f;   // CURV_SCALE from load_data_auto_drive.py
};

// ─── Per-frame bundle passed to annotate_frame ────────────────────────────────
struct DebugView {
    uint64_t    frame_id   = 0;
    double      wall_ms    = 0;
    double      pre_ms     = 0;
    double      ad_ms      = 0;
    double      as_ms      = 0;
    double      asp_ms     = 0;
    std::string src_label;

    models::AutoDriveOutput       auto_drive;
    models::AutoSteerOutput       auto_steer;
    models::AutoSpeedOutput       auto_speed;

    fusion::CIPOFusionEstimate    cipo;
    fusion::LateralFusionEstimate   lateral;

    VehicleParams vehicle;
    // Directory with wheel_white.png / wheel_green.png (0.9/images by default)
    std::string wheel_dir;
};

// Load wheel PNGs once (idempotent). Call from main after config is known.
void init_wheel_assets(const std::string& wheel_dir);

// Draws onto 1024×512 BGR frame:
//   • AutoSpeed boxes (L1 red, L2 yellow, L3 cyan)
//   • AutoSteer ego path — 64 waypoints × h_vector mask (green dots + polyline)
//   • Steering wheels: green = fused κ, white = AutoDrive κ
//   • 3-column HUD
void annotate_frame(cv::Mat& frame, const DebugView& view);

}  // namespace visionpilot::debug
