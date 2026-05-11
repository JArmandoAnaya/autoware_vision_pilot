#pragma once

#include <array>
#include <vector>

namespace visionpilot::models {

// ─── AutoDrive ────────────────────────────────────────────────────────────────
// Raw outputs from the AutoDrive two-frame unified model.
// Domain conversion is the caller's responsibility:
//   distance_m    = D_MAX_M * (1.0f - dist_normalized)
//   curvature_1pm = curvature_raw * CURV_SCALE
struct AutoDriveOutput {
    float dist_normalized = 0.f;  // normalised distance [0, 1]
    float curvature_raw   = 0.f;  // raw model curvature output
    float flag_prob       = 0.f;  // sigmoid(flag_logit), CIPO probability [0, 1]
    bool  valid           = false;
};

// ─── AutoSteer ────────────────────────────────────────────────────────────────
// Raw outputs from the AutoSteer single-frame path prediction model.
// xp and h_vector are (2 × 64) tensors flattened row-major:
//   indices [0 .. 63]   = row 0
//   indices [64 .. 127] = row 1
struct AutoSteerOutput {
    std::array<float, 128> xp{};        // (2, 64) ego-path waypoints
    std::array<float, 128> h_vector{};  // (2, 64) homography vectors
    bool                   valid = false;
};

// ─── AutoSpeed ────────────────────────────────────────────────────────────────
struct Detection {
    float x1 = 0.f, y1 = 0.f;  // top-left  (model-input pixel space)
    float x2 = 0.f, y2 = 0.f;  // bottom-right
    float score    = 0.f;
    int   class_id = 0;
};

struct AutoSpeedOutput {
    std::vector<Detection> detections;
    bool valid = false;
};

}  // namespace visionpilot::models
