#pragma once

#include <models/auto_drive.hpp>
#include <models/auto_speed.hpp>
#include <opencv2/core.hpp>

#include <random>
#include <string>
#include <vector>

namespace visionpilot::fusion {

// ─── Output ────────────────────────────────────────────────────────────────────
struct CIPOFusionEstimate {
    bool  valid             = false;

    // Particle-filter fused posterior
    float distance_m        = 0.f;
    float velocity_ms       = 0.f;   // negative = approaching; derived as Δd/Δt
    float distance_stddev_m = 0.f;

    // Raw homography distance from AutoSpeed bboxes (no tracking state)
    bool  homo_found        = false;
    float homo_dist_m       = 0.f;
};

// ─── LongitudinalFusion ────────────────────────────────────────────────────────
//
//  Per-frame CIPO longitudinal estimation:
//    1. If homography_path is set: project AutoSpeed bbox bottom-centres through H
//       to world space and take the closest in-front detection as a distance measure.
//    2. Particle filter (state: [distance_m, velocity_ms]) fuses AutoDrive output
//       and the homography distance, using log-weight accumulation (MRPT style).
//    3. Velocity is reported as an EMA-smoothed finite difference of the fused
//       distance, NOT from the PF velocity particles (which are unreliable when
//       only distance is observed and measurements are noisy).
//
class LongitudinalFusion {
public:
    struct Config {
        int   n_particles          = 500;
        float d_max_m              = 150.f;
        float dt_s                 = 0.10f;   // nominal dt; overridden per-call
        float process_noise_dist_m = 0.50f;
        float process_noise_vel_ms = 0.20f;   // used inside PF predict only
        float autodrive_noise_m    = 15.f;
        float homo_noise_m         = 5.f;     // homography distance 1-sigma noise
        // EMA factor for velocity smoothing (higher = more responsive, noisier).
        float velocity_ema_alpha   = 0.3f;
        std::string homography_path = "";
        bool debug = false;
    };

    LongitudinalFusion();
    explicit LongitudinalFusion(Config cfg);

    CIPOFusionEstimate update(
        const models::AutoDriveOutput& autodrive,
        const models::AutoSpeedOutput& autospeed,
        const cv::Mat& preprocessed_frame,
        float dt_s = 0.f);

    void reset();
    const Config& config() const { return cfg_; }

private:
    struct Particle { float distance_m, velocity_ms, log_w; };
    struct Meas     { float distance_m = 0.f; float stddev_m = 15.f; bool valid = false; };

    // Returns the closest in-path distance from AutoSpeed detections via homography.
    // Returns valid=false when no suitable detection exists or homography not loaded.
    Meas project_closest(const std::vector<models::Detection>& dets) const;

    void  init_from(float dist_m, float stddev_m);
    void  predict(float dt_s);
    void  weight_update(const Meas& ad, const Meas& homo);
    std::vector<float> linear_weights() const;
    float effective_n() const;
    void  resample();
    static float gaussian_loglik(float z, float mean, float sigma);

    Config cfg_;
    std::vector<Particle> particles_;
    bool   initialised_   = false;
    float  prev_fused_d_  = -1.f;  // for velocity finite difference
    float  velocity_ema_  = 0.f;
    std::mt19937 rng_;

    // Homography — loaded once from cfg_.homography_path
    cv::Mat H_;
    bool    homo_loaded_ = false;
};

}  // namespace visionpilot::fusion
