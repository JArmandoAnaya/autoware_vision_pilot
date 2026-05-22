#include <fusion/longitudinal_fusion.hpp>
#include <logging/logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace visionpilot::fusion {

// ─── Homography loader ────────────────────────────────────────────────────────
// Parses the YAML produced by OpenCV FileStorage or the manual calibration tool:
//   H:
//     rows: 3
//     cols: 3
//     data: [v0, v1, ..., v8]   (or one value per line with leading '-')
static cv::Mat load_homography_yaml(const std::string& path)
{
    std::ifstream f(path);
    if (!f.is_open())
        throw std::runtime_error("LongitudinalFusion: cannot open homography: " + path);

    std::vector<double> data;
    bool in_data = false;
    std::string line;
    while (std::getline(f, line) && data.size() < 9) {
        if (line.find("data:") != std::string::npos) {
            in_data = true;
            auto lb = line.find('[');
            if (lb != std::string::npos) {
                auto rb = line.find(']', lb);
                std::string seq = line.substr(lb + 1, rb - lb - 1);
                std::replace(seq.begin(), seq.end(), ',', ' ');
                std::istringstream ss(seq);
                double v;
                while (ss >> v) data.push_back(v);
                break;
            }
            continue;
        }
        if (in_data) {
            auto dash = line.find('-');
            if (dash == std::string::npos) continue;
            try { data.push_back(std::stod(line.substr(dash + 1))); } catch (...) {}
        }
    }
    if (data.size() != 9)
        throw std::runtime_error("LongitudinalFusion: expected 9 homography values, got " +
                                 std::to_string(data.size()));
    cv::Mat H64(3, 3, CV_64F, data.data());
    cv::Mat H32;
    H64.convertTo(H32, CV_32F);
    return H32.clone();
}

// ─── Construction ─────────────────────────────────────────────────────────────

LongitudinalFusion::LongitudinalFusion()
    : LongitudinalFusion(Config{})
{}

LongitudinalFusion::LongitudinalFusion(Config cfg)
    : cfg_(cfg)
    , rng_(std::random_device{}())
{
    if (cfg_.n_particles < 10)
        throw std::invalid_argument("LongitudinalFusion: n_particles must be >= 10");
    particles_.reserve(static_cast<std::size_t>(cfg_.n_particles));
}

// ─── Public API ───────────────────────────────────────────────────────────────

void LongitudinalFusion::reset()
{
    particles_.clear();
    initialised_  = false;
    prev_fused_d_ = -1.f;
    velocity_ema_ = 0.f;
    homo_loaded_  = false;
    H_            = cv::Mat();
}

CIPOFusionEstimate LongitudinalFusion::update(
    const models::AutoDriveOutput& autodrive,
    const models::AutoSpeedOutput& autospeed,
    const cv::Mat& /*preprocessed_frame*/,
    float dt_s)
{
    // ── Step 1: Lazy-load homography ──────────────────────────────────────────
    if (!homo_loaded_ && !cfg_.homography_path.empty()) {
        try {
            H_          = load_homography_yaml(cfg_.homography_path);
            homo_loaded_ = true;
            VP_INFO("[Fusion] Homography loaded: %s", cfg_.homography_path.c_str());
        } catch (const std::exception& e) {
            VP_WARN("[Fusion] %s — running without homography", e.what());
            cfg_.homography_path.clear();
        }
    }

    // ── Step 2: AutoSpeed → world distance via homography ────────────────────
    // No tracking state — just project each bbox bottom-centre and pick closest.
    CIPOFusionEstimate est;
    Meas homo_meas;
    if (homo_loaded_ && autospeed.valid) {
        homo_meas = project_closest(autospeed.detections);
        if (homo_meas.valid) {
            est.homo_found  = true;
            est.homo_dist_m = homo_meas.distance_m;
        }
    }

    // ── Step 3: AutoDrive distance ────────────────────────────────────────────
    static constexpr float D_MAX = 150.f;
    Meas ad_meas;
    if (autodrive.valid) {
        ad_meas.distance_m = D_MAX * (1.f - autodrive.dist_normalized);
        ad_meas.stddev_m   = cfg_.autodrive_noise_m;
        ad_meas.valid      = true;
    }

    // ── Step 4: Particle filter ───────────────────────────────────────────────
    const float dt = (dt_s > 1e-6f) ? dt_s : cfg_.dt_s;

    if (!initialised_) {
        if (!ad_meas.valid) return est;
        init_from(ad_meas.distance_m, ad_meas.stddev_m);
        initialised_ = true;
    } else {
        predict(dt);
    }

    weight_update(ad_meas, homo_meas);
    if (effective_n() < 0.5f * static_cast<float>(cfg_.n_particles)) resample();

    // ── Step 5: Posterior distance (weighted mean + stddev) ───────────────────
    const auto   w      = linear_weights();
    const auto   N      = particles_.size();
    float mean_d = 0.f;
    for (std::size_t i = 0; i < N; ++i) mean_d += w[i] * particles_[i].distance_m;
    float var_d  = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        const float dd = particles_[i].distance_m - mean_d;
        var_d += w[i] * dd * dd;
    }

    // ── Step 6: Velocity = EMA-smoothed finite difference of fused distance ───
    // The PF velocity particles are unreliable when only distance is observed and
    // measurements are noisy (they accumulate a biased random walk).  A simple
    // numerical derivative is far more consistent with the displayed distances.
    float vel = 0.f;
    if (prev_fused_d_ >= 0.f && dt > 1e-6f) {
        const float raw_vel = (mean_d - prev_fused_d_) / dt;
        velocity_ema_ = cfg_.velocity_ema_alpha * raw_vel
                      + (1.f - cfg_.velocity_ema_alpha) * velocity_ema_;
        vel = velocity_ema_;
    }
    prev_fused_d_ = mean_d;

    est.valid             = true;
    est.distance_m        = mean_d;
    est.velocity_ms       = vel;
    est.distance_stddev_m = std::sqrt(std::max(0.f, var_d));

    // ── Step 7: Debug log ─────────────────────────────────────────────────────
    if (cfg_.debug) {
        char ad_buf[32], ho_buf[32];
        if (ad_meas.valid)
            std::snprintf(ad_buf, sizeof(ad_buf), "%.1f m", ad_meas.distance_m);
        else
            std::snprintf(ad_buf, sizeof(ad_buf), "(invalid)");
        if (homo_meas.valid)
            std::snprintf(ho_buf, sizeof(ho_buf), "%.1f m", homo_meas.distance_m);
        else
            std::snprintf(ho_buf, sizeof(ho_buf), "(none)");

        VP_INFO("[Fusion] AD=%s | Homo=%s | Fused=%.1f m  v=%.2f m/s  ±%.1f m",
                ad_buf, ho_buf, est.distance_m, est.velocity_ms, est.distance_stddev_m);
    }

    return est;
}

// ─── Homography projection ────────────────────────────────────────────────────

LongitudinalFusion::Meas
LongitudinalFusion::project_closest(const std::vector<models::Detection>& dets) const
{
    Meas best;
    best.valid      = false;
    best.distance_m = std::numeric_limits<float>::max();
    best.stddev_m   = cfg_.homo_noise_m;

    // ZOD convention: world_y is negative for objects ahead.  A vehicle 30m
    // ahead projects to roughly (x≈0, y≈-30).  Distance = |y|.
    for (const auto& d : dets) {
        const float ux = (d.x1 + d.x2) * 0.5f;
        const float uy = d.y2;

        std::vector<cv::Point2f> src = {cv::Point2f(ux, uy)}, dst;
        cv::perspectiveTransform(src, dst, H_);
        const cv::Point2f& wp = dst[0];

        if (wp.y >= -1.f) continue;  // not in front of ego (at least 1 m ahead)

        const float dist = -wp.y;    // forward distance in metres
        if (dist < best.distance_m) {
            best.distance_m = dist;
            best.valid      = true;
        }
    }
    return best;
}

// ─── Particle filter internals ────────────────────────────────────────────────

void LongitudinalFusion::init_from(float dist_m, float stddev_m)
{
    particles_.resize(static_cast<std::size_t>(cfg_.n_particles));
    std::normal_distribution<float> nd(dist_m, stddev_m);
    std::normal_distribution<float> nv(0.f, 2.f);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(nd(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = nv(rng_);
        p.log_w       = 0.f;
    }
}

void LongitudinalFusion::predict(float dt_s)
{
    std::normal_distribution<float> nd(0.f, cfg_.process_noise_dist_m);
    std::normal_distribution<float> nv(0.f, cfg_.process_noise_vel_ms);
    for (auto& p : particles_) {
        p.distance_m  = std::clamp(p.distance_m + p.velocity_ms * dt_s + nd(rng_), 0.f, cfg_.d_max_m);
        p.velocity_ms = p.velocity_ms + nv(rng_);
    }
}

float LongitudinalFusion::gaussian_loglik(float z, float mean, float sigma)
{
    const float d = z - mean;
    return -0.5f * (d / sigma) * (d / sigma);
}

void LongitudinalFusion::weight_update(const Meas& ad, const Meas& homo)
{
    for (auto& p : particles_) {
        if (ad.valid)   p.log_w += gaussian_loglik(ad.distance_m,   p.distance_m, ad.stddev_m);
        if (homo.valid) p.log_w += gaussian_loglik(homo.distance_m, p.distance_m, homo.stddev_m);
    }
}

std::vector<float> LongitudinalFusion::linear_weights() const
{
    const std::size_t N = particles_.size();
    float max_lw = particles_[0].log_w;
    for (const auto& p : particles_) max_lw = std::max(max_lw, p.log_w);

    std::vector<float> w(N);
    float sum = 0.f;
    for (std::size_t i = 0; i < N; ++i) {
        w[i] = std::exp(particles_[i].log_w - max_lw);
        sum  += w[i];
    }
    if (sum < 1e-12f) {
        const float w0 = 1.f / static_cast<float>(N);
        for (auto& wi : w) wi = w0;
    } else {
        for (auto& wi : w) wi /= sum;
    }
    return w;
}

float LongitudinalFusion::effective_n() const
{
    const auto w = linear_weights();
    float ss = 0.f;
    for (auto wi : w) ss += wi * wi;
    return 1.f / (ss + 1e-12f);
}

void LongitudinalFusion::resample()
{
    const int N = static_cast<int>(particles_.size());
    if (N == 0) return;

    const auto w = linear_weights();
    std::vector<float> cs(static_cast<std::size_t>(N));
    cs[0] = w[0];
    for (int i = 1; i < N; ++i)
        cs[static_cast<std::size_t>(i)] =
            cs[static_cast<std::size_t>(i-1)] + w[static_cast<std::size_t>(i)];

    std::vector<Particle> np;
    np.reserve(static_cast<std::size_t>(N));
    std::uniform_real_distribution<float> u(0.f, 1.f / static_cast<float>(N));
    const float u0 = u(rng_);
    int j = 0;
    for (int i = 0; i < N; ++i) {
        const float thr = u0 + static_cast<float>(i) / static_cast<float>(N);
        while (j < N-1 && cs[static_cast<std::size_t>(j)] < thr) ++j;
        np.push_back({particles_[static_cast<std::size_t>(j)].distance_m,
                      particles_[static_cast<std::size_t>(j)].velocity_ms,
                      0.f});
    }
    particles_ = std::move(np);
}

}  // namespace visionpilot::fusion
