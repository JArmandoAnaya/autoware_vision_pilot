#include <debug/debug_draw.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <vector>

namespace visionpilot::debug {

namespace fs = std::filesystem;

// ─── Layout (matches AutoSteer / AutoDrive Python visualizations) ─────────────
static constexpr int   kNetW      = 1024;
static constexpr int   kNetH      = 512;
static constexpr int   kPathPts   = 64;
static constexpr int   kWheelPx   = 92;

// AutoSpeed bbox colors (BGR)
static const cv::Scalar kClrL1        {  0,  0, 220};
static const cv::Scalar kClrL2        {  0, 210, 255};
static const cv::Scalar kClrL3        {220, 200,   0};
static const cv::Scalar kClrLOther    { 80, 200,  80};

static const cv::Scalar kClrEgoPath   {  0, 255,   0};   // green — video_visualization.py
static const cv::Scalar kClrHudBg     {  0,   0,   0};
static const cv::Scalar kClrHeader    {220, 220, 220};
static const cv::Scalar kClrNormal    {200, 200, 200};
static const cv::Scalar kClrFusedLong {  0, 220,   0};
static const cv::Scalar kClrFusedLat  {  0, 220, 255};  // yellow/cyan
static const cv::Scalar kClrTopBar    {200, 200, 200};
static const cv::Scalar kClrSteerLbl  {120, 240, 120};

static constexpr int    kFont  = cv::FONT_HERSHEY_SIMPLEX;
static constexpr double kSmall = 0.38;
static constexpr double kNorm  = 0.42;
static constexpr int    kThin  = 1;
static constexpr int    kBold  = 2;

// ─── Wheel assets (lazy load) ─────────────────────────────────────────────────
static std::mutex              g_wheel_mu;
static cv::Mat                 g_wheel_white;   // BGRA
static cv::Mat                 g_wheel_green;
static bool                    g_wheels_tried  = false;

static inline cv::Scalar det_color(int class_id) {
    switch (class_id) {
        case 1: return kClrL1;
        case 2: return kClrL2;
        case 3: return kClrL3;
        default: return kClrLOther;
    }
}

static inline std::string fd(float v, int d) {
    char b[24];
    std::snprintf(b, sizeof(b), "%.*f", d, static_cast<double>(v));
    return b;
}

static void fill_rect(cv::Mat& img, cv::Rect r, cv::Scalar color, double alpha)
{
    const cv::Rect clip = r & cv::Rect(0, 0, img.cols, img.rows);
    if (clip.width <= 0 || clip.height <= 0) return;
    cv::Mat roi = img(clip);
    cv::Mat block(roi.size(), roi.type(), color);
    cv::addWeighted(block, alpha, roi, 1.0 - alpha, 0.0, roi);
}

// κ [1/m] → road-wheel angle [deg] (image_visualization.py)
static float curvature_to_steer_deg(float curv_1pm,
                                    float wheelbase_m,
                                    float steer_ratio)
{
    return static_cast<float>(
        std::atan(curv_1pm * wheelbase_m) * steer_ratio * (180.0 / M_PI));
}

static cv::Mat load_wheel_rgba(const fs::path& path, int size_px)
{
    cv::Mat img = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (img.empty()) return {};
    cv::resize(img, img, cv::Size(size_px, size_px), 0, 0, cv::INTER_LINEAR);
    if (img.channels() == 3)
        cv::cvtColor(img, img, cv::COLOR_BGR2BGRA);
    return img;
}

static fs::path resolve_wheel_dir(const std::string& requested)
{
    if (!requested.empty() && fs::is_directory(requested))
        return requested;

    const fs::path cwd = fs::current_path();
    const std::vector<fs::path> candidates = {
        cwd / ".." / "0.9" / "images",
        cwd / ".." / ".." / "development_releases" / "0.9" / "images",
        cwd / "VisionPilot" / "development_releases" / "0.9" / "images",
        cwd / "VisionPilot" / "production_release" / "images",
        fs::path("VisionPilot/development_releases/0.9/images"),
        fs::path("VisionPilot/production_release/images"),
    };
    for (const auto& c : candidates) {
        if (fs::is_directory(c))
            return c;
    }
    return {};
}

void init_wheel_assets(const std::string& wheel_dir)
{
    std::lock_guard<std::mutex> lock(g_wheel_mu);
    if (g_wheels_tried) return;
    g_wheels_tried = true;

    const fs::path dir = resolve_wheel_dir(wheel_dir);
    if (dir.empty()) return;

    g_wheel_white = load_wheel_rgba(dir / "wheel_white.png", kWheelPx);
    g_wheel_green = load_wheel_rgba(dir / "wheel_green.png", kWheelPx);
}

static cv::Mat rotate_wheel_rgba(const cv::Mat& src, float angle_deg)
{
    if (src.empty()) return {};
    cv::Point2f center(src.cols / 2.f, src.rows / 2.f);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle_deg, 1.0);
    cv::Mat out;
    cv::warpAffine(src, out, rot, src.size(), cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0, 0));
    return out;
}

static void paste_rgba(cv::Mat& base, const cv::Mat& overlay, int x, int y)
{
    if (overlay.empty() || base.empty()) return;
    const int w = overlay.cols, h = overlay.rows;
    const int x1 = std::max(x, 0), y1 = std::max(y, 0);
    const int x2 = std::min(x + w, base.cols), y2 = std::min(y + h, base.rows);
    if (x2 <= x1 || y2 <= y1) return;

    cv::Mat roi = base(cv::Rect(x1, y1, x2 - x1, y2 - y1));
    cv::Mat src = overlay(cv::Rect(x1 - x, y1 - y, x2 - x1, y2 - y1));

    if (src.channels() == 4) {
        std::vector<cv::Mat> ch;
        cv::split(src, ch);
        cv::Mat rgb, alpha;
        cv::merge(std::vector<cv::Mat>{ch[0], ch[1], ch[2]}, rgb);
        alpha = ch[3];
        cv::Mat rgb_f, roi_f, alpha_f;
        rgb.convertTo(rgb_f, CV_32FC3, 1.0 / 255.0);
        roi.convertTo(roi_f, CV_32FC3, 1.0 / 255.0);
        alpha.convertTo(alpha_f, CV_32FC1, 1.0 / 255.0);
        cv::Mat alpha3;
        cv::cvtColor(alpha_f, alpha3, cv::COLOR_GRAY2BGR);
        cv::Mat blended = rgb_f.mul(alpha3) + roi_f.mul(cv::Scalar(1.f, 1.f, 1.f) - alpha3);
        blended.convertTo(roi, CV_8UC3, 255.0);
    } else {
        src.copyTo(roi);
    }
}

// ─── AutoSpeed detections ─────────────────────────────────────────────────────

static void draw_autospeed_detections(cv::Mat& img,
                                       const models::AutoSpeedOutput& speed)
{
    if (!speed.valid) return;
    for (const auto& d : speed.detections) {
        const cv::Scalar clr = det_color(d.class_id);
        const cv::Point  tl(static_cast<int>(d.x1), static_cast<int>(d.y1));
        const cv::Point  br(static_cast<int>(d.x2), static_cast<int>(d.y2));
        cv::rectangle(img, tl, br, clr, 2, cv::LINE_AA);
        char lbl[32];
        std::snprintf(lbl, sizeof(lbl), "L%d %.0f%%", d.class_id, d.score * 100.f);
        cv::putText(img, lbl, cv::Point(tl.x + 2, std::max(tl.y - 4, 10)),
                    kFont, 0.40, clr, 1, cv::LINE_AA);
    }
}

// ─── AutoSteer ego path (video_visualization.py) ─────────────────────────────
// xp/h_vector are (2, 64): row 0 and row 1 lateral samples at fixed image rows.
// y = linspace(0, H-1, 64); u = xp[row,i] * W; mask with h_vector >= 0.5

static void draw_autosteer_ego_path(cv::Mat& img,
                                     const models::AutoSteerOutput& steer)
{
    if (!steer.valid) return;

    const int W = img.cols > 0 ? img.cols : kNetW;
    const int H = img.rows > 0 ? img.rows : kNetH;

    // Fixed row indices — same as np.linspace(0, 511, 64)
    int y_pts[kPathPts];
    for (int i = 0; i < kPathPts; ++i)
        y_pts[i] = (kPathPts <= 1) ? 0
                   : static_cast<int>(std::lround(static_cast<double>(i) * (H - 1) / (kPathPts - 1)));

    static constexpr int kRows = 2;
    for (int row = 0; row < kRows; ++row) {
        std::vector<cv::Point> poly;
        for (int i = 0; i < kPathPts; ++i) {
            const int idx = row * kPathPts + i;
            if (steer.h_vector[idx] < 0.5f) continue;

            const int u = static_cast<int>(steer.xp[idx] * static_cast<float>(W));
            const int v = y_pts[i];
            if (u < 0 || u >= W || v < 0 || v >= H) continue;

            const cv::Point pt(u, v);
            cv::circle(img, pt, 3, kClrEgoPath, -1, cv::LINE_AA);
            poly.push_back(pt);
        }
        if (poly.size() >= 2)
            cv::polylines(img, poly, false, kClrEgoPath, 2, cv::LINE_AA);
    }
}

// ─── Steering wheels from curvature (image_visualization.py) ─────────────────

static void draw_steering_wheels(cv::Mat& img, const DebugView& v)
{
    const auto& veh = v.vehicle;

    float ad_curv = 0.f, ad_steer = 0.f;
    if (v.auto_drive.valid) {
        ad_curv   = v.auto_drive.curvature_raw * veh.curv_scale;
        ad_steer  = curvature_to_steer_deg(ad_curv, veh.wheelbase_m, veh.steer_ratio);
    }

    float fused_curv = 0.f, fused_steer = 0.f;
    if (v.lateral.valid) {
        fused_curv  = v.lateral.curvature;
        fused_steer = curvature_to_steer_deg(fused_curv, veh.wheelbase_m, veh.steer_ratio);
    }

    const int pad = std::max(8, static_cast<int>(std::min(img.cols, img.rows) * 0.01f));
    const int y   = pad;
    const int x_ad    = img.cols - pad - kWheelPx;
    const int x_fused = x_ad - pad - kWheelPx;

    std::lock_guard<std::mutex> lock(g_wheel_mu);
    if (!g_wheels_tried)
        init_wheel_assets(v.wheel_dir);

    // Green = fused lateral curvature; white = AutoDrive raw curvature
    if (!g_wheel_green.empty() && v.lateral.valid)
        paste_rgba(img, rotate_wheel_rgba(g_wheel_green, fused_steer), x_fused, y);
    if (!g_wheel_white.empty() && v.auto_drive.valid)
        paste_rgba(img, rotate_wheel_rgba(g_wheel_white, ad_steer), x_ad, y);

    if (v.lateral.valid)
        cv::putText(img, "Fused:" + fd(fused_steer, 1) + " deg",
                    cv::Point(x_fused + 2, y + kWheelPx + 16),
                    kFont, 0.44, kClrSteerLbl, 1, cv::LINE_AA);
    if (v.auto_drive.valid)
        cv::putText(img, "AD:" + fd(ad_steer, 1) + " deg",
                    cv::Point(x_ad + 2, y + kWheelPx + 16),
                    kFont, 0.44, kClrNormal, 1, cv::LINE_AA);

    // CIPO flag from AutoDrive
    if (v.auto_drive.valid) {
        const int flag_cls = (v.auto_drive.flag_prob >= veh.flag_threshold) ? 1 : 0;
        char flag_buf[48];
        std::snprintf(flag_buf, sizeof(flag_buf), "CIPO flag %d (p=%.2f)",
                      flag_cls, static_cast<double>(v.auto_drive.flag_prob));
        cv::putText(img, flag_buf, cv::Point(pad, y + kWheelPx + 36),
                    kFont, 0.40, kClrNormal, 1, cv::LINE_AA);
    }
}

// ─── HUD panel ───────────────────────────────────────────────────────────────

static void draw_top_bar(cv::Mat& img, const DebugView& v)
{
    fill_rect(img, cv::Rect(0, 0, img.cols, 20), kClrHudBg, 0.65);
    char buf[128];
    std::snprintf(buf, sizeof(buf),
                  "VisionPilot  #%llu  wall=%.1f ms (%.0f fps)  pre=%.1f ms  src=%s",
                  static_cast<unsigned long long>(v.frame_id),
                  v.wall_ms, (v.wall_ms > 0) ? 1000.0 / v.wall_ms : 0.0,
                  v.pre_ms, v.src_label.c_str());
    cv::putText(img, buf, cv::Point(6, 14), kFont, kSmall, kClrTopBar, kThin, cv::LINE_AA);
}

static void draw_hud_panel(cv::Mat& img, const DebugView& v)
{
    const int W  = img.cols;
    const int H  = img.rows;
    const int pH = 118;
    const int py = H - pH;

    fill_rect(img, cv::Rect(0, py, W, pH), kClrHudBg, 0.72);

    const int lineH = 18;
    const int c1x = 8, c2x = 345, c3x = 690;

    auto text = [&](int x, int y, const std::string& s, cv::Scalar clr,
                    double scale = -1, int thickness = kThin) {
        cv::putText(img, s, cv::Point(x, y), kFont,
                    scale < 0 ? kNorm : scale, clr, thickness, cv::LINE_AA);
    };

    static constexpr float D_MAX = 150.f;
    const auto& veh = v.vehicle;

    // Col 1 — MODELS
    int y = py + 14;
    text(c1x, y, "[ MODELS ]", kClrHeader, kSmall, kThin); y += lineH;

    if (v.auto_drive.valid) {
        const float d_m = D_MAX * (1.f - v.auto_drive.dist_normalized);
        const float curv = v.auto_drive.curvature_raw * veh.curv_scale;
        const float steer = curvature_to_steer_deg(curv, veh.wheelbase_m, veh.steer_ratio);
        text(c1x, y, "AutoDrive  d=" + fd(d_m, 1) + " m  k=" + fd(curv, 4)
             + "  steer=" + fd(steer, 1) + " deg  [" + fd(static_cast<float>(v.ad_ms), 1) + "ms]",
             kClrNormal, kSmall);
    } else {
        text(c1x, y, "AutoDrive  --", kClrNormal, kSmall);
    }
    y += lineH;

    if (v.auto_steer.valid)
        text(c1x, y, "AutoSteer  ego path  [" + fd(static_cast<float>(v.as_ms), 1) + "ms]",
             kClrNormal, kSmall);
    else
        text(c1x, y, "AutoSteer  --", kClrNormal, kSmall);
    y += lineH;

    if (v.auto_speed.valid)
        text(c1x, y, "AutoSpeed  dets=" + std::to_string(v.auto_speed.detections.size())
             + "  [" + fd(static_cast<float>(v.asp_ms), 1) + "ms]", kClrNormal, kSmall);
    else
        text(c1x, y, "AutoSpeed  --", kClrNormal, kSmall);
    y += lineH;

    if (v.cipo.cipo_raw_found)
        text(c1x, y, "CIPO raw   " + fd(v.cipo.cipo_raw_dist_m, 1) + " m"
             + (v.cipo.cut_in_detected ? "  [CUT-IN]" : ""), kClrNormal, kSmall);
    else
        text(c1x, y, "CIPO raw   (none)", kClrNormal, kSmall);

    // Col 2 — FUSED LONGITUDINAL
    y = py + 14;
    text(c2x, y, "[ FUSED LONGITUDINAL ]", kClrFusedLong, kSmall, kThin); y += lineH;
    if (v.cipo.valid) {
        text(c2x, y, "dist  " + fd(v.cipo.distance_m, 1) + " m", kClrFusedLong, -1, kBold);
        y += lineH + 2;
        text(c2x, y, "vel   " + fd(v.cipo.velocity_ms, 2) + " m/s", kClrFusedLong, -1, kBold);
        y += lineH + 2;
        text(c2x, y, "unc   ±" + fd(v.cipo.distance_stddev_m, 1) + " m", kClrNormal, kSmall);
    } else {
        text(c2x, y, "no estimate", kClrNormal, kSmall);
    }

    // Col 3 — FUSED LATERAL
    y = py + 14;
    text(c3x, y, "[ FUSED LATERAL ]", kClrFusedLat, kSmall, kThin); y += lineH;
    if (v.lateral.valid) {
        const float fused_steer = curvature_to_steer_deg(
            v.lateral.curvature, veh.wheelbase_m, veh.steer_ratio);
        text(c3x, y, "CTE   " + fd(v.lateral.cte_m, 2) + " m", kClrFusedLat, -1, kBold);
        y += lineH + 2;
        text(c3x, y, "yaw   " + fd(v.lateral.yaw_rad, 3) + " rad", kClrFusedLat, -1, kBold);
        y += lineH + 2;
        text(c3x, y, "k     " + fd(v.lateral.curvature, 4)
             + "  steer=" + fd(fused_steer, 1) + " deg", kClrFusedLat, -1, kBold);
    } else {
        text(c3x, y, "no estimate", kClrNormal, kSmall);
    }
}

// ─── Public ───────────────────────────────────────────────────────────────────

void annotate_frame(cv::Mat& frame, const DebugView& view)
{
    draw_autospeed_detections(frame, view.auto_speed);
    draw_autosteer_ego_path(frame, view.auto_steer);
    draw_steering_wheels(frame, view);
    draw_top_bar(frame, view);
    draw_hud_panel(frame, view);
}

}  // namespace visionpilot::debug
