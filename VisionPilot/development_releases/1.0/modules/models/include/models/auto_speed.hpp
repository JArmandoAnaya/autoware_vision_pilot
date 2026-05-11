#pragma once

#include "models/model_types.hpp"
#include "models/onnx_session.hpp"

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>
#include <vector>

namespace visionpilot::models {

// ─── Interface ────────────────────────────────────────────────────────────────
// Abstract base for the AutoSpeed YOLO-style object detection model.
//
// Preprocessing contract (caller's responsibility before calling infer()):
//   • Letterbox-resize frame to NET_W × NET_H (1024 × 512), preserving aspect
//     ratio, padding with (114, 114, 114)
//   • Convert BGR → RGB
//   • Scale pixel values to [0, 1]  (NO ImageNet normalisation)
//   • Layout: CHW float32, size = 3 × NET_H × NET_W = CHW_SIZE elements
//
// Raw model output shape: [1, C, N]
//   C = 4 + num_classes  (cx, cy, w, h, class_logit_0, …, class_logit_{K-1})
//   N = total number of anchor boxes
// Post-processing (sigmoid + threshold + NMS) is done internally.
class AutoSpeedBase {
public:
    virtual ~AutoSpeedBase() = default;

    // image_chw  : float32 CHW buffer, CHW_SIZE elements, RGB, [0, 1].
    // conf_thres : minimum sigmoid class probability to keep a box.
    // iou_thres  : IoU threshold for non-maximum suppression.
    virtual AutoSpeedOutput infer(const float* image_chw,
                                  float conf_thres = 0.6f,
                                  float iou_thres  = 0.45f) = 0;
};

// ─── ONNX Runtime implementation ─────────────────────────────────────────────
class AutoSpeedOnnx final : public AutoSpeedBase {
public:
    static constexpr int NET_H    = 512;
    static constexpr int NET_W    = 1024;
    static constexpr int CHW_SIZE = 3 * NET_H * NET_W;  // 1 572 864

    explicit AutoSpeedOnnx(const OnnxSessionConfig& cfg);
    ~AutoSpeedOnnx() override = default;

    AutoSpeedOutput infer(const float* image_chw,
                          float conf_thres = 0.6f,
                          float iou_thres  = 0.45f) override;

private:
    // Decode, threshold, convert xywh→xyxy, apply NMS.
    AutoSpeedOutput postProcess(const Ort::Value& tensor,
                                float conf_thres,
                                float iou_thres) const;

    // IoU between two detections.
    static float iou(const Detection& a, const Detection& b);

    // Sort-by-score + greedy NMS.
    static std::vector<Detection> nms(std::vector<Detection> dets,
                                      float iou_thres);

    Ort::Env                       env_;
    std::unique_ptr<Ort::Session>  session_;
    Ort::MemoryInfo                mem_info_;

    std::vector<std::string>  in_name_strs_;
    std::vector<const char*>  in_names_;

    std::vector<std::string>  out_name_strs_;
    std::vector<const char*>  out_names_;

    std::vector<int64_t>  input_shape_;  // {1, 3, NET_H, NET_W}
};

}  // namespace visionpilot::models
