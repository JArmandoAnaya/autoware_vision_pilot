#pragma once

#include <onnxruntime_cxx_api.h>

#include <memory>
#include <string>

namespace visionpilot::models {

// Runtime configuration passed to every engine on construction.
struct OnnxSessionConfig {
    std::string model_path;

    // Execution provider: "cpu" | "cuda" | "tensorrt"
    std::string provider     = "cpu";

    // Precision used only when provider == "tensorrt": "fp32" | "fp16"
    std::string precision    = "fp32";

    int         device_id    = 0;

    // TensorRT-only settings
    std::string cache_dir    = "/tmp/visionpilot_trt_cache";
    std::string cache_prefix = "model_";
    double      workspace_gb = 1.0;
};

// Shared factory that creates an Ort::Session for any of the three execution
// providers.  Extending to TensorRT later requires no changes in the engines.
class OnnxSessionFactory {
public:
    static std::unique_ptr<Ort::Session> create(
        Ort::Env&                env,
        const OnnxSessionConfig& cfg);

private:
    static std::unique_ptr<Ort::Session> createCpu(
        Ort::Env& env, const std::string& model_path);

    static std::unique_ptr<Ort::Session> createCuda(
        Ort::Env& env, const std::string& model_path, int device_id);

    static std::unique_ptr<Ort::Session> createTensorRT(
        Ort::Env& env, const OnnxSessionConfig& cfg);
};

}  // namespace visionpilot::models
