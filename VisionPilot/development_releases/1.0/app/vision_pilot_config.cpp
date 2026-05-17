#include "vision_pilot_config.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string trim(const std::string& s)
{
    const auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
}

std::string expand_user_path(std::string path)
{
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home == nullptr) {
            throw std::runtime_error("Cannot expand ~ in path: HOME is not set");
        }
        if (path.size() == 1 || path[1] == '/') {
            path.replace(0, 1, home);
        } else {
            throw std::runtime_error("Only ~ and ~/path are supported in config paths");
        }
    }
    return path;
}

std::map<std::string, std::string> parse_conf_file(const std::string& config_path)
{
    std::ifstream in(config_path);
    if (!in) {
        throw std::runtime_error("Cannot open config file: " + config_path);
    }

    std::map<std::string, std::string> kv;
    std::string line;
    int line_no = 0;

    while (std::getline(in, line)) {
        ++line_no;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        const auto eq = line.find('=');
        if (eq == std::string::npos) {
            throw std::runtime_error(
                config_path + ":" + std::to_string(line_no) +
                ": expected 'key = value', got: " + line);
        }

        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        if (key.empty()) {
            throw std::runtime_error(
                config_path + ":" + std::to_string(line_no) + ": empty key");
        }
        kv[key] = val;
    }
    return kv;
}

const std::string& require_key(
    const std::map<std::string, std::string>& kv, const std::string& key)
{
    const auto it = kv.find(key);
    if (it == kv.end() || it->second.empty()) {
        throw std::runtime_error("Missing required config key: " + key);
    }
    return it->second;
}

std::string optional_key(
    const std::map<std::string, std::string>& kv,
    const std::string& key,
    const std::string& default_val)
{
    const auto it = kv.find(key);
    if (it == kv.end() || it->second.empty()) return default_val;
    return it->second;
}

int parse_int(const std::string& s, const std::string& key)
{
    try {
        return std::stoi(s);
    } catch (...) {
        throw std::runtime_error("Invalid integer for " + key + ": " + s);
    }
}

double parse_double(const std::string& s, const std::string& key)
{
    try {
        return std::stod(s);
    } catch (...) {
        throw std::runtime_error("Invalid number for " + key + ": " + s);
    }
}

bool file_exists(const std::string& path)
{
    return !path.empty() && std::filesystem::is_regular_file(path);
}

bool parse_bool(const std::string& s, const std::string& key)
{
    if (s == "1" || s == "true" || s == "True" || s == "yes" || s == "on") return true;
    if (s == "0" || s == "false" || s == "False" || s == "no" || s == "off") return false;
    throw std::runtime_error("Invalid boolean for " + key + ": " + s);
}

}  // namespace

SourceMode parse_source_mode(const std::string& value)
{
    if (value == "0" || value == "ros2") return SourceMode::Ros2;
    if (value == "1" || value == "v4l2") return SourceMode::V4l2;
    if (value == "2" || value == "video") return SourceMode::Video;
    throw std::runtime_error(
        "Invalid source.mode: '" + value + "'. Use video | ros2 | v4l2 (or 0/1/2)");
}

VisionPilotConfig load_vision_pilot_config(const std::string& config_path)
{
    const auto kv = parse_conf_file(config_path);

    VisionPilotConfig cfg;
    cfg.autodrive_model = expand_user_path(require_key(kv, "models.autodrive_path"));
    cfg.autosteer_model = expand_user_path(require_key(kv, "models.autosteer_path"));
    cfg.autospeed_model = expand_user_path(require_key(kv, "models.autospeed_path"));

    cfg.engine_cfg.provider     = optional_key(kv, "engine.provider", "cpu");
    cfg.engine_cfg.precision    = optional_key(kv, "engine.precision", "fp32");
    cfg.engine_cfg.device_id    = parse_int(optional_key(kv, "engine.device_id", "0"), "engine.device_id");
    cfg.engine_cfg.cache_dir    = expand_user_path(optional_key(kv, "engine.cache_dir", "/tmp/visionpilot_trt_cache"));
    cfg.engine_cfg.workspace_gb = parse_double(optional_key(kv, "engine.workspace_gb", "1.0"), "engine.workspace_gb");

    cfg.source.mode = parse_source_mode(optional_key(kv, "source.mode", "video"));
    cfg.source.video_path = expand_user_path(optional_key(kv, "source.video_path", ""));
    cfg.source.video_realtime = parse_bool(
        optional_key(kv, "source.video_realtime", "true"), "source.video_realtime");
    cfg.source.video_loop = parse_bool(
        optional_key(kv, "source.video_loop", "false"), "source.video_loop");
    cfg.source.ros2_topic   = optional_key(kv, "source.ros2_topic", "/camera/image");
    cfg.source.v4l2_device  = optional_key(kv, "source.v4l2_device", "/dev/video0");
    cfg.source.v4l2_fps     = parse_int(optional_key(kv, "source.v4l2_fps", "10"), "source.v4l2_fps");

    cfg.pipeline.initial_inference_check = parse_bool(
        optional_key(kv, "pipeline.initial_inference_check", "true"),
        "pipeline.initial_inference_check");

    if (cfg.source.mode == SourceMode::Video) {
        if (cfg.source.video_path.empty()) {
            throw std::runtime_error(
                "source.mode=video requires source.video_path in config");
        }
        if (!file_exists(cfg.source.video_path)) {
            throw std::runtime_error(
                "source.video_path does not exist: " + cfg.source.video_path);
        }
    }

    for (const auto& [label, path] : std::vector<std::pair<const char*, const std::string&>>{
             {"models.autodrive_path", cfg.autodrive_model},
             {"models.autosteer_path", cfg.autosteer_model},
             {"models.autospeed_path", cfg.autospeed_model},
         }) {
        if (!file_exists(path)) {
            throw std::runtime_error(
                std::string(label) + " does not exist: " + path);
        }
    }

    return cfg;
}

std::string resolve_vision_pilot_config_path(int argc, char** argv)
{
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" || arg == "-c") {
            return argv[i + 1];
        }
        const std::string prefix = "--config=";
        if (arg.rfind(prefix, 0) == 0) {
            return arg.substr(prefix.size());
        }
    }

    if (const char* env = std::getenv("VISIONPILOT_CONFIG")) {
        if (file_exists(env)) return env;
    }

    constexpr const char* candidates[] = {
        "config/vision_pilot.conf",
        "vision_pilot.conf",
    };
    for (const char* path : candidates) {
        if (file_exists(path)) return path;
    }

    return {};
}
