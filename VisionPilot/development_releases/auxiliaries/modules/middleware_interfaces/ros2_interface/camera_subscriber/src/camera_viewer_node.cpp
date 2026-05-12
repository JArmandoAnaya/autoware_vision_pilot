#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <tuple>

#include "camera_subscriber/ros2_to_opencv.hpp"

using namespace std::chrono;

namespace {

std::string format_fps(double fps)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << fps;
    return stream.str();
}

std::string format_text(const std::string &label, const std::string &value)
{
    return label + value;
}

}  // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::string topic = "/camera/image";
    if (argc > 1) {
        topic = argv[1];
    }

    auto node = std::make_shared<camera_subscriber::ROS2ImageSubscriber>(
        topic,
        "camera_viewer_node"
    );

    cv::namedWindow("camera_viewer", cv::WINDOW_NORMAL);

    bool window_initialized = false;

    uint64_t frame_count = 0;
    auto last_frame_time = steady_clock::now();
    double received_fps = 0.0;
    std::deque<double> frame_intervals;
    constexpr std::size_t kFpsWindowSize = 30;

    RCLCPP_INFO(node->get_logger(), "camera_viewer_node started");
    RCLCPP_INFO(node->get_logger(), "  topic: %s", topic.c_str());

    rclcpp::Rate loop_rate(30);

    while (rclcpp::ok()) {
        rclcpp::spin_some(node);

        bool has_frames_before = node->has_frames();
        auto [ok, frame] = node->get_latest_frame();
        if (ok && !frame.empty()) {
            frame_count++;

            if (!window_initialized) {
                cv::resizeWindow("camera_viewer", frame.cols, frame.rows);
                window_initialized = true;
                RCLCPP_INFO(
                    node->get_logger(),
                    "first frame received: %dx%d, channels=%d, type=%d",
                    frame.cols,
                    frame.rows,
                    frame.channels(),
                    frame.type()
                );
            }

            auto now = steady_clock::now();
            auto elapsed_sec = duration<double>(now - last_frame_time).count();
            if (elapsed_sec > 0.0) {
                frame_intervals.push_back(elapsed_sec);
                if (frame_intervals.size() > kFpsWindowSize) {
                    frame_intervals.pop_front();
                }

                double total_interval = 0.0;
                for (double interval : frame_intervals) {
                    total_interval += interval;
                }

                if (total_interval > 0.0) {
                    received_fps = static_cast<double>(frame_intervals.size()) / total_interval;
                }
            }
            last_frame_time = now;

            auto stats = node->get_stats();
            bool stream_active = node->is_stream_active();
            bool has_frames = has_frames_before;

            // STATS OVERLAY
            
            std::vector<std::string> overlay_lines = {
                "frame: " + std::to_string(frame_count),
                "fps: " + format_fps(received_fps),
                "topic: " + topic
            };

            cv::Mat display = frame.clone();
            const int font_face = cv::FONT_HERSHEY_SIMPLEX;
            const double font_scale = 0.55;
            const int thickness = 1;
            const int line_gap = 8;
            const int left_padding = 12;
            const int top_padding = 24;

            int box_width = 0;
            int box_height = 0;
            for (const auto &line : overlay_lines) {
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(line, font_face, font_scale, thickness, &baseline);
                box_width = std::max(box_width, text_size.width);
                box_height += text_size.height + line_gap;
            }

            cv::rectangle(
                display,
                cv::Rect(6, 6, box_width + left_padding * 2, box_height + top_padding),
                cv::Scalar(0, 0, 0),
                cv::FILLED
            );

            int y = 6 + top_padding - 6;
            for (const auto &line : overlay_lines) {
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(line, font_face, font_scale, thickness, &baseline);
                y += text_size.height;
                cv::putText(
                    display,
                    line,
                    cv::Point(12, y),
                    font_face,
                    font_scale,
                    cv::Scalar(0, 255, 255),
                    thickness,
                    cv::LINE_AA
                );
                y += line_gap;
            }

            cv::imshow("camera_viewer", display);
            cv::waitKey(1);

            RCLCPP_INFO(
                node->get_logger(),
                "cv::Mat: %dx%d channels=%d type=%d | received_fps=%.2f | stream_active=%s | has_frames=%s | received=%lu dropped=%lu errors=%lu",
                frame.cols,
                frame.rows,
                frame.channels(),
                frame.type(),
                received_fps,
                stream_active ? "true" : "false",
                has_frames ? "true" : "false",
                static_cast<unsigned long>(stats.frames_received),
                static_cast<unsigned long>(stats.frames_dropped),
                static_cast<unsigned long>(stats.conversion_errors)
            );
        }

        loop_rate.sleep();
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
