#ifndef VISIONPILOT_CMD_TO_ROS2_HPP
#define VISIONPILOT_CMD_TO_ROS2_HPP

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <thread>

namespace control_interface {

/**
 * @class ControlCommandPublisher
 * @brief Middleware-agnostic ROS2 node that publishes VisionPilot control
 *        commands as `ackermann_msgs/AckermannDriveStamped`.
 *
 * This is a generic ROS2 actuation sink: it knows nothing about any particular
 * vehicle or simulator. The destination topic is fully configurable, so the same
 * node drives a real Ackermann vehicle, CARLA, or any other consumer that speaks
 * AckermannDriveStamped — the consumer-specific topic name comes from config.
 *
 * Mirrors the camera_subscriber pattern (camera_subscriber/ros2_to_opencv.hpp):
 * - Initializes ROS2 (rclcpp::init) once if not already up
 * - Owns an internal node, spun on a background thread
 * - Publishes the latest steering / speed / acceleration command
 *
 * Message contract: `steering_angle` is the real front-wheel angle (rad, +ve =
 * left); `speed` is the longitudinal target (m/s); `acceleration` is advisory.
 */
class ControlCommandPublisher {
public:
    /**
     * @brief Construct and start the publisher.
     *
     * @param topic_name Topic to publish on (from config; e.g. "/control/ackermann_cmd").
     * @param frame_id   header.frame_id for the published command.
     * @param node_name  Internal ROS2 node name.
     */
    explicit ControlCommandPublisher(
        const std::string& topic_name,
        const std::string& frame_id  = "base_link",
        const std::string& node_name = "visionpilot_control_publisher");

    ~ControlCommandPublisher();

    /**
     * @brief Publish one Ackermann control command.
     *
     * @param steering_rad Front-wheel angle [rad], +ve = left.
     * @param speed_mps    Target forward speed [m/s].
     * @param accel_mps2   Desired acceleration [m/s^2] (0 = as-fast-as-possible).
     */
    void publish(double steering_rad, double speed_mps, double accel_mps2 = 0.0);

private:
    std::string topic_name_;
    std::string frame_id_;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    std::thread spin_thread_;
};

}  // namespace control_interface

#endif  // VISIONPILOT_CMD_TO_ROS2_HPP
