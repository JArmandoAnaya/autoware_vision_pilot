#ifndef VISIONPILOT_ROS2_TO_CAN_HPP
#define VISIONPILOT_ROS2_TO_CAN_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>

namespace vehicle_state_interface {

/**
 * @class VehicleStateSubscriber
 * @brief Middleware-agnostic ROS2 node that subscribes to vehicle ego-state
 *        (`nav_msgs/Odometry`) and exposes the current ego speed for longitudinal
 *        control feedback.
 *
 * Generic ego-state source: it knows nothing about any particular vehicle or
 * simulator. The odometry topic is configurable, so the same node consumes a
 * real vehicle's odometry, CARLA's, or any other `nav_msgs/Odometry` publisher —
 * the source-specific topic name comes from config.
 *
 * Mirrors the camera_subscriber pattern: initializes ROS2 once, owns an internal
 * node spun on a background thread, and caches the latest message thread-safely.
 */
class VehicleStateSubscriber {
public:
    /**
     * @brief Construct and start the subscriber.
     *
     * @param topic_name Odometry topic (from config; e.g. "/vehicle/odometry").
     * @param node_name  Internal ROS2 node name.
     */
    explicit VehicleStateSubscriber(
        const std::string& topic_name,
        const std::string& node_name = "visionpilot_vehicle_state_subscriber");

    ~VehicleStateSubscriber();

    /**
     * @brief Latest ego speed [m/s] (planar magnitude of the body twist).
     *        Returns 0 until the first message arrives.
     */
    double get_ego_speed_mps() const;

    /// True once at least one odometry message has been received.
    bool has_state() const;

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    std::string topic_name_;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::thread spin_thread_;

    std::atomic<double> ego_speed_mps_{0.0};
    std::atomic<bool>   has_state_{false};
};

}  // namespace vehicle_state_interface

#endif  // VISIONPILOT_ROS2_TO_CAN_HPP
