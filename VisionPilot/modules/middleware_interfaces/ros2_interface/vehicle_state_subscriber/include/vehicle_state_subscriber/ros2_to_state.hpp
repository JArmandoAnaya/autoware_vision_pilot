#ifndef VISIONPILOT_ROS2_TO_STATE_HPP
#define VISIONPILOT_ROS2_TO_STATE_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace vehicle_state_subscriber {

    /**
    * @class VehicleStateSubscriber
    * @brief Subscribes to vehicle odometry (nav_msgs/Odometry) and exposes the live
    *        ego speed to the control loop.
    *
    * Ego speed = hypot(twist.twist.linear.x, twist.twist.linear.y) (planar speed in m/s).
    * Until the first message arrives, ego_speed_mps() returns 0.0 and has_state() is false,
    * so callers can fall back to a configured constant.
    *
    * Coexistence: this node uses an init-ownership guard (owns rclcpp init only if it called
    * it) and a flag-controlled spin_some loop, so it never shuts down a context owned by a
    * co-resident node (e.g. the camera subscriber).
    */
    class VehicleStateSubscriber {

        public:

            /**
            * @brief Constructor.
            *
            * @param topic     The odometry topic to subscribe to.
            * @param node_name The internal ROS2 node name.
            */
            explicit VehicleStateSubscriber(
                const std::string& topic,
                const std::string& node_name = "vision_pilot_vehicle_state"
            );

            /**
            * @brief Destructor. Stops the spin loop, joins the thread, and shuts down ROS2
            *        only if this object initialized it.
            */
            ~VehicleStateSubscriber();

            /**
            * @brief Whether at least one odometry message has been received.
            */
            bool has_state() const;

            /**
            * @brief Latest planar ego speed in m/s (0.0 until the first message).
            */
            double ego_speed_mps() const;

        private:

            void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

            bool owns_init_ = false;
            std::shared_ptr<rclcpp::Node> node_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;

            std::thread spin_thread_;
            std::atomic<bool> running_{false};

            mutable std::mutex mutex_;
            double ego_speed_mps_ = 0.0;
            bool has_state_ = false;
    };

}  // namespace vehicle_state_subscriber

#endif //VISIONPILOT_ROS2_TO_STATE_HPP
