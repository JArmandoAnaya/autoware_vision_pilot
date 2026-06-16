#include <vehicle_state_subscriber/ros2_to_can.hpp>

#include <cmath>

namespace vehicle_state_interface {

VehicleStateSubscriber::VehicleStateSubscriber(
    const std::string& topic_name,
    const std::string& node_name)
    : topic_name_(topic_name)
{
    // Init ROS2 once (the camera subscriber / control publisher may have done it).
    if (!rclcpp::ok()) {
        static int argc = 1;
        static const char* argv[] = {"visionpilot_vehicle_state_subscriber", nullptr};
        rclcpp::init(argc, const_cast<char**>(argv));
    }

    node_ = std::make_shared<rclcpp::Node>(node_name);

    RCLCPP_INFO(node_->get_logger(), "Initializing Vehicle State Subscriber");
    RCLCPP_INFO(node_->get_logger(), "  Topic: %s", topic_name_.c_str());

    // Best-effort, keep-last(1): we only ever want the freshest ego state.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
    subscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        topic_name_, qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_callback(msg); });

    spin_thread_ = std::thread([this]() { rclcpp::spin(this->node_); });
}

void VehicleStateSubscriber::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg) return;
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    ego_speed_mps_.store(std::hypot(vx, vy));
    has_state_.store(true);
}

double VehicleStateSubscriber::get_ego_speed_mps() const
{
    return ego_speed_mps_.load();
}

bool VehicleStateSubscriber::has_state() const
{
    return has_state_.load();
}

VehicleStateSubscriber::~VehicleStateSubscriber()
{
    rclcpp::shutdown();
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

}  // namespace vehicle_state_interface
