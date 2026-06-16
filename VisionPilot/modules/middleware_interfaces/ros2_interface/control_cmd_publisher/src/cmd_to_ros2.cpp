#include <control_cmd_publisher/cmd_to_ros2.hpp>

namespace control_interface {

ControlCommandPublisher::ControlCommandPublisher(
    const std::string& topic_name,
    const std::string& frame_id,
    const std::string& node_name)
    : topic_name_(topic_name), frame_id_(frame_id)
{
    // Init ROS2 once (the subscriber may have already done it).
    if (!rclcpp::ok()) {
        static int argc = 1;
        static const char* argv[] = {"visionpilot_control_publisher", nullptr};
        rclcpp::init(argc, const_cast<char**>(argv));
    }

    node_ = std::make_shared<rclcpp::Node>(node_name);

    RCLCPP_INFO(node_->get_logger(), "Initializing Control Command Publisher");
    RCLCPP_INFO(node_->get_logger(), "  Topic: %s", topic_name_.c_str());

    // Reliable, keep-last(1): a control sink wants the freshest command, delivered.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    publisher_ =
        node_->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(topic_name_, qos);

    spin_thread_ = std::thread([this]() { rclcpp::spin(this->node_); });
}

void ControlCommandPublisher::publish(double steering_rad, double speed_mps, double accel_mps2)
{
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp    = node_->now();
    msg.header.frame_id = frame_id_;

    msg.drive.steering_angle = static_cast<float>(steering_rad);
    msg.drive.speed          = static_cast<float>(speed_mps);
    msg.drive.acceleration   = static_cast<float>(accel_mps2);

    publisher_->publish(msg);
}

ControlCommandPublisher::~ControlCommandPublisher()
{
    rclcpp::shutdown();
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
}

}  // namespace control_interface
