// Runtime smoke for VehicleStateSubscriber: publish a known Odometry and confirm the
// subscriber reports the matching planar ego speed. No model weights, no camera needed.
#include <rclcpp/rclcpp.hpp>
#include <vehicle_state_subscriber/ros2_to_can.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

namespace
{

int g_failures = 0;

void check(const char * name, bool ok)
{
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", name);
  if (!ok) ++g_failures;
}

bool approx(double a, double b, double tol)
{
  return std::fabs(a - b) <= tol;
}

}  // namespace

int main()
{
  const std::string topic = "/vehicle/odometry";

  // Subscriber under test (owns rclcpp::init).
  vehicle_state_subscriber::VehicleStateSubscriber subscriber(topic);

  // No message yet: no state, zero speed.
  check("no state before first message", !subscriber.has_state());
  check("zero speed before first message", approx(subscriber.ego_speed_mps(), 0.0, 1e-9));

  // Publisher on the same context + matching QoS (reliable, latest-only).
  auto pub_node = std::make_shared<rclcpp::Node>("test_vehicle_state_publisher");
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto pub = pub_node->create_publisher<nav_msgs::msg::Odometry>(topic, qos);

  nav_msgs::msg::Odometry odom;
  odom.twist.twist.linear.x = 3.0;
  odom.twist.twist.linear.y = 4.0;  // hypot(3, 4) == 5

  // Publish + spin until received or timeout (discovery + delivery take a moment).
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!subscriber.has_state() && std::chrono::steady_clock::now() < deadline) {
    pub->publish(odom);
    rclcpp::spin_some(pub_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  check("state received", subscriber.has_state());
  if (subscriber.has_state()) {
    check("ego speed = hypot(vx, vy)", approx(subscriber.ego_speed_mps(), 5.0, 1e-4));
  }

  std::printf(
    "\n%s (%d failure%s)\n", g_failures ? "FAILED" : "ALL PASS", g_failures,
    g_failures == 1 ? "" : "s");
  return g_failures == 0 ? 0 : 1;
}
