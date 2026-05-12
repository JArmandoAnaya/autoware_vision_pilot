cmake_minimum_required(VERSION 3.22.1)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_viewer_node
        src/camera_viewer_node.cpp
)

# Ensure C++17 standard is used
target_compile_features(camera_viewer_node PRIVATE cxx_std_17)

target_link_libraries(camera_viewer_node
    PRIVATE
    camera_subscriber
    rclcpp::rclcpp
    cv_bridge::cv_bridge
    ${OpenCV_LIBRARIES}
)