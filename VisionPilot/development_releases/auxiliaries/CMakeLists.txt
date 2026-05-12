cmake_minimum_required(VERSION 3.22.1)

project(Aux_VisionPilot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Put all executables in build/
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Modules
add_subdirectory(modules/middleware_interfaces/ros2_interface/camera_subscriber)

# Camera subscriber
add_subdirectory(../1.0/modules/middleware_interfaces/ros2_interface/camera_subscriber
                 ${CMAKE_BINARY_DIR}/camera_subscriber)