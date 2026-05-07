cmake_minimum_required(VERSION 3.22.1)

project(video_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Define the video_publisher library
add_library(${PROJECT_NAME}
    src/video_file_publisher.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# Link against dependencies
target_link_libraries(${PROJECT_NAME} PUBLIC
    rclcpp::rclcpp
    ${sensor_msgs_LIBRARIES}
    cv_bridge::cv_bridge
    ${OpenCV_LIBRARIES}
)

# Install targets
install(
    TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Print build info
message(STATUS "Building video_publisher")
message(STATUS "  OpenCV libraries: ${OpenCV_LIBRARIES}")