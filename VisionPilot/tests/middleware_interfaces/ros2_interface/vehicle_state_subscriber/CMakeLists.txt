find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(test_vehicle_state_subscriber
        test_vehicle_state_subscriber.cpp
)

target_link_libraries(test_vehicle_state_subscriber
        PRIVATE
        vehicle_state_subscriber
        rclcpp::rclcpp
        ${nav_msgs_TARGETS}
)
