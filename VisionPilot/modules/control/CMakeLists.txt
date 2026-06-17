cmake_minimum_required(VERSION 3.22.1)

add_library(control
        src/longitudinal_control.cpp
        src/lateral_control.cpp
        src/controller.cpp
)

target_include_directories(control
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Perception -> planner -> control integration glue. Owns the Planner + Controller so the
# app calls the whole drive step in one line; kept here (not in app/) per the module layout.
add_library(control_bridge
        src/control_bridge.cpp
)
target_include_directories(control_bridge
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(control_bridge PUBLIC control planning logging)