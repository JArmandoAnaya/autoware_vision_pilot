cmake_minimum_required(VERSION 3.22.1)

add_library(control
        src/longitudinal_control.cpp
        src/lateral_control.cpp
)

target_include_directories(control
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(control
        PUBLIC
        planning
        logging
)

target_compile_features(control PUBLIC cxx_std_17)
