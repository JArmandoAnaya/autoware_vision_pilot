add_executable(test_control
        test_control.cpp
)

target_link_libraries(test_control
        PRIVATE
        control
)
