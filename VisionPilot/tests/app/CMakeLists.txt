add_executable(test_vision_pilot
        test_vision_pilot.cpp
)

target_link_libraries(test_vision_pilot
        PRIVATE
        control_bridge   # ControlBridge (+ control, planning)
)
