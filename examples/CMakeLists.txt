foreach(example 1d_robot_tracking altitude_estimation 2d_differential_kinematics)
    add_executable(example_${example} ${example}.cpp)
    target_link_libraries(example_${example} PRIVATE kfl::kalman_filter_lib kfl_warnings)
endforeach()
