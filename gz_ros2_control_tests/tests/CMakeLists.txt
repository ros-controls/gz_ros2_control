find_package(launch_testing_ament_cmake REQUIRED)

set(TEST_PYTHONPATH "${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")

add_launch_test(position_test.py
  TIMEOUT 50
)
add_launch_test(velocity_test.py
  TIMEOUT 50
)
add_launch_test(velocity_custom_plugin_test.py
  TIMEOUT 50
)
add_launch_test(effort_test.py
  TIMEOUT 50
)

add_launch_test(ft_sensor_test.py
  TIMEOUT 50
)

add_launch_test(pendulum_effort_test.py
  TIMEOUT 50
)

add_launch_test(pendulum_position_test.py
  TIMEOUT 50
  ENV PYTHONPATH=${TEST_PYTHONPATH}
)

add_launch_test(gripper_mimic_joint_position_test.py
  TIMEOUT 50
)

add_launch_test(gripper_mimic_joint_effort_test.py
  TIMEOUT 50
)
