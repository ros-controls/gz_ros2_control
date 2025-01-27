find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(position_test.py
  TIMEOUT 50
)

add_launch_test(velocity_test.py
  TIMEOUT 50
)

add_launch_test(effort_test.py
  TIMEOUT 50
)
