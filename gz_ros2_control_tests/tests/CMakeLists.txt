find_package(launch_testing_ament_cmake REQUIRED)

add_launch_test(position_test.py
  TARGET test_position_controller
  TIMEOUT 120
)
