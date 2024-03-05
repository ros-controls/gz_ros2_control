cmake_minimum_required(VERSION 3.5)
project(gz_ros2_control_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(control_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)

  add_executable(test_position src/test_position.cpp)
  ament_target_dependencies(test_position
    rclcpp
    rclcpp_action
    control_msgs
  )

  ## Install
  install(
    TARGETS
      test_position
    DESTINATION
      lib/${PROJECT_NAME}
  )

  install(DIRECTORY
    urdf
    config
    DESTINATION share/${PROJECT_NAME}/
  )

  add_subdirectory(tests)
endif()

ament_package()
