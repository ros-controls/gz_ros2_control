cmake_minimum_required(VERSION 3.5)
project(gz_ros2_control_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ## Install
  install(DIRECTORY
    urdf
    config
    DESTINATION share/${PROJECT_NAME}/
  )

  add_subdirectory(tests)
endif()

ament_package()
