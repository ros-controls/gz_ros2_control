cmake_minimum_required(VERSION 3.5)
project(gz_ros2_control_tests)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
