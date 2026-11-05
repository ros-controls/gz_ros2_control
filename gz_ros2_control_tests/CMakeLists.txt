cmake_minimum_required(VERSION 3.10)
project(gz_ros2_control_tests)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Helpers shared by the launch tests
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
