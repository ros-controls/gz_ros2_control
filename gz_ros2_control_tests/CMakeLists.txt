cmake_minimum_required(VERSION 3.10)
project(gz_ros2_control_tests)

find_package(ament_cmake REQUIRED)

# Install Python package (CRITICAL)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
