cmake_minimum_required(VERSION 3.5.0)
project(gz_ros2_control_demos)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)

set(GZ_SIM)

if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo harmonic")
  set(GZ_SIM gz-sim${GZ_SIM_VER}::core)
  add_definitions(-DGZ_HEADERS)
else()
  find_package(ignition-gazebo6 REQUIRED)
  set(GZ_SIM_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo fortress")
  set(GZ_SIM ignition-gazebo${GZ_SIM_VER}::core)
endif()

install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position
  rclcpp
  rclcpp_action
  control_msgs
)

# use the same example_position.cpp for example_velocity
add_executable(example_velocity examples/example_position.cpp)
ament_target_dependencies(example_velocity
  rclcpp
  rclcpp_action
  control_msgs
)

# use the same example_position.cpp for example_effort
add_executable(example_effort examples/example_position.cpp)
ament_target_dependencies(example_effort
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(example_diff_drive examples/example_diff_drive.cpp)
ament_target_dependencies(example_diff_drive
  rclcpp
  geometry_msgs
)

add_executable(example_tricycle_drive examples/example_tricycle_drive.cpp)
ament_target_dependencies(example_tricycle_drive
  rclcpp
  geometry_msgs
)

add_executable(example_ackermann_drive examples/example_ackermann_drive.cpp)
ament_target_dependencies(example_ackermann_drive
  rclcpp
  geometry_msgs
)

add_executable(example_gripper examples/example_gripper.cpp)
ament_target_dependencies(example_gripper
  rclcpp
  std_msgs
)

#########

add_library(gz_custom_hardware_plugins SHARED
  src/gz_custom_system.cpp
)
target_include_directories(gz_custom_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/gz_custom_hardware_plugins>
)
target_link_libraries(gz_custom_hardware_plugins
  ${GZ_SIM}
)
ament_target_dependencies(gz_custom_hardware_plugins
  control_toolbox
  hardware_interface
  gz_ros2_control
  rclcpp
  rclcpp_lifecycle
)

#########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install
install(
  TARGETS
    example_position
    example_velocity
    example_effort
    example_diff_drive
    example_tricycle_drive
    example_ackermann_drive
    example_gripper
  DESTINATION
    lib/${PROJECT_NAME}
)

install(TARGETS
  gz_custom_hardware_plugins
  EXPORT export_gz_custom_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_gz_custom_hardware_plugins HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(gz_ros2_control gz_custom_hardware_plugins.xml)

ament_package()
