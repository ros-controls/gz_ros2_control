cmake_minimum_required(VERSION 3.5.0)
project(gz_ros2_control_demos)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)

install(DIRECTORY
  launch
  config
  sdf
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
PUBLIC
  gz-sim::gz-sim
  control_toolbox::control_toolbox
  gz_ros2_control::gz_ros2_control-system
  hardware_interface::hardware_interface
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
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
