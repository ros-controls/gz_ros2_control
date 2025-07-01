cmake_minimum_required(VERSION 3.5.0)
project(gz_ros2_control_demos)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  launch
  config
  sdf
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position PUBLIC
  ${control_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# use the same example_position.cpp for example_velocity
add_executable(example_velocity examples/example_position.cpp)
target_link_libraries(example_velocity PUBLIC
  ${control_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

# use the same example_position.cpp for example_effort
add_executable(example_effort examples/example_position.cpp)
target_link_libraries(example_effort PUBLIC
  ${control_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
)

add_executable(example_mobile_robots examples/example_mobile_robots.cpp)
target_link_libraries(example_mobile_robots
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
)

add_executable(example_gripper examples/example_gripper.cpp)
target_link_libraries(example_gripper PUBLIC
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
)

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
    example_mobile_robots
    example_gripper
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
