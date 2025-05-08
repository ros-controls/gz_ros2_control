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
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_position examples/example_position.cpp)
target_link_libraries(example_position PUBLIC
  rclcpp
  rclcpp_action
  control_msgs
)

# use the same example_position.cpp for example_velocity
add_executable(example_velocity examples/example_position.cpp)
target_link_libraries(example_velocity PUBLIC
  rclcpp
  rclcpp_action
  control_msgs
)

# use the same example_position.cpp for example_effort
add_executable(example_effort examples/example_position.cpp)
target_link_libraries(example_effort PUBLIC
  rclcpp
  rclcpp_action
  control_msgs
)

add_executable(example_mobile_robots examples/example_mobile_robots.cpp)
target_link_libraries(example_mobile_robots PUBLIC
  rclcpp
  geometry_msgs
)

add_executable(example_gripper examples/example_gripper.cpp)
target_link_libraries(example_gripper PUBLIC
  rclcpp
  std_msgs
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
