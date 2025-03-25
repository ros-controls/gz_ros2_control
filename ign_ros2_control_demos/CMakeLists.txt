cmake_minimum_required(VERSION 3.5.0)
project(ign_ros2_control_demos)

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
find_package(ament_index_cpp REQUIRED)

install(DIRECTORY
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}/
)


add_executable(example_position examples/example_position.cpp)
ament_target_dependencies(example_position
  ament_index_cpp
)

# use the same example_position.cpp for example_velocity
add_executable(example_velocity examples/example_position.cpp)
ament_target_dependencies(example_velocity
  ament_index_cpp
)

# use the same example_position.cpp for example_effort
add_executable(example_effort examples/example_position.cpp)
ament_target_dependencies(example_effort
  ament_index_cpp
)

add_executable(example_diff_drive examples/example_diff_drive.cpp)
ament_target_dependencies(example_diff_drive
  ament_index_cpp
)

add_executable(example_tricycle_drive examples/example_tricycle_drive.cpp)
ament_target_dependencies(example_tricycle_drive
  ament_index_cpp
)

add_executable(example_ackermann_drive examples/example_ackermann_drive.cpp)
ament_target_dependencies(example_ackermann_drive
  ament_index_cpp
)

add_executable(example_gripper examples/example_gripper.cpp)
ament_target_dependencies(example_gripper
  ament_index_cpp
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
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

ament_package()
