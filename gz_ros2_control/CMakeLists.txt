cmake_minimum_required(VERSION 3.10)
project(gz_ros2_control)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(include)

#########

add_library(${PROJECT_NAME}-system SHARED
  src/gz_ros2_control_plugin.cpp
)
add_library(${PROJECT_NAME}-system::${PROJECT_NAME}-system ALIAS ${PROJECT_NAME}-system)

target_link_libraries(${PROJECT_NAME}-system
  gz-sim::gz-sim
  gz-plugin::register
)
ament_target_dependencies(${PROJECT_NAME}-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)
target_include_directories(${PROJECT_NAME}-system PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

#########

add_library(gz_hardware_plugins SHARED
  src/gz_system.cpp
)
ament_target_dependencies(gz_hardware_plugins
  rclcpp_lifecycle
  hardware_interface
  rclcpp
)
target_link_libraries(gz_hardware_plugins
  gz-sim::gz-sim
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install library and export targets
install(TARGETS
  ${PROJECT_NAME}-system
  gz_hardware_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT export_${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION share/${PROJECT_NAME}/cmake
)

#########

add_library(ign_ros2_control-system SHARED
  src/gz_ros2_control_plugin.cpp
)

target_link_libraries(ign_ros2_control-system
  gz-sim::gz-sim
  gz-plugin::register
)
ament_target_dependencies(ign_ros2_control-system
  ament_index_cpp
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor
  rclcpp_lifecycle
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export old-style CMake variables
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}-system gz_hardware_plugins)
ament_export_libraries(ign_ros2_control-system gz_hardware_plugins)

# Export modern CMake targets
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
    controller_manager
    hardware_interface
)
install(TARGETS ign_ros2_control-system
  DESTINATION lib
)

pluginlib_export_plugin_description_file(gz_ros2_control gz_hardware_plugins.xml)

# Setup the project
ament_package()
