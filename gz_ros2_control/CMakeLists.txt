cmake_minimum_required(VERSION 3.5)
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
find_package(yaml_cpp_vendor REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)

find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}-system SHARED
  src/gz_ros2_control_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}-system
  PUBLIC
  gz-sim::gz-sim
  gz-plugin::register
  ament_index_cpp::ament_index_cpp
  controller_manager::controller_manager
  hardware_interface::mock_components
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

#########

add_library(gz_hardware_plugins SHARED
  src/gz_system.cpp
)
target_link_libraries(gz_hardware_plugins
PUBLIC
  gz-sim::gz-sim
  hardware_interface::mock_components
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

## Install
install(TARGETS
  gz_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  include/
  DESTINATION include
)

# Testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}-system gz_hardware_plugins)

# Install directories
install(TARGETS ${PROJECT_NAME}-system
  DESTINATION lib
)

pluginlib_export_plugin_description_file(gz_ros2_control gz_hardware_plugins.xml)

# Setup the project
ament_package()
