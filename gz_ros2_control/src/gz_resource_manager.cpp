// Copyright 2026 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gz_resource_manager.hpp"

#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <rclcpp/logging.hpp>

namespace gz_ros2_control
{
GZResourceManager::GZResourceManager(
  rclcpp::Node::SharedPtr & node,
  gz::sim::EntityComponentManager & ecm,
  std::map<std::string, gz::sim::Entity> enabledJoints,
  SystemLoader system_loader)
: hardware_interface::ResourceManager(
    node->get_node_clock_interface(), node->get_node_logging_interface()),
  node_(node),
  ecm_(&ecm),
  enabledJoints_(std::move(enabledJoints)),
  system_loader_(std::move(system_loader)),
  logger_(node->get_logger().get_child("GZResourceManager"))
{
  if (!system_loader_) {
    gz_system_loader_ = std::make_unique<pluginlib::ClassLoader<GazeboSimSystemInterface>>(
      "gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface");
    system_loader_ = [this](const std::string & plugin_name) {
        return std::unique_ptr<GazeboSimSystemInterface>(
          gz_system_loader_->createUnmanagedInstance(plugin_name));
      };
  }
}

bool GZResourceManager::load_and_initialize_components(
  const hardware_interface::ResourceManagerParams & params)
{
  components_are_loaded_and_initialized_ = true;

  std::vector<hardware_interface::HardwareInfo> hardware_info;
  try {
    hardware_info =
      hardware_interface::parse_control_resources_from_urdf(params.robot_description);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger_, "Error while parsing the robot description: %s", ex.what());
    components_are_loaded_and_initialized_ = false;
    return false;
  }

  for (const auto & individual_hardware_info : hardware_info) {
    std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
    RCLCPP_DEBUG(
      logger_, "Load hardware interface %s ...",
      robot_hw_sim_type_str_.c_str());

    // Load hardware
    std::unique_ptr<GazeboSimSystemInterface> gzSimSystem;
    std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
    try {
      gzSimSystem = system_loader_(robot_hw_sim_type_str_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_,
        "The plugin failed to load for some reason. Error: %s\n",
        ex.what());
    }
    if (!gzSimSystem) {
      RCLCPP_FATAL(
        logger_, "Could not load robot simulation interface %s",
        robot_hw_sim_type_str_.c_str());
      components_are_loaded_and_initialized_ = false;
      break;
    }

    // initialize simulation requirements
    bool sim_initialized = false;
    try {
      sim_initialized = gzSimSystem->initSim(
        node_,
        enabledJoints_,
        individual_hardware_info,
        *ecm_,
        params.update_rate);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        logger_, "Exception while initializing robot simulation interface %s: %s",
        robot_hw_sim_type_str_.c_str(), ex.what());
    }
    if (!sim_initialized) {
      RCLCPP_FATAL(
        logger_, "Could not initialize robot simulation interface");
      components_are_loaded_and_initialized_ = false;
      break;
    }
    RCLCPP_DEBUG(
      logger_, "Initialized robot simulation interface %s!",
      robot_hw_sim_type_str_.c_str());

    hardware_interface::HardwareComponentParams component_params;
    component_params.hardware_info = individual_hardware_info;
    component_params.executor = params.executor;
    component_params.clock = params.clock;
    component_params.logger = params.logger;
    // initialize hardware
    import_component(std::move(gzSimSystem), component_params);
  }

  return components_are_loaded_and_initialized_;
}
}  // namespace gz_ros2_control
