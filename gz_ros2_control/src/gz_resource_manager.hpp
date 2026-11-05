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

#ifndef GZ_RESOURCE_MANAGER_HPP_
#define GZ_RESOURCE_MANAGER_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>

#include "gz_ros2_control/gz_system_interface.hpp"

// Internal resource manager of GazeboSimROS2ControlPlugin. Not installed, only shared with the
// tests.
namespace gz_ros2_control
{
/// \brief Resource manager that loads GazeboSimSystemInterface plugins and initializes them
/// with the simulation before handing them to ros2_control
class GZResourceManager : public hardware_interface::ResourceManager
{
public:
  /// \brief Creates a hardware component from its plugin name, throws std::exception on failure
  using SystemLoader = std::function<
    std::unique_ptr<GazeboSimSystemInterface>(const std::string & plugin_name)>;

  /// \brief Constructor
  /// \param[in] node Node passed to the hardware components
  /// \param[in] ecm Entity component manager
  /// \param[in] enabledJoints Joints of the model, by name
  /// \param[in] system_loader Creates the hardware components, they are loaded with pluginlib
  /// if empty
  GZResourceManager(
    rclcpp::Node::SharedPtr & node,
    gz::sim::EntityComponentManager & ecm,
    std::map<std::string, gz::sim::Entity> enabledJoints,
    SystemLoader system_loader = nullptr);

  GZResourceManager(const GZResourceManager &) = delete;

  /// \brief Called from Controller Manager when robot description is initialized from callback
  /// \return false if the robot description can't be parsed, or a hardware component can't be
  /// loaded or initialized with the simulation
  bool load_and_initialize_components(
    const hardware_interface::ResourceManagerParams & params) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  gz::sim::EntityComponentManager * ecm_;
  std::map<std::string, gz::sim::Entity> enabledJoints_;

  /// \brief Interface loader, only created when no system loader is given
  std::unique_ptr<pluginlib::ClassLoader<GazeboSimSystemInterface>> gz_system_loader_;

  SystemLoader system_loader_;

  rclcpp::Logger logger_;
};
}  // namespace gz_ros2_control

#endif  // GZ_RESOURCE_MANAGER_HPP_
