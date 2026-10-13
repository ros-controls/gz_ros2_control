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

#include "gz_ros2_control_plugin_utils.hpp"

#include <rcl/arguments.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
#include <sdf/Joint.hh>

#include <rclcpp/logging.hpp>

namespace gz_ros2_control::detail
{
//////////////////////////////////////////////////
std::optional<PluginConfig> ParsePluginConfig(
  const sdf::ElementConstPtr & sdf, std::string & error)
{
  PluginConfig config;

  // Get params from SDF
  std::string paramFileName = sdf->Get<std::string>("parameters");

  if (paramFileName.empty()) {
    error = "Gazebo ros2 control found an empty parameters file. Failed to initialize.";
    return std::nullopt;
  }

  sdf::ElementConstPtr argument_sdf_param = sdf->FindElement("parameters");
  while (argument_sdf_param) {
    config.parameter_files.push_back(argument_sdf_param->Get<std::string>());
    argument_sdf_param = argument_sdf_param->GetNextElement("parameters");
  }

  // Get controller manager node name
  if (sdf->HasElement("controller_manager_name")) {
    config.controller_manager_name =
      sdf->FindElement("controller_manager_name")->Get<std::string>();
  }

  // Hold joints if no control mode is active?
  if (sdf->HasElement("hold_joints")) {
    config.hold_joints = sdf->FindElement("hold_joints")->Get<bool>();
  }
  if (sdf->HasElement("position_proportional_gain")) {
    config.position_proportional_gain =
      sdf->FindElement("position_proportional_gain")->Get<double>();
  }

  if (sdf->HasElement("ros")) {
    sdf::ElementConstPtr sdfRos = sdf->FindElement("ros");

    // Set namespace if tag is present
    if (sdfRos->HasElement("namespace")) {
      config.ns = NormalizeNamespace(sdfRos->FindElement("namespace")->Get<std::string>());
    }

    // Get list of remapping rules from SDF
    sdf::ElementConstPtr argument_sdf_remapping = sdfRos->FindElement("remapping");
    while (argument_sdf_remapping) {
      config.remappings.push_back(argument_sdf_remapping->Get<std::string>());
      argument_sdf_remapping = argument_sdf_remapping->GetNextElement("remapping");
    }
  }

  return config;
}

//////////////////////////////////////////////////
std::string NormalizeNamespace(const std::string & ns)
{
  // prevent exception: namespace must be absolute, it must lead with a '/'
  if (ns.empty() || ns[0] != '/') {
    return '/' + ns;
  }
  return ns;
}

//////////////////////////////////////////////////
std::vector<std::string> BuildRosArguments(const PluginConfig & config)
{
  std::vector<std::string> arguments = {"--ros-args"};

  for (const auto & parameter_file : config.parameter_files) {
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(parameter_file);
  }

  if (!config.remappings.empty()) {
    arguments.push_back(RCL_ROS_ARGS_FLAG);
    for (const auto & remapping : config.remappings) {
      arguments.push_back(RCL_REMAP_FLAG);
      arguments.push_back(remapping);
    }
  }

  return arguments;
}

//////////////////////////////////////////////////
std::map<std::string, gz::sim::Entity> GetEnabledJoints(
  const gz::sim::Entity & model,
  const gz::sim::EntityComponentManager & ecm,
  const rclcpp::Logger & logger)
{
  std::map<std::string, gz::sim::Entity> output;

  // Get all available joints
  auto jointEntities = ecm.ChildrenByComponents(model, gz::sim::components::Joint());

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto & jointEntity : jointEntities) {
    const auto jointName = ecm.Component<gz::sim::components::Name>(
      jointEntity)->Data();

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto * jointType = ecm.Component<gz::sim::components::JointType>(jointEntity);
    switch (jointType->Data()) {
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX:
        {
          // Supported joint type
          break;
        }
      case sdf::JointType::FIXED:
        {
          RCLCPP_INFO_STREAM(
            logger,
            "[gz_ros2_control] Fixed joint ['" << jointName << "'] (Entity='" << jointEntity
                                               << "') is skipped.");
          continue;
        }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL:
        {
          RCLCPP_WARN_STREAM(
            logger,
            "[gz_ros2_control] Joint ['" << jointName << "'] (Entity='" << jointEntity
                                         << "') is of unsupported type."
                                           " Only joints with a single axis are supported.");
          continue;
        }
      default:
        {
          RCLCPP_WARN_STREAM(
            logger,
            "[gz_ros2_control] Joint [" << jointName << "] (Entity=" << jointEntity
                                        << ") is of unknown type."
          );
          continue;
        }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

//////////////////////////////////////////////////
PeriodMismatch CheckPeriodMismatch(
  const rclcpp::Duration & control_period,
  const rclcpp::Duration & gazebo_period)
{
  if (control_period < gazebo_period) {
    return PeriodMismatch::kControlFaster;
  }
  if (control_period > gazebo_period) {
    return PeriodMismatch::kControlSlower;
  }
  return PeriodMismatch::kNone;
}
}  // namespace gz_ros2_control::detail
