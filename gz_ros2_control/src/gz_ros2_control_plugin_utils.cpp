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

#include <algorithm>
#include <chrono>
#include <cstdint>
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
  sdf::ElementConstPtr argument_sdf_param = sdf->FindElement("parameters");
  while (argument_sdf_param) {
    const std::string paramFileName = argument_sdf_param->Get<std::string>();
    if (paramFileName.empty()) {
      error = "Gazebo ros2 control found an empty parameters file. Failed to initialize.";
      return std::nullopt;
    }
    config.parameter_files.push_back(paramFileName);
    argument_sdf_param = argument_sdf_param->GetNextElement("parameters");
  }

  if (config.parameter_files.empty()) {
    error = "Gazebo ros2 control found no parameters file. Failed to initialize.";
    return std::nullopt;
  }

  // Get controller manager node name
  if (sdf->HasElement("controller_manager_name")) {
    config.controller_manager_name =
      sdf->FindElement("controller_manager_name")->Get<std::string>();
  }

  // Seconds to wait for the hardware to be initialized, 0 waits forever
  if (sdf->HasElement("hardware_init_timeout")) {
    config.hardware_init_timeout = sdf->FindElement("hardware_init_timeout")->Get<double>();
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
  std::string normalized = (ns.empty() || ns[0] != '/') ? '/' + ns : ns;
  // and must not end with a '/', unless it is the root namespace
  while (normalized.size() > 1 && normalized.back() == '/') {
    normalized.pop_back();
  }
  return normalized;
}

//////////////////////////////////////////////////
std::vector<std::string> BuildRosArguments(const PluginConfig & config)
{
  std::vector<std::string> arguments = {"--ros-args"};

  for (const auto & parameter_file : config.parameter_files) {
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(parameter_file);
  }

  for (const auto & remapping : config.remappings) {
    arguments.push_back(RCL_REMAP_FLAG);
    arguments.push_back(remapping);
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
    const auto * nameComponent = ecm.Component<gz::sim::components::Name>(jointEntity);
    if (nameComponent == nullptr) {
      RCLCPP_WARN_STREAM(
        logger,
        "[gz_ros2_control] Joint (Entity=" << jointEntity << ") has no name and is skipped.");
      continue;
    }
    const auto & jointName = nameComponent->Data();

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto * jointType = ecm.Component<gz::sim::components::JointType>(jointEntity);
    if (jointType == nullptr) {
      RCLCPP_WARN_STREAM(
        logger,
        "[gz_ros2_control] Joint [" << jointName << "] (Entity=" << jointEntity
                                    << ") has no type and is skipped.");
      continue;
    }
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

//////////////////////////////////////////////////
EffectivePeriodInfo ComputeEffectivePeriod(
  const rclcpp::Duration & control_period,
  const rclcpp::Duration & gazebo_period)
{
  EffectivePeriodInfo info;
  const int64_t control_ns = control_period.nanoseconds();
  const int64_t gazebo_ns = gazebo_period.nanoseconds();
  if (control_ns <= 0 || gazebo_ns <= 0) {
    return info;
  }

  // ShouldUpdate triggers on the first step with sim_period >= control_period,
  // so updates occur every ceil(control_ns / gazebo_ns) physics steps.
  const int64_t steps = (control_ns - 1) / gazebo_ns + 1;
  const int64_t effective_ns = steps * gazebo_ns;
  info.steps_per_update = steps;
  info.effective_period = rclcpp::Duration(std::chrono::nanoseconds(effective_ns));
  info.effective_rate_hz = 1e9 / static_cast<double>(effective_ns);

  if (control_ns > gazebo_ns) {
    const int64_t overshoot_ns = effective_ns - control_ns;
    // Allow <= 1 ns per physics step (capped below large relative fractions for tiny test
    // durations) to absorb integer nanosecond truncation of 1 / rate frequencies.
    const int64_t rounding_tolerance_ns = std::min<int64_t>(steps, gazebo_ns / 1000);
    info.not_multiple = overshoot_ns > rounding_tolerance_ns;
  }

  return info;
}

//////////////////////////////////////////////////
bool ShouldUpdate(
  const rclcpp::Duration & sim_period,
  const rclcpp::Duration & control_period)
{
  return sim_period >= control_period;
}
}  // namespace gz_ros2_control::detail
