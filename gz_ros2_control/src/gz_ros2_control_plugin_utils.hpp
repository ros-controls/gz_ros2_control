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

#ifndef GZ_ROS2_CONTROL_PLUGIN_UTILS_HPP_
#define GZ_ROS2_CONTROL_PLUGIN_UTILS_HPP_

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <sdf/Element.hh>

#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>

// Internal helpers of GazeboSimROS2ControlPlugin. Not installed, only shared with the tests.
namespace gz_ros2_control::detail
{
/// \brief Plugin settings read from the <plugin> SDF element
struct PluginConfig
{
  /// \brief Values of all the <parameters> elements, in order
  std::vector<std::string> parameter_files;

  /// \brief Value of <controller_manager_name>
  std::string controller_manager_name{"controller_manager"};

  /// \brief Value of <hardware_init_timeout>, seconds to wait for the hardware to be
  /// initialized, 0 waits forever
  double hardware_init_timeout{0.0};

  /// \brief Value of <hold_joints>
  bool hold_joints{true};

  /// \brief Value of <position_proportional_gain>
  double position_proportional_gain{0.1};

  /// \brief Value of <ros><namespace>, normalized with NormalizeNamespace
  std::string ns{"/"};

  /// \brief Values of all the <ros><remapping> elements, in order
  std::vector<std::string> remappings;
};

/// \brief Read the plugin settings from its SDF element
/// \param[in] sdf The <plugin> element
/// \param[out] error Reason for the failure, if any
/// \return The settings, or std::nullopt if a <parameters> element is missing or empty
std::optional<PluginConfig> ParsePluginConfig(
  const sdf::ElementConstPtr & sdf, std::string & error);

/// \brief Make a namespace valid for rclcpp: add a leading '/' and remove trailing ones
std::string NormalizeNamespace(const std::string & ns);

/// \brief Build the ROS arguments used to initialize rclcpp
std::vector<std::string> BuildRosArguments(const PluginConfig & config);

/// \brief Get the joints of a model that can be controlled, i.e. those with a single axis
/// \param[in] model Model entity
/// \param[in] ecm Entity component manager
/// \param[in] logger Logger used to report the skipped joints
/// \return Map from joint name to joint entity
std::map<std::string, gz::sim::Entity> GetEnabledJoints(
  const gz::sim::Entity & model,
  const gz::sim::EntityComponentManager & ecm,
  const rclcpp::Logger & logger);

/// \brief How the controller period compares to the simulation period
enum class PeriodMismatch
{
  kNone,
  kControlFaster,
  kControlSlower,
};

/// \brief Compare the controller period with the simulation period
PeriodMismatch CheckPeriodMismatch(
  const rclcpp::Duration & control_period,
  const rclcpp::Duration & gazebo_period);

/// \brief Effective controller timing resulting from discrete simulation steps
struct EffectivePeriodInfo
{
  /// \brief True when control_period exceeds gazebo_period and is not an integer multiple
  ///        (within 1 ns per simulation step rounding tolerance)
  bool not_multiple{false};

  /// \brief Number of simulation steps between consecutive controller updates
  int64_t steps_per_update{0};

  /// \brief Actual period between controller updates
  rclcpp::Duration effective_period{0, 0};

  /// \brief Effective controller update frequency in Hz
  double effective_rate_hz{0.0};
};

/// \brief Compute the effective controller update period and rate given the simulation step
/// \param[in] control_period Desired controller update period
/// \param[in] gazebo_period Simulation physics step period
/// \return Effective timing details; steps_per_update is 0 if either period is non-positive
EffectivePeriodInfo ComputeEffectivePeriod(
  const rclcpp::Duration & control_period,
  const rclcpp::Duration & gazebo_period);

/// \brief Whether enough simulation time has passed to run the controllers again
/// \param[in] sim_period Simulation time elapsed since the last controller update
/// \param[in] control_period Controller update period
bool ShouldUpdate(
  const rclcpp::Duration & sim_period,
  const rclcpp::Duration & control_period);
}  // namespace gz_ros2_control::detail

#endif  // GZ_ROS2_CONTROL_PLUGIN_UTILS_HPP_
