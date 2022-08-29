// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef GZ_ROS2_CONTROL__GZ_ROS2_CONTROL_PLUGIN_HPP_
#define GZ_ROS2_CONTROL__GZ_ROS2_CONTROL_PLUGIN_HPP_

#include <memory>

#ifdef GZ_HEADERS
#include <gz/sim/System.hh>
#define GZ_SIM_NAMESPACE gz::sim::
#else
#include <ignition/gazebo/System.hh>
#define GZ_SIM_NAMESPACE ignition::gazebo::
#endif

namespace gz_ros2_control
{
// Forward declarations.
class GazeboSimROS2ControlPluginPrivate;

class GazeboSimROS2ControlPlugin
  : public GZ_SIM_NAMESPACE System,
  public GZ_SIM_NAMESPACE ISystemConfigure,
  public GZ_SIM_NAMESPACE ISystemPreUpdate,
  public GZ_SIM_NAMESPACE ISystemPostUpdate
{
public:
  /// \brief Constructor
  GazeboSimROS2ControlPlugin();

  /// \brief Destructor
  ~GazeboSimROS2ControlPlugin() override;

  // Documentation inherited
  void Configure(
    const GZ_SIM_NAMESPACE Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    GZ_SIM_NAMESPACE EntityComponentManager & _ecm,
    GZ_SIM_NAMESPACE EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const GZ_SIM_NAMESPACE UpdateInfo & _info,
    GZ_SIM_NAMESPACE EntityComponentManager & _ecm) override;

  void PostUpdate(
    const GZ_SIM_NAMESPACE UpdateInfo & _info,
    const GZ_SIM_NAMESPACE EntityComponentManager & _ecm) override;

private:
  /// \brief Private data pointer.
  std::unique_ptr<GazeboSimROS2ControlPluginPrivate> dataPtr;
};
}  // namespace gz_ros2_control

#endif  // GZ_ROS2_CONTROL__GZ_ROS2_CONTROL_PLUGIN_HPP_
