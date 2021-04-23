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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ignition_ros2_control/ignition_system.hpp"

#include "ignition/gazebo/components/JointForce.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointPositionReset.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"

class ignition_ros2_control::IgnitionSystemPrivate
{
public:
  IgnitionSystemPrivate() = default;

  ~IgnitionSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<std::string> joint_names_;

  /// \brief vector with the control method defined in the URDF for each joint.
  std::vector<IgnitionSystemInterface::ControlMethod> joint_control_methods_;

  /// \brief handles to the joints from within Gazebo
  std::vector<ignition::gazebo::Entity> sim_joints_;

  /// \brief vector with the current joint position
  std::vector<double> joint_position_;

  /// \brief vector with the current joint velocity
  std::vector<double> joint_velocity_;

  /// \brief vector with the current joint effort
  std::vector<double> joint_effort_;

  /// \brief vector with the current cmd joint position
  std::vector<double> joint_position_cmd_;

  /// \brief vector with the current cmd joint velocity
  std::vector<double> joint_velocity_cmd_;

  /// \brief vector with the current cmd joint effort
  std::vector<double> joint_effort_cmd_;

  /// \brief The current positions of the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_pos_state_;

  /// \brief The current velocities of the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_vel_state_;

  /// \brief The current effort forces applied to the joints
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> joint_eff_state_;

  /// \brief The position command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_pos_cmd_;

  /// \brief The velocity command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_vel_cmd_;

  /// \brief The effort command interfaces of the joints
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> joint_eff_cmd_;

  ignition::gazebo::EntityComponentManager * ecm;
};

namespace ignition_ros2_control
{

bool IgnitionSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, ignition::gazebo::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  this->dataPtr = std::make_unique<IgnitionSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  RCLCPP_ERROR(this->nh_->get_logger(), "n_dof_ %d", this->dataPtr->n_dof_);

  this->dataPtr->joint_names_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_control_methods_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_pos_state_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_vel_state_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_eff_state_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_position_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_velocity_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_effort_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_pos_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_vel_cmd_.resize(this->dataPtr->n_dof_);
  this->dataPtr->joint_eff_cmd_.resize(this->dataPtr->n_dof_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_WARN_STREAM(this->nh_->get_logger(), "There is not joint available ");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    std::string joint_name = this->dataPtr->joint_names_[j] = hardware_info.joints[j].name;

    ignition::gazebo::Entity simjoint = enableJoints[joint_name];
    if (!simjoint) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the ignition model.");
      continue;
    }
    this->dataPtr->sim_joints_.push_back(simjoint);

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        ignition::gazebo::components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(simjoint, ignition::gazebo::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        ignition::gazebo::components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(simjoint, ignition::gazebo::components::JointVelocity());
    }

    // Create joint force component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        ignition::gazebo::components::JointForce().TypeId()))
    {
      _ecm.CreateComponent(simjoint, ignition::gazebo::components::JointForce());
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); i++) {
      if (hardware_info.joints[j].command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->joint_control_methods_[j] |= POSITION;
        this->dataPtr->joint_pos_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joint_position_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joint_control_methods_[j] |= VELOCITY;
        this->dataPtr->joint_vel_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joint_velocity_cmd_[j]);
      }
      if (hardware_info.joints[j].command_interfaces[i].name == "effort") {
        this->dataPtr->joint_control_methods_[j] |= EFFORT;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->joint_eff_cmd_[j] = std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joint_effort_cmd_[j]);
      }
    }

    RCLCPP_INFO_STREAM(
      this->nh_->get_logger(), "\tState:");
    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); i++) {
      if (hardware_info.joints[j].state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->joint_pos_state_[j] = std::make_shared<hardware_interface::StateInterface>(
          joint_name, hardware_interface::HW_IF_POSITION, &this->dataPtr->joint_position_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joint_vel_state_[j] = std::make_shared<hardware_interface::StateInterface>(
          joint_name, hardware_interface::HW_IF_VELOCITY, &this->dataPtr->joint_velocity_[j]);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->joint_eff_state_[j] = std::make_shared<hardware_interface::StateInterface>(
          joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joint_effort_[j]);
      }
    }
  }
  return true;
}

hardware_interface::return_type
IgnitionSystem::configure(const hardware_interface::HardwareInfo & actuator_info)
{
  if (configure_default(actuator_info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface>
IgnitionSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &this->dataPtr->joint_position_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &this->dataPtr->joint_velocity_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &this->dataPtr->joint_effort_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IgnitionSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_POSITION,
        &this->dataPtr->joint_position_cmd_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_VELOCITY,
        &this->dataPtr->joint_velocity_cmd_[i]));
  }
  for (unsigned int i = 0; i < this->dataPtr->joint_names_.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joint_names_[i],
        hardware_interface::HW_IF_EFFORT,
        &this->dataPtr->joint_effort_cmd_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type IgnitionSystem::start()
{
  status_ = hardware_interface::status::STARTED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IgnitionSystem::stop()
{
  status_ = hardware_interface::status::STOPPED;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IgnitionSystem::read()
{
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocity>(
      this->dataPtr->sim_joints_[j]);

    // TODO(ahcorde): Revisit this part
    // Get the joint force
    // const auto * jointForce =
    //   this->dataPtr->ecm->Component<ignition::gazebo::components::JointForce>(
    //   this->dataPtr->sim_joints_[j]);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointPosition>(
      this->dataPtr->sim_joints_[j]);

    this->dataPtr->joint_position_[j] = jointPositions->Data()[0];
    this->dataPtr->joint_velocity_[j] = jointVelocity->Data()[0];
    // this->dataPtr->joint_effort_[j] = jointForce->Data()[0];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IgnitionSystem::write()
{
  for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
    if (this->dataPtr->joint_control_methods_[j] & VELOCITY) {
      if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
          this->dataPtr->sim_joints_[j]))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->sim_joints_[j],
          ignition::gazebo::components::JointVelocityCmd({0}));
      } else {
        const auto jointVelCmd =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
          this->dataPtr->sim_joints_[j]);
        *jointVelCmd = ignition::gazebo::components::JointVelocityCmd(
          {this->dataPtr->joint_velocity_cmd_[j]});
      }
    }

    if (this->dataPtr->joint_control_methods_[j] & POSITION) {
      if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointPositionReset>(
          this->dataPtr->sim_joints_[j]))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->sim_joints_[j],
          ignition::gazebo::components::JointPositionReset({this->dataPtr->joint_position_[j]}));
        const auto jointPosCmd =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointPositionReset>(
          this->dataPtr->sim_joints_[j]);
        *jointPosCmd = ignition::gazebo::components::JointPositionReset(
          {this->dataPtr->joint_position_cmd_[j]});
      }
    }

    if (this->dataPtr->joint_control_methods_[j] & EFFORT) {
      if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
          this->dataPtr->sim_joints_[j]))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->sim_joints_[j],
          ignition::gazebo::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
          this->dataPtr->sim_joints_[j]);
        *jointEffortCmd = ignition::gazebo::components::JointForceCmd(
          {this->dataPtr->joint_effort_cmd_[j]});
      }
    }
  }

  // // Get the simulation time and period
  // ignition::common::Time gz_time_now = this->dataPtr->parent_model_->GetWorld()->SimTime();
  // rclcpp::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  // rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
  //
  // for (unsigned int j = 0; j < this->dataPtr->joint_names_.size(); j++) {
  //   if (this->dataPtr->joint_control_methods_[j] & POSITION) {
  //     this->dataPtr->sim_joints_[j]->SetPosition(
  //       0, this->dataPtr->joint_position_cmd_[j],
  //       true);
  //   }
  //   if (this->dataPtr->joint_control_methods_[j] & VELOCITY) {
  //     this->dataPtr->sim_joints_[j]->SetVelocity(
  //       0,
  //       this->dataPtr->joint_velocity_cmd_[j]);
  //   }
  //   if (this->dataPtr->joint_control_methods_[j] & EFFORT) {
  //     const double effort =
  //       this->dataPtr->joint_effort_cmd_[j];
  //     this->dataPtr->sim_joints_[j]->SetForce(0, effort);
  //   }
  // }
  //
  // this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;

  return hardware_interface::return_type::OK;
}
}  // namespace ignition_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  ignition_ros2_control::IgnitionSystem, ignition_ros2_control::IgnitionSystemInterface)
