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

#include "ign_ros2_control/ign_system.hpp"

#include <ignition/msgs/imu.pb.h>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>

#include <ignition/transport/Node.hh>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief handles to the joints from within Gazebo
  ignition::gazebo::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  ign_ros2_control::IgnitionSystemInterface::ControlMethod joint_control_method;
};

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  ignition::gazebo::Entity sim_imu_sensors_ = ignition::gazebo::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const ignition::msgs::IMU & _msg);
};

void ImuData::OnIMU(const ignition::msgs::IMU & _msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class ign_ros2_control::IgnitionSystemPrivate
{
public:
  IgnitionSystemPrivate() = default;

  ~IgnitionSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus .
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  ignition::gazebo::EntityComponentManager * ecm;

  /// \brief controller update rate
  int * update_rate;

  /// \brief Ignition communication node.
  ignition::transport::Node node;
};

namespace ign_ros2_control
{

bool IgnitionSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, ignition::gazebo::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  ignition::gazebo::EntityComponentManager & _ecm,
  int & update_rate)
{
  this->dataPtr = std::make_unique<IgnitionSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = &update_rate;

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_WARN_STREAM(this->nh_->get_logger(), "There is not joint available ");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    std::string joint_name = this->dataPtr->joints_[j].name = hardware_info.joints[j].name;

    ignition::gazebo::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;

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
    for (unsigned int i = 0; i < hardware_info.joints[j].command_interfaces.size(); ++i) {
      if (hardware_info.joints[j].command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
      } else if (hardware_info.joints[j].command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
      } else if (hardware_info.joints[j].command_interfaces[i].name == "effort") {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); ++i) {
      if (hardware_info.joints[j].state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort);
      }
    }
  }

  registerSensors(hardware_info);

  return true;
}

void IgnitionSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> sensor_components_;

  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    sensor_components_.push_back(component);
  }
  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures

  this->dataPtr->ecm->Each<ignition::gazebo::components::Imu,
    ignition::gazebo::components::Name>(
    [&](const ignition::gazebo::Entity & _entity,
    const ignition::gazebo::components::Imu *,
    const ignition::gazebo::components::Name * _name) -> bool
    {
      auto imuData = std::make_shared<ImuData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        ignition::gazebo::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      imuData->name = _name->Data();
      imuData->sim_imu_sensors_ = _entity;

      hardware_interface::ComponentInfo component;
      for (auto & comp : sensor_components_) {
        if (comp.name == _name->Data()) {
          component = comp;
        }
      }

      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr->state_interfaces_.emplace_back(
          imuData->name,
          state_interface.name,
          &imuData->imu_sensor_data_[data_index]);
      }
      this->dataPtr->imus_.push_back(imuData);
      return true;
    });
}

CallbackReturn
IgnitionSystem::on_init(const hardware_interface::HardwareInfo & actuator_info)
{
  RCLCPP_WARN(this->nh_->get_logger(), "On init...");
  if (hardware_interface::SystemInterface::on_init(actuator_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn IgnitionSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
IgnitionSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
IgnitionSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn IgnitionSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn IgnitionSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type IgnitionSystem::read()
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocity>(
      this->dataPtr->joints_[i].sim_joint);

    // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
    // Get the joint force
    // const auto * jointForce =
    //   _ecm.Component<ignition::gazebo::components::JointForce>(
    //   this->dataPtr->sim_joints_[j]);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointPosition>(
      this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    // this->dataPtr->joint_effort_[j] = jointForce->Data()[0];
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        ignition::gazebo::components::SensorTopic>(this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "IMU " << this->dataPtr->imus_[i]->name <<
            " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
          this->dataPtr->imus_[i].get());
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IgnitionSystem::write()
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {
      if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          ignition::gazebo::components::JointVelocityCmd({0}));
      } else {
        const auto jointVelCmd =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointVelCmd = ignition::gazebo::components::JointVelocityCmd(
          {this->dataPtr->joints_[i].joint_velocity_cmd});
      }
    }

    if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      // Get error in position
      double error;
      error = (this->dataPtr->joints_[i].joint_position -
        this->dataPtr->joints_[i].joint_position_cmd) * *this->dataPtr->update_rate;

      // Calculate target velcity
      double targetVel = -error;

      auto vel =
        this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocityCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          ignition::gazebo::components::JointVelocityCmd({targetVel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = targetVel;
      }
    }

    if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
      if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          ignition::gazebo::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = ignition::gazebo::components::JointForceCmd(
          {this->dataPtr->joints_[i].joint_effort_cmd});
      }
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace ign_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  ign_ros2_control::IgnitionSystem, ign_ros2_control::IgnitionSystemInterface)
