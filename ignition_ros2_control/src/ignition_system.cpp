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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ignition_ros2_control/ignition_system.hpp"

struct jointData
{
  /// \brief vector with the joint's names.
  std::string name;

  /// \brief vector with the current joint position
  double joint_position;

  /// \brief vector with the current joint velocity
  double joint_velocity;

  /// \brief vector with the current joint effort
  double joint_effort;

  /// \brief vector with the current cmd joint position
  double joint_position_cmd;

  /// \brief vector with the current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief vector with the current cmd joint effort
  double joint_effort_cmd;

  /// \brief handles to the joints from within Gazebo
  ignition::gazebo::Entity sim_joint;

  /// \brief vector with the control method defined in the URDF for each joint.
  ignition_ros2_control::IgnitionSystemInterface::ControlMethod joint_control_method;

  /// \brief The current positions of the joints
  std::shared_ptr<hardware_interface::StateInterface> joint_pos_state;

  /// \brief The current velocities of the joints
  std::shared_ptr<hardware_interface::StateInterface> joint_vel_state;

  /// \brief The current effort forces applied to the joints
  std::shared_ptr<hardware_interface::StateInterface> joint_eff_state;

  /// \brief The position command interfaces of the joints
  std::shared_ptr<hardware_interface::CommandInterface> joint_pos_cmd;

  /// \brief The velocity command interfaces of the joints
  std::shared_ptr<hardware_interface::CommandInterface> joint_vel_cmd;

  /// \brief The effort command interfaces of the joints
  std::shared_ptr<hardware_interface::CommandInterface> joint_eff_cmd;
};

struct ImuData
{
  /// \brief imu's name.
  std::string name;

  std::string state_interface;

  /// \brief handles to the imu from within Gazebo
  ignition::gazebo::Entity sim_imu_sensors_;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief The ROS2 Control state interface for the current imu readings
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> imu_state_;
};

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
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus .
  std::vector<struct ImuData> imus_;

  /// \brief handles to the FT sensors from within Gazebo
  std::vector<ignition::gazebo::Entity> sim_ft_sensors_;

  /// \brief An array per FT sensor for 3D force and torquee
  std::vector<std::array<double, 6>> ft_sensor_data_;

  /// \brief The ROS2 Control state interface for the current FT sensor readings
  std::vector<std::shared_ptr<hardware_interface::StateInterface>> ft_sensor_state_;

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

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %d", this->dataPtr->n_dof_);

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
        this->dataPtr->joints_[j].joint_pos_cmd =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
      } else if (hardware_info.joints[j].command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
        this->dataPtr->joints_[j].joint_vel_cmd =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
      } else if (hardware_info.joints[j].command_interfaces[i].name == "effort") {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->joints_[j].joint_eff_cmd =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name, hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
    // register the state handles
    for (unsigned int i = 0; i < hardware_info.joints[j].state_interfaces.size(); ++i) {
      if (hardware_info.joints[j].state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->joints_[j].joint_pos_state =
          std::make_shared<hardware_interface::StateInterface>(
          joint_name, hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joints_[j].joint_vel_state =
          std::make_shared<hardware_interface::StateInterface>(
          joint_name, hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
      }
      if (hardware_info.joints[j].state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->joints_[j].joint_eff_state =
          std::make_shared<hardware_interface::StateInterface>(
          joint_name, hardware_interface::HW_IF_EFFORT, &this->dataPtr->joints_[j].joint_effort);
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

  this->dataPtr->imus_.resize(n_sensors);

  int imuIndex = 0;

  this->dataPtr->ecm->Each<ignition::gazebo::components::Imu,
            ignition::gazebo::components::Name>(
    [&](const ignition::gazebo::Entity &_entity,
        const ignition::gazebo::components::Imu *,
        const ignition::gazebo::components::Name *_name) -> bool
    {
      if (imuIndex > 0 )
        return true;
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      this->dataPtr->imus_[imuIndex].name = _name->Data();
      this->dataPtr->imus_[imuIndex].sim_imu_sensors_ = _entity;

      hardware_interface::ComponentInfo component;
      for (auto & comp: sensor_components_)
      {
        if (comp.name == _name->Data())
        {
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
        this->dataPtr->imus_[imuIndex].imu_state_.push_back(
          std::make_shared<hardware_interface::StateInterface>(
            this->dataPtr->imus_[imuIndex].name,
            state_interface.name,
            &this->dataPtr->imus_[imuIndex].imu_sensor_data_[data_index]));
      }
      imuIndex++;
      return true;
    });
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

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joints_[i].name,
        hardware_interface::HW_IF_POSITION,
        &this->dataPtr->joints_[i].joint_position));
  }
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joints_[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &this->dataPtr->joints_[i].joint_velocity));
  }
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        this->dataPtr->joints_[i].name,
        hardware_interface::HW_IF_EFFORT,
        &this->dataPtr->joints_[i].joint_effort));
  }

  static const std::vector<std::string> interface_names = {
    "orientation.x",
    "orientation.y",
    "orientation.z",
    "orientation.w",
    "angular_velocity.x",
    "angular_velocity.y",
    "angular_velocity.z",
    "linear_acceleration.x",
    "linear_acceleration.y",
    "linear_acceleration.z"
  };

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    for (unsigned int j = 0; j < this->dataPtr->imus_[i].imu_state_.size(); ++j) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          this->dataPtr->imus_[i].name,
          interface_names[j],
          &this->dataPtr->imus_[i].imu_sensor_data_[j]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
IgnitionSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joints_[i].name,
        hardware_interface::HW_IF_POSITION,
        &this->dataPtr->joints_[i].joint_position_cmd));
  }
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joints_[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &this->dataPtr->joints_[i].joint_velocity_cmd));
  }
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->dataPtr->joints_[i].name,
        hardware_interface::HW_IF_EFFORT,
        &this->dataPtr->joints_[i].joint_effort_cmd));
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
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointVelocity>(
        this->dataPtr->joints_[i].sim_joint);
    // std::cerr << "this->dataPtr->joints_[i].sim_joint " << this->dataPtr->joints_[i].sim_joint << '\n';

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
    std::cerr << "this->dataPtr->imus_[i].sim_imu_sensors_ " << this->dataPtr->imus_[i].sim_imu_sensors_ << '\n';
    const auto * worldPose = this->dataPtr->ecm->Component<ignition::gazebo::components::WorldPose>(
        this->dataPtr->imus_[i].sim_imu_sensors_);
    if (!worldPose)
    {
      std::cerr << "fail! worldPose" << '\n';
      continue;
    }
    this->dataPtr->imus_[i].imu_sensor_data_[0] = worldPose->Data().Rot().X();
    this->dataPtr->imus_[i].imu_sensor_data_[1] = worldPose->Data().Rot().Y();
    this->dataPtr->imus_[i].imu_sensor_data_[2] = worldPose->Data().Rot().Z();
    this->dataPtr->imus_[i].imu_sensor_data_[3] = worldPose->Data().Rot().W();

    // std::cerr << "worldPose->Data() " << std::setprecision(5) << worldPose->Data().Rot().X() << "\t" << worldPose->Data().Rot().X() << "\t" << worldPose->Data().Rot().Z() << "\t" << worldPose->Data().Rot().W() << '\n';

    const auto * angularVel = this->dataPtr->ecm->Component<ignition::gazebo::components::AngularVelocity>(
        this->dataPtr->imus_[i].sim_imu_sensors_);
    if (!angularVel)
    {
      std::cerr << "fail! angularVel" << '\n';
      continue;
    }
    this->dataPtr->imus_[i].imu_sensor_data_[4] = angularVel->Data().X();
    this->dataPtr->imus_[i].imu_sensor_data_[5] = angularVel->Data().Y();
    this->dataPtr->imus_[i].imu_sensor_data_[6] = angularVel->Data().Z();
    // std::cerr << "angularVel->Data() " << std::setprecision(5)  << angularVel->Data().X() << "\t" << angularVel->Data().X() << "\t" << angularVel->Data().Z() << '\n';

    const auto * _linearAcc = this->dataPtr->ecm->Component<ignition::gazebo::components::LinearAcceleration>(
        this->dataPtr->imus_[i].sim_imu_sensors_);
    if (!_linearAcc)
    {
      std::cerr << "fail!" << '\n';
      continue;
    }
    this->dataPtr->imus_[i].imu_sensor_data_[7] = _linearAcc->Data().X();
    this->dataPtr->imus_[i].imu_sensor_data_[8] = _linearAcc->Data().Y();
    this->dataPtr->imus_[i].imu_sensor_data_[9] = _linearAcc->Data().Z();
    std::cerr << "linearAcc->Data() " << std::setprecision(5)  << _linearAcc->Data().X() << "\t" << _linearAcc->Data().X() << "\t" << _linearAcc->Data().Z() << '\n';
  }
  // this->dataPtr->ecm->Each<ignition::gazebo::components::Imu,
  //           ignition::gazebo::components::Name,
  //           ignition::gazebo::components::WorldPose,
  //           ignition::gazebo::components::AngularVelocity,
  //           ignition::gazebo::components::LinearAcceleration>(
  //   [&](const ignition::gazebo::Entity &_entity,
  //       const ignition::gazebo::components::Imu *,
  //       const ignition::gazebo::components::Name *_name,
  //       const ignition::gazebo::components::WorldPose *_worldPose,
  //       const ignition::gazebo::components::AngularVelocity *_angularVel,
  //       const ignition::gazebo::components::LinearAcceleration *_linearAcc) -> bool
  //   {
  //     std::cerr << "worldPose->Data() " << std::setprecision(5) << _entity << "\t" << _name->Data()
  //               << "\t" << _worldPose->Data().Rot().X() << "\t" << _worldPose->Data().Rot().X()
  //               << "\t" << _worldPose->Data().Rot().Z() << "\t" << _worldPose->Data().Rot().W() << '\n';
  //
  //     const auto * parentEntity = this->dataPtr->ecm->Component<ignition::gazebo::components::ParentEntity>(
  //         _entity);
  //     if (parentEntity)
  //     {
  //       const auto * worldPoseParent = this->dataPtr->ecm->Component<ignition::gazebo::components::Pose>(
  //         parentEntity->Data());
  //       if (worldPoseParent)
  //       {
  //         std::cerr << "worldPose->Data() Parent " << std::setprecision(5) << parentEntity->Data()
  //                   << "\t" << worldPoseParent->Data().Rot().X() << "\t" << worldPoseParent->Data().Rot().X()
  //                   << "\t" << worldPoseParent->Data().Rot().Z() << "\t" << worldPoseParent->Data().Rot().W() << '\n';
  //       }
  //       const auto * nameParent = this->dataPtr->ecm->Component<ignition::gazebo::components::Name>(
  //         parentEntity->Data());
  //       if (nameParent)
  //       {
  //         std::cerr << "nameParent " << std::setprecision(5) << parentEntity->Data() << "\t" << nameParent->Data() << "\n";
  //       }
  //     }
  //
  //     std::cerr << "_linearAcc->Data() " << std::setprecision(5) << _entity << "\t" << _name->Data() << "\t" << _linearAcc->Data().X() << "\t" << _linearAcc->Data().X() << "\t" << _linearAcc->Data().Z() << '\n';
  //     std::cerr << "_angularVel->Data() " << std::setprecision(5) << _entity << "\t" << _name->Data() << "\t" << _angularVel->Data().X() << "\t" << _angularVel->Data().X() << "\t" << _angularVel->Data().Z() << '\n';
  //     // unsigned int index = 0;
  //     // for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
  //     //   if (this->dataPtr->imus_[i].name == _name->Data()) {
  //     //     index = i;
  //     //     break;
  //     //   }
  //     // }
  //     // this->dataPtr->imus_[index].imu_sensor_data_[0] = _worldPose->Data().Rot().X();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[1] = _worldPose->Data().Rot().Y();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[2] = _worldPose->Data().Rot().Z();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[3] = _worldPose->Data().Rot().W();
  //     //
  //     // this->dataPtr->imus_[index].imu_sensor_data_[4] = _angularVel->Data().X();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[5] = _angularVel->Data().Y();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[6] = _angularVel->Data().Z();
  //     //
  //     // this->dataPtr->imus_[index].imu_sensor_data_[7] = _linearAcc->Data().X();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[8] = _linearAcc->Data().Y();
  //     // this->dataPtr->imus_[index].imu_sensor_data_[9] = _linearAcc->Data().Z();
  //     return true;
  //   });
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
      if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointPositionReset>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          ignition::gazebo::components::JointPositionReset(
            {this->dataPtr->joints_[i].joint_position}));
        const auto jointPosCmd =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointPositionReset>(
          this->dataPtr->joints_[i].sim_joint);
        *jointPosCmd = ignition::gazebo::components::JointPositionReset(
          {this->dataPtr->joints_[i].joint_position_cmd});
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
}  // namespace ignition_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  ignition_ros2_control::IgnitionSystem, ignition_ros2_control::IgnitionSystemInterface)
