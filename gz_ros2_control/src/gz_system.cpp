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

#include "gz_ros2_control/gz_system.hpp"

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

struct InterfaceData
{
  /// \brief State interface shared pointer
  hardware_interface::StateInterface::SharedPtr state;

  /// \brief Value of the state to be able to use even when the interface is not defined
  double state_value{std::numeric_limits<double>::quiet_NaN()};

  /// \brief Command interface shared pointer
  hardware_interface::CommandInterface::SharedPtr command;
};

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current state and command joint position
  InterfaceData position;

  /// \brief Current state and command joint velocity
  InterfaceData velocity;

  /// \brief Current state and command joint effort
  InterfaceData effort;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;

  /// \brief Precomputed interface names ("joint_name/position" etc.) to avoid
  /// per-call string allocations in perform_command_mode_switch.
  std::string if_name_position;
  std::string if_name_velocity;
  std::string if_name_effort;
};

class ForceTorqueData
{
public:
  /// \brief force torque sensor's name.
  std::string name{};

  /// \brief force torque sensor's topic name.
  std::string topicName{};

  /// \brief handles to the force torque from within Gazebo
  sim::Entity sim_ft_sensors_ = sim::kNullEntity;

  /// \brief State interfaces for FT sensor data
  std::array<hardware_interface::StateInterface::SharedPtr, 6> state_interfaces_;

  /// \brief callback to get the Force Torque topic values
  void OnForceTorque(const gz::msgs::Wrench & _msg);
};

void ForceTorqueData::OnForceTorque(const gz::msgs::Wrench & _msg)
{
  if (state_interfaces_[0]) {
    (void)state_interfaces_[0]->set_value(_msg.force().x(), true);
  }
  if (state_interfaces_[1]) {
    (void)state_interfaces_[1]->set_value(_msg.force().y(), true);
  }
  if (state_interfaces_[2]) {
    (void)state_interfaces_[2]->set_value(_msg.force().z(), true);
  }
  if (state_interfaces_[3]) {
    (void)state_interfaces_[3]->set_value(_msg.torque().x(), true);
  }
  if (state_interfaces_[4]) {
    (void)state_interfaces_[4]->set_value(_msg.torque().y(), true);
  }
  if (state_interfaces_[5]) {
    (void)state_interfaces_[5]->set_value(_msg.torque().z(), true);
  }
}

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief State interfaces for IMU data
  /// (4 orientation, 3 angular velocity, 3 linear acceleration)
  std::array<hardware_interface::StateInterface::SharedPtr, 10> state_interfaces_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const gz::msgs::IMU & _msg);
};

void ImuData::OnIMU(const gz::msgs::IMU & _msg)
{
  if (state_interfaces_[0]) {
    (void)state_interfaces_[0]->set_value(_msg.orientation().x(), true);
  }
  if (state_interfaces_[1]) {
    (void)state_interfaces_[1]->set_value(_msg.orientation().y(), true);
  }
  if (state_interfaces_[2]) {
    (void)state_interfaces_[2]->set_value(_msg.orientation().z(), true);
  }
  if (state_interfaces_[3]) {
    (void)state_interfaces_[3]->set_value(_msg.orientation().w(), true);
  }
  if (state_interfaces_[4]) {
    (void)state_interfaces_[4]->set_value(_msg.angular_velocity().x(), true);
  }
  if (state_interfaces_[5]) {
    (void)state_interfaces_[5]->set_value(_msg.angular_velocity().y(), true);
  }
  if (state_interfaces_[6]) {
    (void)state_interfaces_[6]->set_value(_msg.angular_velocity().z(), true);
  }
  if (state_interfaces_[7]) {
    (void)state_interfaces_[7]->set_value(_msg.linear_acceleration().x(), true);
  }
  if (state_interfaces_[8]) {
    (void)state_interfaces_[8]->set_value(_msg.linear_acceleration().y(), true);
  }
  if (state_interfaces_[9]) {
    (void)state_interfaces_[9]->set_value(_msg.linear_acceleration().z(), true);
  }
}

class gz_ros2_control::GazeboSimSystemPrivate
{
public:
  GazeboSimSystemPrivate() = default;

  ~GazeboSimSystemPrivate() = default;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus.
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief vector with the force torque sensors.
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  unsigned int update_rate;

  /// \brief Gazebo communication node.
  gz::transport::Node node;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;

  // Should hold the joints if no control_mode is active
  bool hold_joints_ = true;
};

namespace gz_ros2_control
{

bool GazeboSimSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int update_rate)
{
  this->dataPtr = std::make_unique<GazeboSimSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;

  this->dataPtr->update_rate = update_rate;

  try {
    this->dataPtr->hold_joints_ = this->nh_->get_parameter("hold_joints").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  }
  RCLCPP_DEBUG_STREAM(
    this->nh_->get_logger(), "hold_joints (system): " << this->dataPtr->hold_joints_ << std::endl);


  this->dataPtr->joints_.resize(hardware_info.joints.size());

  try {
    this->dataPtr->position_proportional_gain_ =
      this->nh_->get_parameter("position_proportional_gain").as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  }

  RCLCPP_INFO_STREAM(
    this->nh_->get_logger(),
    "The position_proportional_gain has been set to: " <<
      this->dataPtr->position_proportional_gain_);

  if (this->dataPtr->joints_.empty()) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);
    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = it_joint->second;
    this->dataPtr->joints_[j].sim_joint = simjoint;
    this->dataPtr->joints_[j].joint_type = _ecm.Component<sim::components::JointType>(
      simjoint)->Data();
    this->dataPtr->joints_[j].joint_axis = _ecm.Component<sim::components::JointAxis>(
      simjoint)->Data();

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointTransmittedWrench().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
    }

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    // Precompute interface names once to avoid allocations in perform_command_mode_switch
    this->dataPtr->joints_[j].if_name_position =
      joint_name + "/" + hardware_interface::HW_IF_POSITION;
    this->dataPtr->joints_[j].if_name_velocity =
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    this->dataPtr->joints_[j].if_name_effort =
      joint_name + "/" + hardware_interface::HW_IF_EFFORT;

    // check if joint is mimicked
    auto it = std::find_if(
      hardware_info.mimic_joints.begin(),
      hardware_info.mimic_joints.end(),
      [j](const hardware_interface::MimicJoint & mj) {
        return mj.joint_index == j;
      });

    if (it != hardware_info.mimic_joints.end()) {
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Joint '" << joint_name << "'is mimicking joint '" <<
          this->dataPtr->joints_[it->mimicked_joint_index].name <<
          "' with multiplier: " << it->multiplier << " and offset: " << it->offset);
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value =
      [this, joint_name](const hardware_interface::InterfaceInfo & interface_info) {
        double initial_value{0.0};
        if (!interface_info.initial_value.empty()) {
          try {
            initial_value = hardware_interface::stod(interface_info.initial_value);
            RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
          } catch (std::invalid_argument &) {
            RCLCPP_ERROR_STREAM(
              this->nh_->get_logger(),
              "Failed converting initial_value string to real number for the joint "
                << joint_name
                << " and state interface " << interface_info.name
                << ". Actual value of parameter: " << interface_info.initial_value
                << ". Initial value will be set to 0.0");
            throw std::invalid_argument("Failed converting initial_value string");
          }
        }
        return initial_value;
      };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].position.state =
          std::make_shared<hardware_interface::StateInterface>(
          joint_name,
          hardware_interface::HW_IF_POSITION);
        (void)this->dataPtr->joints_[j].position.state->set_value(initial_position, true);
        this->dataPtr->joints_[j].position.state_value = initial_position;
        this->dataPtr->state_interfaces_.push_back(this->dataPtr->joints_[j].position.state);
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].velocity.state =
          std::make_shared<hardware_interface::StateInterface>(
          joint_name,
          hardware_interface::HW_IF_VELOCITY);
        (void)this->dataPtr->joints_[j].velocity.state->set_value(initial_velocity, true);
        this->dataPtr->joints_[j].velocity.state_value = initial_velocity;
        this->dataPtr->state_interfaces_.push_back(this->dataPtr->joints_[j].velocity.state);
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].effort.state =
          std::make_shared<hardware_interface::StateInterface>(
          joint_name,
          hardware_interface::HW_IF_EFFORT);
        (void)this->dataPtr->joints_[j].effort.state->set_value(initial_effort, true);
        this->dataPtr->joints_[j].effort.state_value = initial_effort;
        this->dataPtr->state_interfaces_.push_back(this->dataPtr->joints_[j].effort.state);
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->joints_[j].position.command =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name,
          hardware_interface::HW_IF_POSITION);
        if (std::isfinite(initial_position)) {
          (void)this->dataPtr->joints_[j].position.command->set_value(initial_position, true);
        }
        this->dataPtr->command_interfaces_.push_back(this->dataPtr->joints_[j].position.command);
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joints_[j].velocity.command =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name,
          hardware_interface::HW_IF_VELOCITY);
        if (std::isfinite(initial_velocity)) {
          (void)this->dataPtr->joints_[j].velocity.command->set_value(initial_velocity, true);
        }
        this->dataPtr->command_interfaces_.push_back(this->dataPtr->joints_[j].velocity.command);
      } else if (joint_info.command_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->joints_[j].effort.command =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name,
          hardware_interface::HW_IF_EFFORT);
        if (std::isfinite(initial_effort)) {
          (void)this->dataPtr->joints_[j].effort.command->set_value(initial_effort, true);
        }
        this->dataPtr->command_interfaces_.push_back(this->dataPtr->joints_[j].effort.command);
      }
    }
    // independently of existence of command interface set initial value if defined
    if (std::isfinite(initial_position)) {
      this->dataPtr->ecm->CreateComponent(
        this->dataPtr->joints_[j].sim_joint,
        sim::components::JointPositionReset({initial_position}));
    }
    if (std::isfinite(initial_velocity)) {
      this->dataPtr->ecm->CreateComponent(
        this->dataPtr->joints_[j].sim_joint,
        sim::components::JointVelocityReset({initial_velocity}));
    }

    // check if joint is actuated (has command interfaces) or passive
    this->dataPtr->joints_[j].is_actuated = (joint_info.command_interfaces.size() > 0);
  }

  registerSensors(hardware_info);

  return true;
}

void GazeboSimSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Reference sensor components directly from hardware_info — no copy needed.
  const auto & sensor_components_ = hardware_info.sensors;

  // Build a name→ComponentInfo lookup map so each ECM Each<> callback is O(1)
  // instead of doing a linear scan over all sensor_components_ entries.
  std::unordered_map<std::string, const hardware_interface::ComponentInfo *> sensor_map;
  sensor_map.reserve(sensor_components_.size());
  for (const auto & comp : sensor_components_) {
    sensor_map.emplace(comp.name, &comp);
  }

  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures

  this->dataPtr->ecm->Each<sim::components::Imu,
    sim::components::Name>(
    [&](const sim::Entity & _entity,
    const sim::components::Imu *,
    const sim::components::Name * _name) -> bool
    {
      auto imuData = std::make_shared<ImuData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      imuData->name = _name->Data();
      imuData->sim_imu_sensors_ = _entity;

      auto sensor_it = sensor_map.find(_name->Data());
      if (sensor_it == sensor_map.end()) {
        RCLCPP_WARN_STREAM(
          this->nh_->get_logger(),
          "IMU sensor '" << _name->Data() << "' not found in hardware_info, skipping.");
        return true;
      }
      const hardware_interface::ComponentInfo & component = *sensor_it->second;

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
        auto si = std::make_shared<hardware_interface::StateInterface>(
          imuData->name,
          state_interface.name);
        imuData->state_interfaces_[data_index] = si;
        this->dataPtr->state_interfaces_.push_back(si);
      }
      this->dataPtr->imus_.push_back(imuData);
      return true;
    });

  this->dataPtr->ecm->Each<sim::components::ForceTorque,
    sim::components::Name>(
    [&](const sim::Entity & _entity,
    const sim::components::ForceTorque *,
    const sim::components::Name * _name) -> bool
    {
      auto ftData = std::make_shared<ForceTorqueData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      ftData->name = _name->Data();
      ftData->sim_ft_sensors_ = _entity;

      auto sensor_it = sensor_map.find(_name->Data());
      if (sensor_it == sensor_map.end()) {
        RCLCPP_WARN_STREAM(
          this->nh_->get_logger(),
          "ForceTorque sensor '" << _name->Data() << "' not found in hardware_info, skipping.");
        return true;
      }
      const hardware_interface::ComponentInfo & component = *sensor_it->second;

      static const std::map<std::string, size_t> interface_name_map = {
        {"force.x", 0},
        {"force.y", 1},
        {"force.z", 2},
        {"torque.x", 3},
        {"torque.y", 4},
        {"torque.z", 5},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        auto si = std::make_shared<hardware_interface::StateInterface>(
          ftData->name,
          state_interface.name);
        ftData->state_interfaces_[data_index] = si;
        this->dataPtr->state_interfaces_.push_back(si);
      }
      this->dataPtr->ft_sensors_.push_back(ftData);
      return true;
    });
}

CallbackReturn
GazeboSimSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSimSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
GazeboSimSystem::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> const_state_interfaces;
  const_state_interfaces.reserve(this->dataPtr->state_interfaces_.size());
  for (const auto & state_interface : this->dataPtr->state_interfaces_) {
    const_state_interfaces.push_back(state_interface);
  }
  return const_state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
GazeboSimSystem::on_export_command_interfaces()
{
  return this->dataPtr->command_interfaces_;
}

CallbackReturn GazeboSimSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GazeboSimSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type GazeboSimSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<sim::components::JointVelocity>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint force via joint transmitted wrench
    const auto * jointWrench =
      this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[i].sim_joint);

    if (!jointPositions || !jointVelocity || !jointWrench) {
      continue;
    }

    this->dataPtr->joints_[i].position.state_value = jointPositions->Data()[0];
    this->dataPtr->joints_[i].velocity.state_value = jointVelocity->Data()[0];
    if (this->dataPtr->joints_[i].position.state) {
      (void)this->dataPtr->joints_[i].position.state->set_value(jointPositions->Data()[0], true);
    }
    if (this->dataPtr->joints_[i].velocity.state) {
      (void)this->dataPtr->joints_[i].velocity.state->set_value(jointVelocity->Data()[0], true);
    }
    gz::physics::Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {jointWrench->Data().force().x(),
        jointWrench->Data().force().y(), jointWrench->Data().force().z()};
    } else {  // REVOLUTE and CONTINUOUS
      force_or_torque = {jointWrench->Data().torque().x(),
        jointWrench->Data().torque().y(), jointWrench->Data().torque().z()};
    }
    // Calculate the scalar effort along the joint axis
    double effort = force_or_torque.dot(
      gz::physics::Vector3d{this->dataPtr->joints_[i].joint_axis.Xyz()[0],
        this->dataPtr->joints_[i].joint_axis.Xyz()[1],
        this->dataPtr->joints_[i].joint_axis.Xyz()[2]});
    this->dataPtr->joints_[i].effort.state_value = effort;
    if (this->dataPtr->joints_[i].effort.state) {
      (void)this->dataPtr->joints_[i].effort.state->set_value(effort, true);
    }
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(this->dataPtr->imus_[i]->sim_imu_sensors_);
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

  for (unsigned int i = 0; i < this->dataPtr->ft_sensors_.size(); ++i) {
    if (this->dataPtr->ft_sensors_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(this->dataPtr->ft_sensors_[i]->sim_ft_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->ft_sensors_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "ForceTorque " << this->dataPtr->ft_sensors_[i]->name <<
            " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->ft_sensors_[i]->topicName, &ForceTorqueData::OnForceTorque,
          this->dataPtr->ft_sensors_[i].get());
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
GazeboSimSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    for (const std::string & interface_name : stop_interfaces) {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name == this->dataPtr->joints_[j].if_name_position) {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == this->dataPtr->joints_[j].if_name_velocity) {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == this->dataPtr->joints_[j].if_name_effort) {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string & interface_name : start_interfaces) {
      if (interface_name == this->dataPtr->joints_[j].if_name_position) {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_velocity) {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == this->dataPtr->joints_[j].if_name_effort) {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSimSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {
      double velocity_cmd = 0.0;
      if (this->dataPtr->joints_[i].velocity.command->get_value(velocity_cmd, true)) {
        this->dataPtr->ecm->SetComponentData<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint,
          {velocity_cmd});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      // Get error in position
      double position_cmd = 0.0;
      if (this->dataPtr->joints_[i].position.command->get_value(position_cmd, true)) {
        double error = (this->dataPtr->joints_[i].position.state_value - position_cmd) *
          this->dataPtr->update_rate;

        // Calculate target velcity
        double target_vel = -this->dataPtr->position_proportional_gain_ * error;

        auto vel =
          this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint);

        if (vel == nullptr) {
          this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[i].sim_joint,
            sim::components::JointVelocityCmd({target_vel}));
        } else if (!vel->Data().empty()) {
          vel->Data()[0] = target_vel;
        }
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
      if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointForceCmd({0}));
      } else {
        double effort_cmd = 0.0;
        if (this->dataPtr->joints_[i].effort.command->get_value(effort_cmd, true)) {
          const auto jointEffortCmd =
            this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
            this->dataPtr->joints_[i].sim_joint);
          *jointEffortCmd = sim::components::JointForceCmd({effort_cmd});
        }
      }
    } else if (this->dataPtr->joints_[i].is_actuated && this->dataPtr->hold_joints_) {
      // Fallback case is a velocity command of zero (only for actuated joints)
      double target_vel = 0.0;
      auto vel =
        this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointVelocityCmd({target_vel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      }
    }
  }

  // set values of all mimic joints with respect to mimicked joint
  for (const auto & mimic_joint : this->info_.mimic_joints) {
    // Get the joint position
    double position_mimicked_joint =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

    double position_mimic_joint =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)->Data()[0];

    double position_error =
      position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

    double velocity_sp = (-1.0) * position_error * this->dataPtr->update_rate;

    auto vel =
      this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
      this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

    if (vel == nullptr) {
      this->dataPtr->ecm->CreateComponent(
        this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
        sim::components::JointVelocityCmd({velocity_sp}));
    } else if (!vel->Data().empty()) {
      vel->Data()[0] = velocity_sp;
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace gz_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_ros2_control::GazeboSimSystem, gz_ros2_control::GazeboSimSystemInterface)
