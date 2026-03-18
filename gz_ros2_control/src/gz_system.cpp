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

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef GZ_HEADERS
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

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
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::
#define GZ_PHYSICS_NAMESPACE gz::physics::
#define GZ_VECTOR_DOT dot
#else
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/wrench.pb.h>

#include <ignition/math/Vector3.hh>
#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/ForceTorque.hh>
#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointTransmittedWrench.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>
#include <ignition/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE ignition::transport::
#define GZ_MSGS_NAMESPACE ignition::msgs::
#define GZ_PHYSICS_NAMESPACE ignition::math::
#define GZ_VECTOR_DOT Dot
#endif

#include <hardware_interface/hardware_info.hpp>

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

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

struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  std::vector<std::string> interfaces_to_mimic;
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

  /// \brief An array per FT
  std::array<double, 6> ft_sensor_data_;

  /// \brief callback to get the Force Torque topic values
  void OnForceTorque(const GZ_MSGS_NAMESPACE Wrench & _msg);
};

void ForceTorqueData::OnForceTorque(const GZ_MSGS_NAMESPACE Wrench & _msg)
{
  this->ft_sensor_data_[0] = _msg.force().x();
  this->ft_sensor_data_[1] = _msg.force().y();
  this->ft_sensor_data_[2] = _msg.force().z();
  this->ft_sensor_data_[3] = _msg.torque().x();
  this->ft_sensor_data_[4] = _msg.torque().y();
  this->ft_sensor_data_[5] = _msg.torque().z();
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

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg);
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg)
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

class gz_ros2_control::GazeboSimSystemPrivate
{
public:
  GazeboSimSystemPrivate() = default;

  ~GazeboSimSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus.
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief vector with the force torque sensors.
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  int * update_rate;

  /// \brief Ignition communication node.
  GZ_TRANSPORT_NAMESPACE Node node;

  /// \brief mapping of mimicked joints to index of joint they mimic
  std::vector<MimicJoint> mimic_joints_;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;
};

namespace gz_ros2_control
{
bool GazeboSimSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  int & update_rate)
{
  this->dataPtr = std::make_unique<GazeboSimSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = &update_rate;

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  constexpr double default_gain = 0.1;

  try {
    this->dataPtr->position_proportional_gain_ = this->nh_->declare_parameter<double>(
      "position_proportional_gain", default_gain);
  } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException & ex) {
    this->nh_->get_parameter(
      "position_proportional_gain", this->dataPtr->position_proportional_gain_);
  }

  RCLCPP_INFO_STREAM(
    this->nh_->get_logger(),
    "The position_proportional_gain has been set to: " <<
      this->dataPtr->position_proportional_gain_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it = enableJoints.find(joint_name);
    if (it == enableJoints.end()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = it->second;
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

    std::string suffix = "";

    // Precompute interface names once to avoid allocations in perform_command_mode_switch
    this->dataPtr->joints_[j].if_name_position =
      joint_name + "/" + hardware_interface::HW_IF_POSITION;
    this->dataPtr->joints_[j].if_name_velocity =
      joint_name + "/" + hardware_interface::HW_IF_VELOCITY;
    this->dataPtr->joints_[j].if_name_effort =
      joint_name + "/" + hardware_interface::HW_IF_EFFORT;

    // check if joint is mimicked
    if (joint_info.parameters.find("mimic") != joint_info.parameters.end()) {
      const auto mimicked_joint = joint_info.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(
        hardware_info.joints.begin(), hardware_info.joints.end(),
        [&mimicked_joint](const hardware_interface::ComponentInfo & info) {
          return info.name == mimicked_joint;
        });
      if (mimicked_joint_it == hardware_info.joints.end()) {
        throw std::runtime_error(
                std::string("Mimicked joint '") + mimicked_joint + "' not found");
      }

      MimicJoint mimic_joint;
      mimic_joint.joint_index = j;
      mimic_joint.mimicked_joint_index = std::distance(
        hardware_info.joints.begin(), mimicked_joint_it);
      auto param_it = joint_info.parameters.find("multiplier");
      if (param_it != joint_info.parameters.end()) {
        mimic_joint.multiplier = std::stod(joint_info.parameters.at("multiplier"));
      } else {
        mimic_joint.multiplier = 1.0;
      }

      // check joint info of mimicked joint
      auto & joint_info_mimicked = hardware_info.joints[mimic_joint.mimicked_joint_index];
      const auto state_mimicked_interface = std::find_if(
        joint_info_mimicked.state_interfaces.begin(), joint_info_mimicked.state_interfaces.end(),
        [&mimic_joint](const hardware_interface::InterfaceInfo & interface_info) {
          bool pos = interface_info.name == "position";
          if (pos) {mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_POSITION);}
          bool vel = interface_info.name == "velocity";
          if (vel) {mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_VELOCITY);}
          bool eff = interface_info.name == "effort";
          if (vel) {mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_EFFORT);}
          return pos || vel || eff;
        });
      if (state_mimicked_interface == joint_info_mimicked.state_interfaces.end()) {
        throw std::runtime_error(
                std::string(
                  "For mimic joint '") + joint_info.name +
                "' no state interface was found in mimicked joint '" + mimicked_joint +
                " ' to mimic");
      }
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Joint '" << joint_name << "'is mimicking joint '"
                  << mimicked_joint << "' with multiplier: "
                  << mimic_joint.multiplier);
      this->dataPtr->mimic_joints_.push_back(mimic_joint);
      suffix = "_mimic";
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value =
      [this, joint_name](const hardware_interface::InterfaceInfo & interface_info) {
        double initial_value{0.0};
        if (!interface_info.initial_value.empty()) {
          try {
            initial_value = std::stod(interface_info.initial_value);
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
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointPositionReset({initial_position}));
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointVelocityReset({initial_velocity}));
      }
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
        this->dataPtr->state_interfaces_.emplace_back(
          imuData->name,
          state_interface.name,
          &imuData->imu_sensor_data_[data_index]);
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
        this->dataPtr->state_interfaces_.emplace_back(
          ftData->name,
          state_interface.name,
          &ftData->ft_sensor_data_[data_index]);
      }
      this->dataPtr->ft_sensors_.push_back(ftData);
      return true;
    });
}

CallbackReturn
GazeboSimSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (system_info.hardware_class_type.compare("gz_ros2_control/GazeboSimSystem") != 0) {
    RCLCPP_WARN(
      this->nh_->get_logger(),
      "The ign_ros2_control plugin got renamed to gz_ros2_control.\n"
      "Update the <ros2_control> tag and gazebo plugin to\n"
      "<hardware>\n"
      "  <plugin>gz_ros2_control/GazeboSimSystem</plugin>\n"
      "</hardware>\n"
      "<gazebo>\n"
      "  <plugin filename=\"gz_ros2_control-system\""
      "name=\"gz_ros2_control::GazeboSimROS2ControlPlugin\">\n"
      "    ...\n"
      "  </plugin>\n"
      "</gazebo>"
    );
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

std::vector<hardware_interface::StateInterface>
GazeboSimSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSimSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
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

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    GZ_PHYSICS_NAMESPACE Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {jointWrench->Data().force().x(),
        jointWrench->Data().force().y(), jointWrench->Data().force().z()};
    } else {  // REVOLUTE and CONTINUOUS
      force_or_torque = {jointWrench->Data().torque().x(),
        jointWrench->Data().torque().y(), jointWrench->Data().torque().z()};
    }
    // Calculate the scalar effort along the joint axis
    this->dataPtr->joints_[i].joint_effort = force_or_torque.GZ_VECTOR_DOT(
      GZ_PHYSICS_NAMESPACE Vector3d{this->dataPtr->joints_[i].joint_axis.Xyz()[0],
        this->dataPtr->joints_[i].joint_axis.Xyz()[1],
        this->dataPtr->joints_[i].joint_axis.Xyz()[2]});
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
      if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointVelocityCmd({0}));
      } else {
        const auto jointVelCmd =
          this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointVelCmd = sim::components::JointVelocityCmd(
          {this->dataPtr->joints_[i].joint_velocity_cmd});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      // Get error in position
      double error;
      error = (this->dataPtr->joints_[i].joint_position -
        this->dataPtr->joints_[i].joint_position_cmd) * *this->dataPtr->update_rate;

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
    } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
      if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
          this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = sim::components::JointForceCmd(
          {this->dataPtr->joints_[i].joint_effort_cmd});
      }
    } else if (this->dataPtr->joints_[i].is_actuated) {
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
  for (const auto & mimic_joint : this->dataPtr->mimic_joints_) {
    for (const auto & mimic_interface : mimic_joint.interfaces_to_mimic) {
      if (mimic_interface == "position") {
        // Get the joint position
        double position_mimicked_joint =
          this->dataPtr->ecm->Component<sim::components::JointPosition>(
          this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

        double position_mimic_joint =
          this->dataPtr->ecm->Component<sim::components::JointPosition>(
          this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)->Data()[0];

        double position_error =
          position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

        double velocity_sp = (-1.0) * position_error * (*this->dataPtr->update_rate);

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
      if (mimic_interface == "velocity") {
        // get the velocity of mimicked joint
        double velocity_mimicked_joint =
          this->dataPtr->ecm->Component<sim::components::JointVelocity>(
          this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

        if (!this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
            sim::components::JointVelocityCmd({0}));
        } else {
          const auto jointVelCmd =
            this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
          *jointVelCmd = sim::components::JointVelocityCmd(
            {mimic_joint.multiplier * velocity_mimicked_joint});
        }
      }
      if (mimic_interface == "effort") {
        // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
        // Get the joint force
        // const auto * jointForce =
        //   _ecm.Component<sim::components::JointForce>(
        //   this->dataPtr->sim_joints_[j]);
        if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
        {
          this->dataPtr->ecm->CreateComponent(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
            sim::components::JointForceCmd({0}));
        } else {
          const auto jointEffortCmd =
            this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
          *jointEffortCmd = sim::components::JointForceCmd(
            {mimic_joint.multiplier *
              this->dataPtr->joints_[mimic_joint.mimicked_joint_index].joint_effort});
        }
      }
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace gz_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_ros2_control::GazeboSimSystem, gz_ros2_control::GazeboSimSystemInterface)
// for backward compatibility with Ignition Gazebo
PLUGINLIB_EXPORT_CLASS(
  ign_ros2_control::IgnitionSystem, gz_ros2_control::GazeboSimSystemInterface)
