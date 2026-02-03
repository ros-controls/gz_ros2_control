// Copyright 2025 AIT Austrian Institute of Technology GmbH
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

#include "gz_ros2_control_demos/gz_custom_system.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

#include "control_toolbox/low_pass_filter.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

struct InterfaceData
{
  /// \brief State interface shared pointer
  hardware_interface::StateInterface::SharedPtr state;

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

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// damping_frequency for a first-order low pass
  std::unique_ptr<control_toolbox::LowPassFilter<double>> lpf;
};

class gz_ros2_control_demos::GazeboSimSystemPrivate
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

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  unsigned int update_rate;
};

namespace gz_ros2_control_demos
{

bool GazeboCustomSimSystem::initSim(
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
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = update_rate;

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);
    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = enableJoints[joint_name];
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
        this->dataPtr->state_interfaces_.push_back(this->dataPtr->joints_[j].effort.state);
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->joints_[j].velocity.command =
          std::make_shared<hardware_interface::CommandInterface>(
          joint_name,
          hardware_interface::HW_IF_VELOCITY);
        if (std::isfinite(initial_velocity)) {
          (void)this->dataPtr->joints_[j].velocity.command->set_value(initial_velocity, true);
        }
        this->dataPtr->command_interfaces_.push_back(this->dataPtr->joints_[j].velocity.command);
      }
      auto it = joint_info.command_interfaces[i].parameters.find("damping_frequency");
      if (it != joint_info.command_interfaces[i].parameters.end()) {
        double damping_frequency = stod(it->second);
        RCLCPP_INFO(
          this->nh_->get_logger(), "\t\t\t with damping_frequency %.2fs.",
          damping_frequency);
        double damping_intensity = 1.0;
        auto it2 = joint_info.command_interfaces[i].parameters.find("damping_intensity");
        if (it2 != joint_info.command_interfaces[i].parameters.end()) {
          damping_intensity = stod(it2->second);
        }
        RCLCPP_INFO(
          this->nh_->get_logger(), "\t\t\t with damping_intensity %.2fs.",
          damping_intensity);
        this->dataPtr->joints_[j].lpf =
          std::make_unique<control_toolbox::LowPassFilter<double>>(
          update_rate, damping_frequency, damping_intensity);
        this->dataPtr->joints_[j].lpf->configure();
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
  }

  return true;
}

CallbackReturn
GazeboCustomSimSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboCustomSimSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
GazeboCustomSimSystem::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  for (auto & si : this->dataPtr->state_interfaces_) {
    state_interfaces.push_back(si);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
GazeboCustomSimSystem::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  for (auto & ci : this->dataPtr->command_interfaces_) {
    command_interfaces.push_back(ci);
  }
  return command_interfaces;
}

CallbackReturn GazeboCustomSimSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GazeboCustomSimSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type GazeboCustomSimSystem::read(
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

    (void)this->dataPtr->joints_[i].position.state->set_value(jointPositions->Data()[0], true);
    (void)this->dataPtr->joints_[i].velocity.state->set_value(jointVelocity->Data()[0], true);
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
    (void)this->dataPtr->joints_[i].effort.state->set_value(effort, true);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboCustomSimSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }
    double velocity_cmd = 0.0;
    if (this->dataPtr->joints_[i].velocity.command &&
      this->dataPtr->joints_[i].velocity.command->get_value(velocity_cmd, true))
    {
      double vel_cmd;
      if (this->dataPtr->joints_[i].lpf && this->dataPtr->joints_[i].lpf->is_configured()) {
        this->dataPtr->joints_[i].lpf->update(velocity_cmd, vel_cmd);
      } else {
        vel_cmd = velocity_cmd;
      }
      this->dataPtr->ecm->SetComponentData<sim::components::JointVelocityCmd>(
        this->dataPtr->joints_[i].sim_joint,
        {vel_cmd});
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace gz_ros2_control_demos

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_ros2_control_demos::GazeboCustomSimSystem, gz_ros2_control::GazeboSimSystemInterface)
