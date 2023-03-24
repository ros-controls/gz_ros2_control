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

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Imu.hh>
#include <ignition/gazebo/components/JointForce.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/gazebo/components/JointVelocityCmd.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>

#include <ignition/gazebo/components/JointAxis.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Joint.hh>

#include <ignition/gazebo/components/LinearAcceleration.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Sensor.hh>

// pid_pos stuff
#include <gz/math/PID.hh>

#include <ignition/transport/Node.hh>

#include <iostream>

#include <hardware_interface/hardware_info.hpp>
#include <normApi.h>

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

  /// \brief PID for position control
  gz::math::PID pid_pos;

  /// \brief PID for velocity control
  gz::math::PID pid_vel;

  /// \brief Control method defined in the URDF for each joint.
  ign_ros2_control::IgnitionSystemInterface::ControlMethod joint_control_method;
};

struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  double mimic_ff_force_scaling = 0.01;
  std::vector<std::string> interfaces_to_mimic;
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

  /// \brief mapping of mimicked joints to index of joint they mimic
  std::vector<MimicJoint> mimic_joints_;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;
};

namespace ign_ros2_control
{
bool IgnitionSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, ignition::gazebo::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  ignition::gazebo::EntityComponentManager & _ecm, int & update_rate)
{
  this->dataPtr = std::make_unique<IgnitionSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = &update_rate;

  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  constexpr double default_gain = 0.1;
  if (!this->nh_->get_parameter_or(
      "position_proportional_gain", this->dataPtr->position_proportional_gain_,
      default_gain))
  {
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "The position_proportional_gain parameter was not defined, defaulting to: " << default_gain);
  }

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  // parameters needed for joint control
  std::vector<std::string> joint_names;
  this->param_node_ =
    rclcpp::Node::make_shared(
    hardware_info.name,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  std::vector<rclcpp::Parameter> param_vec;

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    // add as one of node parameters
    joint_names.push_back(joint_name);

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

    const auto * jointAxis =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointAxis>(
      this->dataPtr->joints_[
        j].sim_joint);

    bool use_cascade_control =
      (hardware_info.joints[j].parameters.find("use_cascade_control") ==
      hardware_info.joints[j].parameters.end()) ?
      false :
      [&]() {
      if (hardware_info.joints[j].parameters.at("use_cascade_control") == "true" ||
        hardware_info.joints[j].parameters.at("use_cascade_control") == "True")
      {
        return true;
      } else {
        return false;
      }
    } ();

    param_vec.push_back(
      rclcpp::Parameter{"mode." + joint_name + ".use_cascade_control",
        use_cascade_control});

    double upper = jointAxis->Data().Upper();
    double lower = jointAxis->Data().Lower();
    double max_velocity = jointAxis->Data().MaxVelocity();
    double max_effort = jointAxis->Data().Effort();

    double dummy_guess_p_pos = 10 * max_velocity / abs(upper - lower);

    // PID parameters
    double p_gain_pos =
      (hardware_info.joints[j].parameters.find(
        "p_pos") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos :
      stod(hardware_info.joints[j].parameters.at("p_pos"));
    double i_gain_pos =
      (hardware_info.joints[j].parameters.find(
        "i_pos") == hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("i_pos"));
    double d_gain_pos =
      (hardware_info.joints[j].parameters.find(
        "d_pos") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos / 100.0 :
      stod(hardware_info.joints[j].parameters.at("d_pos"));
    // set integral max and min component to 50 percent of the max effort
    double iMax_pos =
      (hardware_info.joints[j].parameters.find("iMax_pos") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("iMax_pos"));
    double iMin_pos =
      (hardware_info.joints[j].parameters.find("iMin_pos") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("iMin_pos"));
    double cmdMax_pos =
      (hardware_info.joints[j].parameters.find("cmdMax_pos") ==
      hardware_info.joints[j].parameters.end()) ?
      max_velocity :
      stod(hardware_info.joints[j].parameters.at("cmdMax_pos"));
    double cmdMin_pos =
      (hardware_info.joints[j].parameters.find("cmdMin_pos") ==
      hardware_info.joints[j].parameters.end()) ?
      -1.0 * max_velocity :
      stod(hardware_info.joints[j].parameters.at("cmdMin_pos"));
    double cmdOffset_pos =
      (hardware_info.joints[j].parameters.find("cmdOffset_pos") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("cmdOffset_pos"));

    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".p_pos", p_gain_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_pos", i_gain_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".d_pos", d_gain_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".iMax_pos", iMax_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".iMin_pos", iMin_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmdMax_pos", cmdMax_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmdMin_pos", cmdMin_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmdOffset_pos", cmdOffset_pos});

    this->dataPtr->joints_[j].pid_pos.Init(
      p_gain_pos, i_gain_pos, d_gain_pos, iMax_pos, iMin_pos, cmdMax_pos,
      cmdMin_pos, cmdOffset_pos);

    double p_gain_vel =
      (hardware_info.joints[j].parameters.find(
        "p_vel") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos / 100.0 :
      stod(hardware_info.joints[j].parameters.at("p_vel"));
    double i_gain_vel =
      (hardware_info.joints[j].parameters.find(
        "i_vel") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos / 1000.0 :
      stod(hardware_info.joints[j].parameters.at("i_vel"));
    double d_gain_vel =
      (hardware_info.joints[j].parameters.find(
        "d_vel") == hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("d_vel"));
    // set integral max and min component to 50 percent of the max effort
    double iMax_vel =
      (hardware_info.joints[j].parameters.find("iMax_vel") ==
      hardware_info.joints[j].parameters.end()) ?
      max_effort / 2.0 :
      stod(hardware_info.joints[j].parameters.at("iMax_vel"));
    double iMin_vel =
      (hardware_info.joints[j].parameters.find("iMin_vel") ==
      hardware_info.joints[j].parameters.end()) ?
      -1.0 * max_effort / 2.0 :
      stod(hardware_info.joints[j].parameters.at("iMin_vel"));
    double cmdMax_vel =
      (hardware_info.joints[j].parameters.find("cmdMax_vel") ==
      hardware_info.joints[j].parameters.end()) ?
      max_effort :
      stod(hardware_info.joints[j].parameters.at("cmdMax_vel"));
    double cmdMin_vel =
      (hardware_info.joints[j].parameters.find("cmdMin_vel") ==
      hardware_info.joints[j].parameters.end()) ?
      -1.0 * max_effort :
      stod(hardware_info.joints[j].parameters.at("cmdMin_vel"));
    double cmdOffset_vel =
      (hardware_info.joints[j].parameters.find("cmdOffset_vel") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("cmdOffset_vel"));

    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".p_vel", p_gain_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_vel", i_gain_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".d_vel", d_gain_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".iMax_vel", iMax_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".iMin_vel", iMin_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmdMax_vel", cmdMax_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmdMin_vel", cmdMin_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmdOffset_vel", cmdOffset_vel});

    this->dataPtr->joints_[j].pid_vel.Init(
      p_gain_vel, i_gain_vel, d_gain_vel, iMax_vel, iMin_vel, cmdMax_vel,
      cmdMin_vel, cmdOffset_vel);

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    std::string suffix = "";

    // check if joint is mimicked
    if (joint_info.parameters.find("mimic") != joint_info.parameters.end()) {
      const auto mimicked_joint = joint_info.parameters.at("mimic");
      const auto mimicked_joint_it = std::find_if(
        hardware_info.joints.begin(), hardware_info.joints.end(),
        [&mimicked_joint](const hardware_interface::ComponentInfo & info) {
          return info.name == mimicked_joint;
        });
      if (mimicked_joint_it == hardware_info.joints.end()) {
        throw std::runtime_error(std::string("Mimicked joint '") + mimicked_joint + "' not found");
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

      mimic_joint.mimic_ff_force_scaling =
        (hardware_info.joints[j].parameters.find("mimic_ff_force_scaling") ==
        hardware_info.joints[j].parameters.end()) ?
        mimic_joint.mimic_ff_force_scaling :
        stod(hardware_info.joints[j].parameters.at("mimic_ff_force_scaling"));

      // check joint info of mimicked joint
      auto & joint_info_mimicked = hardware_info.joints[mimic_joint.mimicked_joint_index];
      const auto state_mimicked_interface =
        std::find_if(
        joint_info_mimicked.state_interfaces.begin(), joint_info_mimicked.state_interfaces.end(),
        [&mimic_joint](const hardware_interface::InterfaceInfo & interface_info) {
          bool pos = interface_info.name == "position";
          if (pos) {
            mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_POSITION);
          }
          bool vel = interface_info.name == "velocity";
          if (vel) {
            mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_VELOCITY);
          }
          bool eff = interface_info.name == "effort";
          if (vel) {
            mimic_joint.interfaces_to_mimic.push_back(hardware_interface::HW_IF_EFFORT);
          }
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
        this->nh_->get_logger(), "Joint '" << joint_name << "'is mimicking joint '" << mimicked_joint
                                           << "' with mutiplier: " << mimic_joint.multiplier);
      this->dataPtr->mimic_joints_.push_back(mimic_joint);
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value = [this](const hardware_interface::InterfaceInfo & interface_info) {
        if (!interface_info.initial_value.empty()) {
          double value = std::stod(interface_info.initial_value);
          RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", value);
          return value;
        } else {
          return 0.0;
        }
      };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix, hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix, hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name + suffix, hardware_interface::HW_IF_EFFORT,
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
          joint_name + suffix, hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix, hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name + suffix, hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
    }
  }

  rclcpp::Parameter joint_names_parameter("joints", joint_names);
  if (!this->param_node_->has_parameter("joints")) {
    this->param_node_->set_parameter(joint_names_parameter);
  }
  for (const auto & p : param_vec) {
    if (!this->param_node_->has_parameter(p.get_name())) {
      this->param_node_->set_parameter(p);
    }
  }

  spin_thread_ = std::thread(
    [this]() {
      exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      exec_->add_node(this->param_node_);

      while (rclcpp::ok() && !stop_spin_) {
        exec_->spin_once();
      }
      exec_->remove_node(this->param_node_);
      exec_.reset();
    });

  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(this->param_node_);
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  // update the params
  param_vec.push_back(joint_names_parameter);
  param_listener_->update(param_vec);

  registerSensors(hardware_info);

  return true;
}

void IgnitionSystem::registerSensors(const hardware_interface::HardwareInfo & hardware_info)
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

  this->dataPtr->ecm->Each<ignition::gazebo::components::Imu, ignition::gazebo::components::Name>(
    [&](const ignition::gazebo::Entity & _entity, const ignition::gazebo::components::Imu *,
    const ignition::gazebo::components::Name * _name) -> bool {
      auto imuData = std::make_shared<ImuData>();
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<ignition::gazebo::components::SensorTopic>(
        _entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");
      imuData->name = _name->Data();
      imuData->sim_imu_sensors_ = _entity;

      hardware_interface::ComponentInfo component;
      for (auto & comp : sensor_components_) {
        if (comp.name == _name->Data()) {
          component = comp;
        }
      }

      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0}, {"orientation.y", 1}, {"orientation.z", 2},
        {"orientation.w", 3}, {"angular_velocity.x", 4}, {"angular_velocity.y", 5},
        {"angular_velocity.z", 6}, {"linear_acceleration.x", 7}, {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr->state_interfaces_.emplace_back(
          imuData->name, state_interface.name,
          &imuData->imu_sensor_data_[data_index]);
      }
      this->dataPtr->imus_.push_back(imuData);
      return true;
    });
}

CallbackReturn IgnitionSystem::on_init(const hardware_interface::HardwareInfo & system_info)
{
  RCLCPP_WARN(this->nh_->get_logger(), "On init...");

  if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn IgnitionSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IgnitionSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> IgnitionSystem::export_command_interfaces()
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
  stop_spin_ = true;
  spin_thread_.join();
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type IgnitionSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
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
    // set effort state interface to computed/propagated effort command - passthrough because of ignitionrobotics/ign-physics#124
    this->dataPtr->joints_[i].joint_effort = this->dataPtr->joints_[i].joint_effort_cmd;
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp =
        this->dataPtr->ecm->Component<ignition::gazebo::components::SensorTopic>(
        this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(),
          "IMU " << this->dataPtr->imus_[i]->name << " has a topic name: " <<
            sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
          this->dataPtr->imus_[i].get());
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
IgnitionSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    for (const std::string & interface_name : stop_interfaces) {
      // Clear joint control method bits corresponding to stop interfaces
      if (interface_name ==
        (this->dataPtr->joints_[j].name + "/" + hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string & interface_name : start_interfaces) {
      if (interface_name ==
        (this->dataPtr->joints_[j].name + "/" + hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" +  // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IgnitionSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{
  // refresh params
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    // assuming every joint has axis
    const auto * jointAxis =
      this->dataPtr->ecm->Component<ignition::gazebo::components::JointAxis>(
      this->dataPtr->joints_[
        i].sim_joint);

    // update PIDs
    this->dataPtr->joints_[i].pid_pos.SetPGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].p_pos);
    this->dataPtr->joints_[i].pid_pos.SetIGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_pos);
    this->dataPtr->joints_[i].pid_pos.SetDGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].d_pos);
    this->dataPtr->joints_[i].pid_pos.SetIMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].iMax_pos);
    this->dataPtr->joints_[i].pid_pos.SetIMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].iMin_pos);
    this->dataPtr->joints_[i].pid_pos.SetCmdMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmdMax_pos);
    this->dataPtr->joints_[i].pid_pos.SetCmdMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmdMin_pos);
    this->dataPtr->joints_[i].pid_pos.SetCmdOffset(
      params_.gains.joints_map[this->dataPtr->joints_[i].name].cmdOffset_pos);

    this->dataPtr->joints_[i].pid_vel.SetPGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].p_vel);
    this->dataPtr->joints_[i].pid_vel.SetIGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_vel);
    this->dataPtr->joints_[i].pid_vel.SetDGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].d_vel);
    this->dataPtr->joints_[i].pid_vel.SetIMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].iMax_vel);
    this->dataPtr->joints_[i].pid_vel.SetIMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].iMin_vel);
    this->dataPtr->joints_[i].pid_vel.SetCmdMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmdMax_vel);
    this->dataPtr->joints_[i].pid_vel.SetCmdMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmdMin_vel);
    this->dataPtr->joints_[i].pid_vel.SetCmdOffset(
      params_.gains.joints_map[this->dataPtr->joints_[i].name].cmdOffset_vel);

    if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {
      double velocity = this->dataPtr->joints_[i].joint_velocity;
      double velocity_cmd_clamped = std::clamp(
        this->dataPtr->joints_[i].joint_velocity_cmd,
        -1.0 * jointAxis->Data().MaxVelocity(), jointAxis->Data().MaxVelocity());

      double velocity_error = velocity - velocity_cmd_clamped;

      // calculate target force/torque - output of inner pid
      double target_force = this->dataPtr->joints_[i].pid_vel.Update(
        velocity_error,
        std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

      // remember for potential effort state interface
      this->dataPtr->joints_[i].joint_effort_cmd = target_force;

      auto forceCmd = this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (forceCmd == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          ignition::gazebo::components::JointForceCmd({target_force}));
      } else {
        *forceCmd = ignition::gazebo::components::JointForceCmd({target_force});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      // calculate error with clamped position command
      double position = this->dataPtr->joints_[i].joint_position;
      double position_cmd_clamped = std::clamp(
        this->dataPtr->joints_[i].joint_position_cmd, jointAxis->Data().Lower(),
        jointAxis->Data().Upper());

      double position_error = position - position_cmd_clamped;

      double position_error_sign = copysign(1.0, position_error);

      double position_error_abs_clamped =
        std::clamp(
        std::abs(position_error), 0.0,
        std::abs(jointAxis->Data().Upper() - jointAxis->Data().Lower()));

      // move forward with calculated position error
      position_error = position_error_sign * position_error_abs_clamped;

      double position_or_velocity_error = 0.0;

      // check if cascade control is used for this joint
      if (params_.mode.joints_map[this->dataPtr->joints_[i].name].use_cascade_control) {
        // calculate target velocity - output of outer pid - input to inner pid
        double target_vel = this->dataPtr->joints_[i].pid_pos.Update(
          position_error, std::chrono::duration<double>(
            period.to_chrono<std::chrono::nanoseconds>()));

        double velocity_error =
          this->dataPtr->joints_[i].joint_velocity -
          std::clamp(
          target_vel, -1.0 * jointAxis->Data().MaxVelocity(),
          jointAxis->Data().MaxVelocity());

        // prepare velocity error value for inner pid
        position_or_velocity_error = velocity_error;
      } else {
        // prepare velocity error value for inner pid
        position_or_velocity_error = position_error;
      }

      // calculate target force/torque - output of inner pid
      double target_force = this->dataPtr->joints_[i].pid_vel.Update(
        position_or_velocity_error,
        std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

      // round the force
      target_force = round(target_force * 10000.0) / 10000.0;

      // remember for potential effort state interface
      this->dataPtr->joints_[i].joint_effort_cmd = target_force;

      auto forceCmd = this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (forceCmd == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          ignition::gazebo::components::JointForceCmd({target_force}));
      } else {
        *forceCmd = ignition::gazebo::components::JointForceCmd({target_force});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
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

    // set values of all mimic joints with respect to mimicked joint
    for (const auto & mimic_joint : this->dataPtr->mimic_joints_) {
      for (const auto & mimic_interface : mimic_joint.interfaces_to_mimic) {
        // assuming every mimic joint has axis
        const auto * jointAxis =
          this->dataPtr->ecm->Component<ignition::gazebo::components::JointAxis>(
          this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

        if (mimic_interface == "position") {
          // Get the joint position
          double position_mimicked_joint = this->dataPtr->ecm
            ->Component<ignition::gazebo::components::JointPosition>(
            this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)
            ->Data()[0];

          double position_mimic_joint = this->dataPtr->ecm
            ->Component<ignition::gazebo::components::JointPosition>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)
            ->Data()[0];

          double position_target_from_mimicked_joint = std::clamp(
            position_mimicked_joint * mimic_joint.multiplier,
            jointAxis->Data().Lower(), jointAxis->Data().Upper());

          double position_error = position_mimic_joint - position_target_from_mimicked_joint;

          // round position error for simulation stability
          position_error = round(position_error * 10000.0) / 10000.0;

          double position_error_sign = copysign(1.0, position_error);

          double position_error_abs_clamped = std::clamp(
            std::abs(position_error), 0.0,
            std::abs(jointAxis->Data().Upper() - jointAxis->Data().Lower()));

          position_error = position_error_sign * position_error_abs_clamped;

          double position_or_velocity_error = 0.0;

          // check if cascade control is used for this joint
          if (params_.mode.joints_map[this->dataPtr->joints_[mimic_joint.joint_index].name].
            use_cascade_control)
          {
            // calculate target velocity - output of outer pid - input to inner pid
            double target_vel = this->dataPtr->joints_[mimic_joint.joint_index].pid_pos.Update(
              position_error,
              std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

            // get mimic joint velocity
            double velocity_mimic_joint = this->dataPtr->ecm
              ->Component<ignition::gazebo::components::JointVelocity>(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)
              ->Data()[0];

            position_or_velocity_error =
              velocity_mimic_joint -
              std::clamp(
              target_vel, -1.0 * jointAxis->Data().MaxVelocity(),
              jointAxis->Data().MaxVelocity());
          } else {
            position_or_velocity_error = position_error;
          }

          // set command offset - feed forward term added to the pid output that is clamped by pid max command value
          // while taking into account mimic multiplier
          this->dataPtr->joints_[mimic_joint.joint_index].pid_vel.SetCmdOffset(
            mimic_joint.multiplier *
            this->dataPtr->joints_[mimic_joint.mimicked_joint_index].joint_effort_cmd);

          // calculate target force/torque - output of inner pid
          double target_force = this->dataPtr->joints_[mimic_joint.joint_index].pid_vel.Update(
            position_or_velocity_error,
            std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

          // round force value for simulation stability
          target_force = round(target_force * 10000.0) / 10000.0;

          // remember for potential effort state interface
          this->dataPtr->joints_[mimic_joint.joint_index].joint_effort_cmd = target_force;

          auto forceCmd =
            this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

          if (forceCmd == nullptr) {
            this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
              ignition::gazebo::components::JointForceCmd({target_force}));
          } else {
            *forceCmd = ignition::gazebo::components::JointForceCmd({target_force});
          }
        }
        if (mimic_interface == "velocity") {
          // get the velocity of mimicked joint
          double velocity_mimicked_joint = this->dataPtr->ecm
            ->Component<ignition::gazebo::components::JointVelocity>(
            this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)
            ->Data()[0];

          // get mimic joint velocity
          double velocity_mimic_joint = this->dataPtr->ecm
            ->Component<ignition::gazebo::components::JointVelocity>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)
            ->Data()[0];

          double velocity_error = velocity_mimic_joint - std::clamp(
            mimic_joint.multiplier * velocity_mimicked_joint,
            -1.0 * jointAxis->Data().MaxVelocity(),
            jointAxis->Data().MaxVelocity());

          this->dataPtr->joints_[mimic_joint.joint_index].pid_vel.SetCmdOffset(
            mimic_joint.multiplier *
            this->dataPtr->joints_[mimic_joint.mimicked_joint_index].joint_effort_cmd);

          // calculate target force/torque - output of inner pid
          double target_force = this->dataPtr->joints_[mimic_joint.joint_index].pid_vel.Update(
            velocity_error,
            std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

          auto forceCmd =
            this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
            this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

          if (forceCmd == nullptr) {
            this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
              ignition::gazebo::components::JointForceCmd({target_force}));
          } else {
            *forceCmd = ignition::gazebo::components::JointForceCmd({target_force});
          }
        }
        if (mimic_interface == "effort") {
          // TODO(ahcorde): Revisit this part ignitionrobotics/ign-physics#124
          // Get the joint force
          // const auto * jointForce =
          //   _ecm.Component<ignition::gazebo::components::JointForce>(
          //   this->dataPtr->sim_joints_[j]);
          if (!this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint))
          {
            this->dataPtr->ecm->CreateComponent(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
              ignition::gazebo::components::JointForceCmd({0}));
          } else {
            const auto jointEffortCmd =
              this->dataPtr->ecm->Component<ignition::gazebo::components::JointForceCmd>(
              this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);
            *jointEffortCmd = ignition::gazebo::components::JointForceCmd(
              {mimic_joint.multiplier *
                this->dataPtr->joints_[mimic_joint.mimicked_joint_index].joint_effort_cmd});
          }
        }
      }
    }

    return hardware_interface::return_type::OK;
  }
}

} // namespace ign_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  ign_ros2_control::IgnitionSystem, ign_ros2_control::IgnitionSystemInterface)
