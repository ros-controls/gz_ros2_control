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

#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Physics.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/plugin/Register.hh>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ignition_ros2_control/ignition_ros2_control_plugin.hpp"
#include "ignition_ros2_control/ignition_system.hpp"

namespace ignition_ros2_control
{
//////////////////////////////////////////////////
class IgnitionROS2ControlPluginPrivate
{
public:
  /// \brief Entity ID for sensor within Gazebo.
  ignition::gazebo::Entity entity_;

  // Node Handles
  std::shared_ptr<rclcpp::Node> node;

  // Thread where the executor will sping
  std::thread thread_executor_spin_;

  // Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  // Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  // Interface loader
  boost::shared_ptr<pluginlib::ClassLoader<
      ignition_ros2_control::IgnitionSystemInterface>> robot_hw_sim_loader_;

  // Controller manager
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // String with the robot description param_name
  // TODO(ahcorde): Add param in plugin tag
  std::string robot_description_ = "robot_description";

  // String with the name of the node that contains the robot_description
  // TODO(ahcorde): Add param in plugin tag
  std::string robot_description_node_ = "robot_state_publisher";

  // Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_;

  std::string getURDF(std::string param_name) const
  {
    std::string urdf_string;

    using namespace std::chrono_literals;
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
      node, robot_description_node_);
    while (!parameters_client->wait_for_service(0.5s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          node->get_logger(), "Interrupted while waiting for %s service. Exiting.",
          robot_description_node_.c_str());
        return 0;
      }
      RCLCPP_ERROR(
        node->get_logger(), "%s service not available, waiting again...",
        robot_description_node_.c_str());
    }

    RCLCPP_INFO(
      node->get_logger(), "connected to service!! %s asking for %s",
      robot_description_node_.c_str(),
      param_name.c_str());

    // search and wait for robot_description on param server
    while (urdf_string.empty()) {
      RCLCPP_DEBUG(node->get_logger(), "param_name %s", param_name.c_str());

      try {
        auto f = parameters_client->get_parameters({param_name});
        f.wait();
        std::vector<rclcpp::Parameter> values = f.get();
        urdf_string = values[0].as_string();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node->get_logger(), "%s", e.what());
      }

      if (!urdf_string.empty()) {
        break;
      } else {
        RCLCPP_ERROR(
          node->get_logger(), "ignition_ros2_control plugin is waiting for model"
          " URDF in parameter [%s] on the ROS param server.", param_name.c_str());
      }
      usleep(100000);
    }
    RCLCPP_INFO(node->get_logger(), "Received URDF from param server");

    return urdf_string;
  }

  //////////////////////////////////////////////////

public:
  std::map<std::string, ignition::gazebo::Entity> GetEnabledJoints(
    const ignition::gazebo::Entity & _entity,
    ignition::gazebo::EntityComponentManager & _ecm) const
  {
    std::map<std::string, ignition::gazebo::Entity> output;

    std::vector<std::string> enabledJoints;

    // Get all available joints
    auto jointEntities = _ecm.ChildrenByComponents(_entity, ignition::gazebo::components::Joint());

    // Iterate over all joints and verify whether they can be enabled or not
    for (const auto & jointEntity : jointEntities) {
      const auto jointName = _ecm.Component<ignition::gazebo::components::Name>(
        jointEntity)->Data();

      // Make sure the joint type is supported, i.e. it has exactly one
      // actuated axis
      const auto * jointType = _ecm.Component<ignition::gazebo::components::JointType>(jointEntity);
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
            igndbg << "[ignition_ros2_control] Fixed joint [" << jointName <<
              "(Entity=" << jointEntity << ")] is skipped.\n";
            continue;
          }
        case sdf::JointType::REVOLUTE2:
        case sdf::JointType::SCREW:
        case sdf::JointType::BALL:
        case sdf::JointType::UNIVERSAL:
          {
            ignwarn << "[ignition_ros2_control] Joint [" << jointName <<
              "(Entity=" << jointEntity <<
              ")] is of unsupported type. Only joints with a single axis"
              " are supported.\n";
            continue;
          }
        default:
          {
            ignwarn << "[ignition_ros2_control] Joint [" << jointName <<
              "(Entity=" << jointEntity << ")] is of unknown type.\n";
            continue;
          }
      }
      output[jointName] = jointEntity;
    }

    return output;
  }
};

//////////////////////////////////////////////////
IgnitionROS2ControlPlugin::IgnitionROS2ControlPlugin()
: dataPtr(std::make_unique<IgnitionROS2ControlPluginPrivate>())
{
}

//////////////////////////////////////////////////
void IgnitionROS2ControlPlugin::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager &)
{
  // Make sure the controller is attached to a valid model
  const auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm)) {
    ignerr << "[Ignition ROS 2 Control] Failed to initialize because [" <<
      model.Name(_ecm) << "(Entity=" << _entity <<
      ")] is not a model. Please make sure that"
      " Ignition ROS 2 Control is attached to a valid model.\n";
    return;
  }

  // Get params from SDF
  std::string paramFileName = _sdf->Get<std::string>("parameters");

  if (paramFileName.empty()) {
    ignerr << "Ignition ros2 control found an empty parameters file. " <<
      "Failed to initialize.";
    return;
  }

  std::vector<std::string> arguments = {"--ros-args", "--params-file", paramFileName};

  std::vector<const char *> argv;
  for (const auto & arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }

  if (!rclcpp::ok()) {
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    this->dataPtr->node = rclcpp::Node::make_shared("ignition_ros2_control");
  }
  this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->node);
  auto spin = [this]()
    {
      while (rclcpp::ok()) {
        this->dataPtr->executor_->spin_once();
      }
    };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG_STREAM(
    this->dataPtr->node->get_logger(), "[Ignition ROS 2 Control] Setting up controller for [" <<
      "(Entity=" << _entity << ")].\n");

  ignmsg << "[Ignition ROS 2 Control] Setting up controller for [" <<
    model.Name(_ecm) << "(Entity=" << _entity << ")].\n";

  // Get list of enabled joints
  auto enabledJoints = this->dataPtr->GetEnabledJoints(
    _entity,
    _ecm);

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware;
  try {
    urdf_string = this->dataPtr->getURDF(this->dataPtr->robot_description_);
    control_hardware = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node->get_logger(),
      "Error parsing URDF in ignition_ros2_control plugin, plugin not active : " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<hardware_interface::ResourceManager>();

  try {
    this->dataPtr->robot_hw_sim_loader_.reset(
      new pluginlib::ClassLoader<ignition_ros2_control::IgnitionSystemInterface>(
        "ignition_ros2_control",
        "ignition_ros2_control::IgnitionSystemInterface"));
  } catch (pluginlib::LibraryLoadException & ex) {
    RCLCPP_ERROR(
      this->dataPtr->node->get_logger(), "Failed to create robot simulation interface loader: %s ",
      ex.what());
    return;
  }

  for (unsigned int i = 0; i < control_hardware.size(); ++i) {
    std::string robot_hw_sim_type_str_ = control_hardware[i].hardware_class_type;
    auto gazeboSystem = std::unique_ptr<ignition_ros2_control::IgnitionSystemInterface>(
      this->dataPtr->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));

    if (!gazeboSystem->initSim(
        this->dataPtr->node,
        enabledJoints,
        control_hardware[i],
        _ecm))
    {
      RCLCPP_FATAL(
        this->dataPtr->node->get_logger(), "Could not initialize robot simulation interface");
      return;
    }

    resource_manager_->import_component(std::move(gazeboSystem));
  }
  // Create the controller manager
  RCLCPP_INFO(this->dataPtr->node->get_logger(), "Loading controller_manager");
  this->dataPtr->controller_manager_.reset(
    new controller_manager::ControllerManager(
      std::move(resource_manager_),
      this->dataPtr->executor_,
      "controller_manager"));
  this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

  if (!this->dataPtr->controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node->get_logger(),
      "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto worldEntity =
    _ecm.EntityByComponents(ignition::gazebo::components::World());

  auto physicsComp =
    _ecm.Component<ignition::gazebo::components::Physics>(worldEntity);
  const auto & physicsParams = physicsComp->Data();
  const auto newStepSize =
    std::chrono::duration<double>(physicsParams.MaxStepSize());

  // Get the Gazebo simulation period
  rclcpp::Duration gazebo_period(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(newStepSize)));

  auto cm_update_rate = this->dataPtr->controller_manager_->get_parameter("update_rate").as_int();
  this->dataPtr->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(cm_update_rate))));
  // Check the period against the simulation period
  if (this->dataPtr->control_period_ < gazebo_period) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node->get_logger(),
      "Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
        " s) is faster than the gazebo simulation period (" <<
        gazebo_period.seconds() << " s).");
  } else if (this->dataPtr->control_period_ > gazebo_period) {
    RCLCPP_WARN_STREAM(
      this->dataPtr->node->get_logger(),
      " Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
        " s) is slower than the gazebo simulation period (" <<
        gazebo_period.seconds() << " s).");
  }

  this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void IgnitionROS2ControlPlugin::Update(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
  // Get the simulation time and period
  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count());
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  if (sim_period >= this->dataPtr->control_period_) {
    this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
    this->dataPtr->controller_manager_->read();
    this->dataPtr->controller_manager_->update();
    this->dataPtr->controller_manager_->write();
  }
}
}  // namespace ignition_ros2_control

IGNITION_ADD_PLUGIN(
  ignition_ros2_control::IgnitionROS2ControlPlugin,
  ignition::gazebo::System,
  ignition_ros2_control::IgnitionROS2ControlPlugin::ISystemConfigure,
  ignition_ros2_control::IgnitionROS2ControlPlugin::ISystemUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  ignition_ros2_control::IgnitionROS2ControlPlugin,
  "ignition::gazebo::systems::IgnitionROS2ControlPlugin")
