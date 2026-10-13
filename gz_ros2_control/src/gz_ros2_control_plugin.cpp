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

#include <chrono>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Model.hh>
#include <gz/plugin/Register.hh>


#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/duration.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/init_options.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include "gz_ros2_control/gz_ros2_control_plugin.hpp"
#include "gz_ros2_control/gz_system.hpp"
#include "gz_ros2_control_plugin_utils.hpp"

namespace gz_ros2_control
{
class GZResourceManager : public hardware_interface::ResourceManager
{
public:
  GZResourceManager(
    rclcpp::Node::SharedPtr & node,
    sim::EntityComponentManager & ecm,
    std::map<std::string, sim::Entity> enabledJoints)
  : hardware_interface::ResourceManager(
      node->get_node_clock_interface(), node->get_node_logging_interface()),
    gz_system_loader_("gz_ros2_control", "gz_ros2_control::GazeboSimSystemInterface"),
    logger_(node->get_logger().get_child("GZResourceManager"))
  {
    node_ = node;
    ecm_ = &ecm;
    enabledJoints_ = enabledJoints;
  }

  GZResourceManager(const GZResourceManager &) = delete;

  // Called from Controller Manager when robot description is initialized from callback
  bool load_and_initialize_components(
    const hardware_interface::ResourceManagerParams & params) override
  {
    components_are_loaded_and_initialized_ = true;

    const auto hardware_info =
      hardware_interface::parse_control_resources_from_urdf(params.robot_description);

    for (const auto & individual_hardware_info : hardware_info) {
      std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
      RCLCPP_DEBUG(
        logger_, "Load hardware interface %s ...",
        robot_hw_sim_type_str_.c_str());

      // Load hardware
      std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface> gzSimSystem;
      std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
      try {
        gzSimSystem = std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface>(
          gz_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_ERROR(
          logger_,
          "The plugin failed to load for some reason. Error: %s\n",
          ex.what());
        continue;
      }

      // initialize simulation requirements
      bool sim_initialized = false;
      try {
        sim_initialized = gzSimSystem->initSim(
          node_,
          enabledJoints_,
          individual_hardware_info,
          *ecm_,
          params.update_rate);
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          logger_, "Exception while initializing robot simulation interface %s: %s",
          robot_hw_sim_type_str_.c_str(), ex.what());
      }
      if (!sim_initialized) {
        RCLCPP_FATAL(
          logger_, "Could not initialize robot simulation interface");
        components_are_loaded_and_initialized_ = false;
        break;
      }
      RCLCPP_DEBUG(
        logger_, "Initialized robot simulation interface %s!",
        robot_hw_sim_type_str_.c_str());

      hardware_interface::HardwareComponentParams component_params;
      component_params.hardware_info = individual_hardware_info;
      component_params.executor = params.executor;
      component_params.clock = params.clock;
      component_params.logger = params.logger;
      // initialize hardware
      import_component(std::move(gzSimSystem), component_params);
    }

    return components_are_loaded_and_initialized_;
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  sim::EntityComponentManager * ecm_;
  std::map<std::string, sim::Entity> enabledJoints_;

  /// \brief Interface loader
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> gz_system_loader_;

  rclcpp::Logger logger_;
};

//////////////////////////////////////////////////
class GazeboSimROS2ControlPluginPrivate
{
public:
  /// \brief Node Handles
  std::shared_ptr<rclcpp::Node> node_{nullptr};

  /// \brief Thread where the executor will spin
  std::thread thread_executor_spin_;

  /// \brief Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  /// \brief Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  /// \brief Controller manager
  std::shared_ptr<controller_manager::ControllerManager>
  controller_manager_{nullptr};

  /// \brief Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ =
    rclcpp::Time(static_cast<int64_t>(0), RCL_ROS_TIME);

  /// \brief controller update rate
  int update_rate;

  /// \brief Whether the control/sim period mismatch warning has been emitted
  bool period_mismatch_warned_{false};
};

//////////////////////////////////////////////////
GazeboSimROS2ControlPlugin::GazeboSimROS2ControlPlugin()
: dataPtr(std::make_unique<GazeboSimROS2ControlPluginPrivate>())
{
}

//////////////////////////////////////////////////
GazeboSimROS2ControlPlugin::~GazeboSimROS2ControlPlugin()
{
  if (this->dataPtr->executor_) {
    this->dataPtr->executor_->cancel();
  }
  if (this->dataPtr->thread_executor_spin_.joinable()) {
    this->dataPtr->thread_executor_spin_.join();
  }
}

//////////////////////////////////////////////////
void GazeboSimROS2ControlPlugin::Configure(
  const sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  sim::EntityComponentManager & _ecm,
  sim::EventManager &)
{
  rclcpp::Logger logger = rclcpp::get_logger("GazeboSimROS2ControlPlugin");
  // Make sure the controller is attached to a valid model
  const auto model = sim::Model(_entity);
  if (!model.Valid(_ecm)) {
    RCLCPP_ERROR_STREAM(
      logger,
      "[Gazebo ROS 2 Control] Failed to initialize because ["
        << model.Name(_ecm) << "] (Entity=" << _entity << ") is not a model. "
        << "Please make sure that Gazebo ROS 2 Control is attached to a valid model."
    );
    return;
  }

  // Get params from SDF
  std::string error;
  const auto config = detail::ParsePluginConfig(_sdf, error);
  if (!config) {
    RCLCPP_ERROR(logger, "%s", error.c_str());
    return;
  }
  const std::string & controllerManagerNodeName = config->controller_manager_name;
  const std::string & ns = config->ns;

  std::vector<std::string> arguments = detail::BuildRosArguments(*config);

  std::vector<const char *> argv;
  for (const auto & arg : arguments) {
    argv.push_back(reinterpret_cast<const char *>(arg.data()));
  }
  // Create a default context, if not already
  if (!rclcpp::ok()) {
    rclcpp::init(
      static_cast<int>(argv.size()), argv.data(), rclcpp::InitOptions(),
      rclcpp::SignalHandlerOptions::None);
  }

  std::string node_name = "gz_ros_control";

  this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns);
  this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  this->dataPtr->executor_->add_node(this->dataPtr->node_);
  auto spin = [this]()
    {
      this->dataPtr->executor_->spin();
    };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG_STREAM(
    this->dataPtr->node_->get_logger(), "[Gazebo Sim ROS 2 Control] Setting up controller for [" <<
      model.Name(_ecm) << "] (Entity=" << _entity << ")].");

  // Get list of enabled joints
  auto enabledJoints = detail::GetEnabledJoints(
    _entity, _ecm, this->dataPtr->node_->get_logger());

  if (enabledJoints.size() == 0) {
    RCLCPP_DEBUG_STREAM(
      this->dataPtr->node_->get_logger(),
      "[Gazebo ROS 2 Control] There are no available Joints.");
    return;
  }

  try {
    this->dataPtr->node_->declare_parameter(
      "hold_joints", rclcpp::ParameterValue(config->hold_joints));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Parameter 'hold_joints' has already been declared, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParametersException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Parameter 'hold_joints' has invalid name, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Parameter 'hold_joints' value is invalid, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Parameter 'hold_joints' value has wrong type, %s",
      e.what());
  }

  try {
    this->dataPtr->node_->declare_parameter(
      "position_proportional_gain",
      rclcpp::ParameterValue(config->position_proportional_gain));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(),
      "Parameter 'position_proportional_gain' has already been declared, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParametersException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(),
      "Parameter 'position_proportional_gain' has invalid name, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(),
      "Parameter 'position_proportional_gain' value is invalid, %s",
      e.what());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(),
      "Parameter 'position_proportional_gain' value has wrong type, %s",
      e.what());
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<gz_ros2_control::GZResourceManager>(this->dataPtr->node_, _ecm, enabledJoints);

  // Create the controller manager
  RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
  rclcpp::NodeOptions options = controller_manager::get_cm_node_options();
  arguments.push_back("-r");
  arguments.push_back("__node:=" + controllerManagerNodeName);
  arguments.push_back("-r");
  arguments.push_back("__ns:=" + ns);
  // Force setting of use_sim_time parameter
  arguments.push_back("-p");
  arguments.push_back("use_sim_time:=true");
  options.arguments(arguments);
  this->dataPtr->controller_manager_.reset(
    new controller_manager::ControllerManager(
      std::move(resource_manager_),
      this->dataPtr->executor_,
      controllerManagerNodeName,
      this->dataPtr->node_->get_namespace(), options));
  this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

  this->dataPtr->update_rate =
    static_cast<int>(this->dataPtr->controller_manager_->get_update_rate());
  this->dataPtr->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr->update_rate))));

  // Wait for CM to receive robot description from the topic and then initialize Resource Manager
  while (!this->dataPtr->controller_manager_->is_resource_manager_initialized()) {
    RCLCPP_WARN(
      this->dataPtr->node_->get_logger(),
      "Waiting RM to load and initialize hardware...");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  }
}

//////////////////////////////////////////////////
void GazeboSimROS2ControlPlugin::PreUpdate(
  const sim::UpdateInfo & _info,
  sim::EntityComponentManager & /*_ecm*/)
{
  if (!this->dataPtr->controller_manager_) {
    return;
  }
  if (!this->dataPtr->period_mismatch_warned_) {
    rclcpp::Duration gazebo_period(_info.dt);

    // Check the period against the simulation period
    const auto mismatch =
      detail::CheckPeriodMismatch(this->dataPtr->control_period_, gazebo_period);
    if (mismatch == detail::PeriodMismatch::kControlFaster) {
      RCLCPP_ERROR_STREAM(
        this->dataPtr->node_->get_logger(),
        "Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
          " s) is faster than the gazebo simulation period (" <<
          gazebo_period.seconds() << " s).");
    } else if (mismatch == detail::PeriodMismatch::kControlSlower) {
      RCLCPP_WARN_STREAM(
        this->dataPtr->node_->get_logger(),
        " Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
          " s) is slower than the gazebo simulation period (" <<
          gazebo_period.seconds() << " s).");
    }
    this->dataPtr->period_mismatch_warned_ = true;
  }

  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count(), RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
  this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
}

//////////////////////////////////////////////////
void GazeboSimROS2ControlPlugin::PostUpdate(
  const sim::UpdateInfo & _info,
  const sim::EntityComponentManager & /*_ecm*/)
{
  if (!this->dataPtr->controller_manager_) {
    return;
  }
  // Get the simulation time and period
  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count(), RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;

  if (detail::ShouldUpdate(sim_period, this->dataPtr->control_period_)) {
    this->dataPtr->last_update_sim_time_ros_ = sim_time_ros;
    this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
    this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
  }
}
}  // namespace gz_ros2_control

GZ_ADD_PLUGIN(
  gz_ros2_control::GazeboSimROS2ControlPlugin,
  gz::sim::System,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemConfigure,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemPreUpdate,
  gz_ros2_control::GazeboSimROS2ControlPlugin::ISystemPostUpdate)
