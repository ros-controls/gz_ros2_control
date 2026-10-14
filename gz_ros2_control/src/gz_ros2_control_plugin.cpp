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

#include <algorithm>
#include <chrono>
#include <exception>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Name.hh>
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

namespace gz_ros2_control
{
namespace
{
/// \brief Declare a parameter on the node, logging an error instead of throwing on failure
template<typename T>
void declare_parameter_or_log(
  const rclcpp::Node::SharedPtr & node, const std::string & name, const T & default_value)
{
  try {
    node->declare_parameter(name, rclcpp::ParameterValue(default_value));
  } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException & e) {
    RCLCPP_ERROR(
      node->get_logger(), "Parameter '%s' has already been declared, %s", name.c_str(), e.what());
  } catch (const rclcpp::exceptions::InvalidParametersException & e) {
    RCLCPP_ERROR(
      node->get_logger(), "Parameter '%s' has invalid name, %s", name.c_str(), e.what());
  } catch (const rclcpp::exceptions::InvalidParameterValueException & e) {
    RCLCPP_ERROR(
      node->get_logger(), "Parameter '%s' value is invalid, %s", name.c_str(), e.what());
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    RCLCPP_ERROR(
      node->get_logger(), "Parameter '%s' value has wrong type, %s", name.c_str(), e.what());
  }
}
}  // namespace

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
  /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _ecm Gazebo Entity Component Manager
  /// \return List of entities containing all enabled joints
  std::map<std::string, sim::Entity> GetEnabledJoints(
    const sim::Entity & _entity,
    sim::EntityComponentManager & _ecm) const;

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
  int update_rate{0};

  /// \brief Whether the control/sim period mismatch warning has been emitted
  bool period_mismatch_warned_{false};

  /// \brief Stop and release the controller manager after a failed initialization, so it can
  /// no longer initialize hardware from the executor thread
  void RemoveControllerManager();
};

//////////////////////////////////////////////////
void GazeboSimROS2ControlPluginPrivate::RemoveControllerManager()
{
  this->executor_->remove_node(this->controller_manager_);
  this->controller_manager_.reset();
}

//////////////////////////////////////////////////
std::map<std::string, sim::Entity>
GazeboSimROS2ControlPluginPrivate::GetEnabledJoints(
  const sim::Entity & _entity,
  sim::EntityComponentManager & _ecm) const
{
  std::map<std::string, sim::Entity> output;

  // Get all available joints
  auto jointEntities = _ecm.ChildrenByComponents(_entity, sim::components::Joint());

  // Iterate over all joints and verify whether they can be enabled or not
  for (const auto & jointEntity : jointEntities) {
    const auto jointName = _ecm.Component<sim::components::Name>(
      jointEntity)->Data();

    // Make sure the joint type is supported, i.e. it has exactly one
    // actuated axis
    const auto * jointType = _ecm.Component<sim::components::JointType>(jointEntity);
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
          RCLCPP_INFO_STREAM(
            node_->get_logger(),
            "[gz_ros2_control] Fixed joint ['" << jointName << "'] (Entity='" << jointEntity
                                               << "') is skipped.");
          continue;
        }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL:
        {
          RCLCPP_WARN_STREAM(
            node_->get_logger(),
            "[gz_ros2_control] Joint ['" << jointName << "'] (Entity='" << jointEntity
                                         << "') is of unsupported type."
                                           " Only joints with a single axis are supported.");
          continue;
        }
      default:
        {
          RCLCPP_WARN_STREAM(
            node_->get_logger(),
            "[gz_ros2_control] Joint [" << jointName << "] (Entity=" << jointEntity
                                        << ") is of unknown type."
          );
          continue;
        }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

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
  std::string paramFileName = _sdf->Get<std::string>("parameters");

  if (paramFileName.empty()) {
    RCLCPP_ERROR(
      logger,
      "Gazebo ros2 control found an empty parameters file. Failed to initialize.");
    return;
  }

  // Get params from SDF
  std::vector<std::string> arguments = {"--ros-args"};

  sdf::ElementConstPtr argument_sdf_param = _sdf->FindElement("parameters");
  while (argument_sdf_param) {
    std::string argument = argument_sdf_param->Get<std::string>();
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(argument);
    argument_sdf_param = argument_sdf_param->GetNextElement("parameters");
  }

  // Get controller manager node name
  std::string controllerManagerNodeName{"controller_manager"};

  if (_sdf->HasElement("controller_manager_name")) {
    controllerManagerNodeName = _sdf->FindElement("controller_manager_name")->Get<std::string>();
  }

  // Seconds to wait for the hardware to be initialized, 0 waits forever
  double hardware_init_timeout = 0.0;  // default
  if (_sdf->HasElement("hardware_init_timeout")) {
    hardware_init_timeout = _sdf->FindElement("hardware_init_timeout")->Get<double>();
  }

  std::string ns = "/";

  // Hold joints if no control mode is active?
  bool hold_joints = true;  // default
  if (_sdf->HasElement("hold_joints")) {
    hold_joints =
      _sdf->FindElement("hold_joints")->Get<bool>();
  }
  double position_proportional_gain = 0.1;  // default
  if (_sdf->HasElement("position_proportional_gain")) {
    position_proportional_gain =
      _sdf->FindElement("position_proportional_gain")->Get<double>();
  }

  if (_sdf->HasElement("ros")) {
    sdf::ElementConstPtr sdfRos = _sdf->FindElement("ros");

    // Set namespace if tag is present
    if (sdfRos->HasElement("namespace")) {
      ns = sdfRos->FindElement("namespace")->Get<std::string>();
      // prevent exception: namespace must be absolute, it must lead with a '/'
      if (ns.empty() || ns[0] != '/') {
        ns = '/' + ns;
      }
    }

    // Get list of remapping rules from SDF
    if (sdfRos->HasElement("remapping")) {
      sdf::ElementConstPtr argument_sdf_remapping = sdfRos->FindElement("remapping");

      arguments.push_back(RCL_ROS_ARGS_FLAG);
      while (argument_sdf_remapping) {
        std::string argument = argument_sdf_remapping->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf_remapping = argument_sdf_remapping->GetNextElement("remapping");
      }
    }
  }

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

  // Carry the plugin's ROS arguments (parameter files, remappings) on the node
  // itself instead of relying only on the context's global arguments. The node's
  // parameter overrides are resolved once at construction time from whatever the
  // context holds at that moment, so when the context was already initialized
  // (e.g. by another plugin instance in the same process) the <parameters> YAML
  // would otherwise never reach this node and settings such as
  // position_proportional_gain would silently fall back to their defaults.
  // Node-local arguments take precedence over global ones. See #962.
  try {
    rclcpp::NodeOptions node_options;
    node_options.arguments(arguments);
    this->dataPtr->node_ =
      rclcpp::Node::make_shared(node_name, ns, node_options);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger, "Failed to create gz_ros2_control node: %s", ex.what());
    return;
  }
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
  auto enabledJoints = this->dataPtr->GetEnabledJoints(
    _entity,
    _ecm);

  if (enabledJoints.size() == 0) {
    RCLCPP_DEBUG_STREAM(
      this->dataPtr->node_->get_logger(),
      "[Gazebo ROS 2 Control] There are no available Joints.");
    return;
  }

  declare_parameter_or_log(this->dataPtr->node_, "hold_joints", hold_joints);
  declare_parameter_or_log(
    this->dataPtr->node_, "position_proportional_gain", position_proportional_gain);

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
  if (this->dataPtr->update_rate <= 0) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(),
      "Controller manager update_rate must be positive, got %d. Failed to initialize.",
      this->dataPtr->update_rate);
    this->dataPtr->RemoveControllerManager();
    return;
  }
  this->dataPtr->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr->update_rate))));

  // Wait for CM to receive robot description from the topic and then initialize Resource Manager
  const auto wait_start = std::chrono::steady_clock::now();
  const auto wait_timeout = std::chrono::duration<double>(hardware_init_timeout);
  while (!this->dataPtr->controller_manager_->is_resource_manager_initialized()) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->dataPtr->node_->get_logger(),
        "ROS was shut down while waiting RM to load and initialize hardware.");
      this->dataPtr->RemoveControllerManager();
      return;
    }
    const auto waited = std::chrono::steady_clock::now() - wait_start;
    if (hardware_init_timeout > 0.0 && waited >= wait_timeout) {
      RCLCPP_ERROR(
        this->dataPtr->node_->get_logger(),
        "Timed out after %.1f s waiting RM to load and initialize hardware. "
        "Is robot_description being published? Failed to initialize.",
        hardware_init_timeout);
      this->dataPtr->RemoveControllerManager();
      return;
    }
    RCLCPP_WARN(
      this->dataPtr->node_->get_logger(),
      "Waiting RM to load and initialize hardware...");
    auto sleep_time = std::chrono::duration<double>(std::chrono::milliseconds(2000));
    if (hardware_init_timeout > 0.0) {
      sleep_time = std::min(
        sleep_time,
        std::chrono::duration_cast<std::chrono::duration<double>>(wait_timeout - waited));
    }
    std::this_thread::sleep_for(sleep_time);
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
    if (this->dataPtr->control_period_ < gazebo_period) {
      RCLCPP_ERROR_STREAM(
        this->dataPtr->node_->get_logger(),
        "Desired controller update period (" << this->dataPtr->control_period_.seconds() <<
          " s) is faster than the gazebo simulation period (" <<
          gazebo_period.seconds() << " s).");
    } else if (this->dataPtr->control_period_ > gazebo_period) {
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

  if (sim_period >= this->dataPtr->control_period_) {
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
