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

#include <unistd.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ignition/gazebo/components/Joint.hh>
#include <ignition/gazebo/components/JointType.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/plugin/Register.hh>

#include <controller_manager/controller_manager.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>

#include "ign_ros2_control/ign_ros2_control_plugin.hpp"
#include "ign_ros2_control/ign_system.hpp"

namespace ign_ros2_control
{
//////////////////////////////////////////////////
class IgnitionROS2ControlPluginPrivate
{
public:
  /// \brief Get the URDF XML from the parameter server
  std::string getURDF() const;

  /// \brief Get a list of enabled, unique, 1-axis joints of the model. If no
  /// joint names are specified in the plugin configuration, all valid 1-axis
  /// joints are returned
  /// \param[in] _entity Entity of the model that the plugin is being
  /// configured for
  /// \param[in] _ecm Ignition Entity Component Manager
  /// \return List of entities containing all enabled joints
  std::map<std::string, ignition::gazebo::Entity> GetEnabledJoints(
    const ignition::gazebo::Entity & _entity,
    ignition::gazebo::EntityComponentManager & _ecm) const;

  /// \brief Entity ID for sensor within Gazebo.
  ignition::gazebo::Entity entity_;

  /// \brief Node Handles
  std::shared_ptr<rclcpp::Node> node_{nullptr};

  /// \brief Thread where the executor will spin
  std::thread thread_executor_spin_;

  /// \brief Context for the executor
  rclcpp::Context::SharedPtr context_;

  /// \brief Executor to spin the controller
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

  /// \brief Timing
  rclcpp::Duration control_period_ = rclcpp::Duration(1, 0);

  /// \brief Interface loader
  std::shared_ptr<pluginlib::ClassLoader<
      ign_ros2_control::IgnitionSystemInterface>>
  robot_hw_sim_loader_{nullptr};

  /// \brief Controller manager
  std::shared_ptr<controller_manager::ControllerManager>
  controller_manager_{nullptr};

  /// \brief String with the robot description param_name
  std::string robot_description_ = "robot_description";

  /// \brief String with the name of the node that contains the robot_description
  std::string robot_description_node_ = "robot_state_publisher";

  /// \brief Last time the update method was called
  rclcpp::Time last_update_sim_time_ros_ =
    rclcpp::Time((int64_t)0, RCL_ROS_TIME);

  /// \brief ECM pointer
  ignition::gazebo::EntityComponentManager * ecm{nullptr};

  /// \brief controller update rate
  int update_rate;
};

//////////////////////////////////////////////////
std::map<std::string, ignition::gazebo::Entity>
IgnitionROS2ControlPluginPrivate::GetEnabledJoints(
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
          RCLCPP_INFO(
            node_->get_logger(),
            "[ign_ros2_control] Fixed joint [%s] (Entity=%lu)] is skipped",
            jointName.c_str(), jointEntity);
          continue;
        }
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::BALL:
      case sdf::JointType::UNIVERSAL:
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "[ign_ros2_control] Joint [%s] (Entity=%lu)] is of unsupported type."
            " Only joints with a single axis are supported.",
            jointName.c_str(), jointEntity);
          continue;
        }
      default:
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "[ign_ros2_control] Joint [%s] (Entity=%lu)] is of unknown type",
            jointName.c_str(), jointEntity);
          continue;
        }
    }
    output[jointName] = jointEntity;
  }

  return output;
}

//////////////////////////////////////////////////
std::string IgnitionROS2ControlPluginPrivate::getURDF() const
{
  std::string urdf_string;

  using namespace std::chrono_literals;
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, robot_description_node_);
  while (!parameters_client->wait_for_service(0.5s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        node_->get_logger(), "Interrupted while waiting for %s service. Exiting.",
        robot_description_node_.c_str());
      return 0;
    }
    RCLCPP_ERROR(
      node_->get_logger(), "%s service not available, waiting again...",
      robot_description_node_.c_str());
  }

  RCLCPP_INFO(
    node_->get_logger(), "connected to service!! %s asking for %s",
    robot_description_node_.c_str(),
    this->robot_description_.c_str());

  // search and wait for robot_description on param server
  while (urdf_string.empty()) {
    RCLCPP_DEBUG(
      node_->get_logger(), "param_name %s",
      this->robot_description_.c_str());

    try {
      auto f = parameters_client->get_parameters({this->robot_description_});
      f.wait();
      std::vector<rclcpp::Parameter> values = f.get();
      urdf_string = values[0].as_string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "%s", e.what());
    }

    if (!urdf_string.empty()) {
      break;
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "ign_ros2_control plugin is waiting for model"
        " URDF in parameter [%s] on the ROS param server.",
        this->robot_description_.c_str());
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100000));
  }
  RCLCPP_INFO(node_->get_logger(), "Received URDF from param server");

  return urdf_string;
}

//////////////////////////////////////////////////
IgnitionROS2ControlPlugin::IgnitionROS2ControlPlugin()
: dataPtr(std::make_unique<IgnitionROS2ControlPluginPrivate>())
{
}

//////////////////////////////////////////////////
IgnitionROS2ControlPlugin::~IgnitionROS2ControlPlugin()
{
  // Stop controller manager thread
  if (!this->dataPtr->controller_manager_) {
    return;
  }
  this->dataPtr->executor_->remove_node(this->dataPtr->controller_manager_);
  this->dataPtr->executor_->cancel();
  this->dataPtr->thread_executor_spin_.join();
}

//////////////////////////////////////////////////
void IgnitionROS2ControlPlugin::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager &)
{
  rclcpp::Logger logger = rclcpp::get_logger("GazeboSimROS2ControlPlugin");
  // Make sure the controller is attached to a valid model
  const auto model = ignition::gazebo::Model(_entity);
  if (!model.Valid(_ecm)) {
    RCLCPP_ERROR(
      logger,
      "[Ignition ROS 2 Control] Failed to initialize because [%s] (Entity=%lu)] is not a model."
      "Please make sure that Ignition ROS 2 Control is attached to a valid model.",
      model.Name(_ecm).c_str(), _entity);
    return;
  }

  // Get params from SDF
  std::string paramFileName = _sdf->Get<std::string>("parameters");

  if (paramFileName.empty()) {
    RCLCPP_ERROR(
      logger,
      "Ignition ros2 control found an empty parameters file. Failed to initialize.");
    return;
  }

  // Get params from SDF
  std::string robot_param_node = _sdf->Get<std::string>("robot_param_node");
  if (!robot_param_node.empty()) {
    this->dataPtr->robot_description_node_ = robot_param_node;
  }
  RCLCPP_INFO(
    logger,
    "robot_param_node is %s", this->dataPtr->robot_description_node_.c_str());

  std::string robot_description = _sdf->Get<std::string>("robot_param");
  if (!robot_description.empty()) {
    this->dataPtr->robot_description_ = robot_description;
  }
  RCLCPP_INFO(
    logger,
    "robot_param_node is %s", this->dataPtr->robot_description_.c_str());

  std::vector<std::string> arguments = {"--ros-args"};

  auto sdfPtr = const_cast<sdf::Element *>(_sdf.get());

  std::vector<std::string> parameter_file_names;

  sdf::ElementPtr argument_sdf = sdfPtr->GetElement("parameters");
  while (argument_sdf) {
    std::string argument = argument_sdf->Get<std::string>();
    arguments.push_back(RCL_PARAM_FILE_FLAG);
    arguments.push_back(argument);
    parameter_file_names.push_back(argument);
    argument_sdf = argument_sdf->GetNextElement("parameters");
  }

  // Get controller manager node name
  std::string controllerManagerNodeName{"controller_manager"};

  if (sdfPtr->HasElement("controller_manager_name")) {
    controllerManagerNodeName = sdfPtr->GetElement("controller_manager_name")->Get<std::string>();
  }

  std::string ns = "/";
  if (sdfPtr->HasElement("ros")) {
    sdf::ElementPtr sdfRos = sdfPtr->GetElement("ros");

    // Set namespace if tag is present
    if (sdfRos->HasElement("namespace")) {
      ns = sdfRos->GetElement("namespace")->Get<std::string>();
      // prevent exception: namespace must be absolute, it must lead with a '/'
      if (ns.empty() || ns[0] != '/') {
        ns = '/' + ns;
      }
      if (ns.length() > 1) {
        this->dataPtr->robot_description_node_ = ns + "/" + this->dataPtr->robot_description_node_;
      }
    }

    // Get list of remapping rules from SDF
    if (sdfRos->HasElement("remapping")) {
      sdf::ElementPtr argument_sdf = sdfRos->GetElement("remapping");

      arguments.push_back(RCL_ROS_ARGS_FLAG);
      while (argument_sdf) {
        std::string argument = argument_sdf->Get<std::string>();
        arguments.push_back(RCL_REMAP_FLAG);
        arguments.push_back(argument);
        argument_sdf = argument_sdf->GetNextElement("remapping");
      }
    }
  }

  std::vector<const char *> argv;
  // for (const auto & arg : arguments) {
  //   RCLCPP_INFO(
  //     logger,
  //     arg.data()
  //   );
  //   argv.push_back(reinterpret_cast<const char *>(arg.data()));
  // }

  // Create a default context, if not already
  if (!rclcpp::ok()) {
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }

  // Create individual context
  std::string msg = ns + " [gz_ros2_control]: rclcpp:init called";
  RCLCPP_INFO(
    logger,
    msg.data()
  );

  // this->dataPtr->context_ = std::make_shared<rclcpp::Context>();
  // this->dataPtr->context_->init(static_cast<int>(argv.size()), argv.data());

  this->dataPtr->context_ = rclcpp::contexts::get_global_default_context();

  std::string node_name = "gz_ros2_control";

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = this->dataPtr->context_;

  rclcpp::NodeOptions node_options;
  node_options.context(this->dataPtr->context_);
  node_options.arguments(arguments);

  this->dataPtr->node_ = rclcpp::Node::make_shared(node_name, ns, node_options);
  this->dataPtr->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(executor_options);
  this->dataPtr->executor_->add_node(this->dataPtr->node_);
  auto spin = [this]()
    {
      this->dataPtr->executor_->spin();
    };
  this->dataPtr->thread_executor_spin_ = std::thread(spin);

  RCLCPP_DEBUG_STREAM(
    this->dataPtr->node_->get_logger(), "[Ignition ROS 2 Control] Setting up controller for [" <<
      model.Name(_ecm) << "] (Entity=" << _entity << ")].");

  // Get list of enabled joints
  auto enabledJoints = this->dataPtr->GetEnabledJoints(
    _entity,
    _ecm);

  if (enabledJoints.size() == 0) {
    RCLCPP_DEBUG_STREAM(
      this->dataPtr->node_->get_logger(),
      "[Ignition ROS 2 Control] There are no available Joints.");
    return;
  }

  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  // This call will block if ROS is not properly initialized.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try {
    urdf_string = this->dataPtr->getURDF();
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  } catch (const std::runtime_error & ex) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node_->get_logger(),
      "Error parsing URDF in ign_ros2_control plugin, plugin not active : " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager_ =
    std::make_unique<hardware_interface::ResourceManager>();

  try {
    resource_manager_->load_urdf(urdf_string, false, false);
  } catch (...) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Error initializing URDF to resource manager!");
  }
  try {
    this->dataPtr->robot_hw_sim_loader_.reset(
      new pluginlib::ClassLoader<ign_ros2_control::IgnitionSystemInterface>(
        "ign_ros2_control",
        "ign_ros2_control::IgnitionSystemInterface"));
  } catch (pluginlib::LibraryLoadException & ex) {
    RCLCPP_ERROR(
      this->dataPtr->node_->get_logger(), "Failed to create robot simulation interface loader: %s ",
      ex.what());
    return;
  }

  for (unsigned int i = 0; i < control_hardware_info.size(); ++i) {
    std::string robot_hw_sim_type_str_ = control_hardware_info[i].hardware_class_type;
    std::unique_ptr<ign_ros2_control::IgnitionSystemInterface> ignitionSystem;
    RCLCPP_DEBUG(
      this->dataPtr->node_->get_logger(), "Load hardware interface %s ...",
      robot_hw_sim_type_str_.c_str());

    try {
      ignitionSystem = std::unique_ptr<ign_ros2_control::IgnitionSystemInterface>(
        this->dataPtr->robot_hw_sim_loader_->createUnmanagedInstance(robot_hw_sim_type_str_));
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        this->dataPtr->node_->get_logger(),
        "The plugin failed to load for some reason. Error: %s\n",
        ex.what());
      continue;
    }
    if (!ignitionSystem->initSim(
        this->dataPtr->node_,
        enabledJoints,
        control_hardware_info[i],
        _ecm,
        this->dataPtr->update_rate))
    {
      RCLCPP_FATAL(
        this->dataPtr->node_->get_logger(), "Could not initialize robot simulation interface");
      return;
    }
    RCLCPP_DEBUG(
      this->dataPtr->node_->get_logger(), "Initialized robot simulation interface %s!",
      robot_hw_sim_type_str_.c_str());

    resource_manager_->import_component(std::move(ignitionSystem), control_hardware_info[i]);

    rclcpp_lifecycle::State state(
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE,
      hardware_interface::lifecycle_state_names::ACTIVE);
    resource_manager_->set_component_state(control_hardware_info[i].name, state);
  }

  rclcpp::NodeOptions cm_node_options = controller_manager::get_cm_node_options();
  cm_node_options.context(this->dataPtr->context_);
  cm_node_options.arguments(arguments);

  // Create the controller manager
  RCLCPP_INFO(this->dataPtr->node_->get_logger(), "Loading controller_manager");
  this->dataPtr->controller_manager_.reset(
    new controller_manager::ControllerManager(
      std::move(resource_manager_),
      this->dataPtr->executor_,
      controllerManagerNodeName,
      this->dataPtr->node_->get_namespace(),
      cm_node_options
    )
      
    );
  this->dataPtr->executor_->add_node(this->dataPtr->controller_manager_);

  auto parameters_client =  std::make_shared<rclcpp::AsyncParametersClient>(
    this->dataPtr->controller_manager_, controllerManagerNodeName
  );

  for (const std::string& parameter_file_name : parameter_file_names)
  {
    RCLCPP_INFO(
      this->dataPtr->controller_manager_->get_logger(),
      "Trying to load file: "
    );

    RCLCPP_INFO(
      this->dataPtr->controller_manager_->get_logger(),
      parameter_file_name.data()
    );
    auto parameter_map = rclcpp::parameter_map_from_yaml_file(parameter_file_name);


    // parameters_client->load_parameters(parameter_map);
    // parameters_client->load_parameters(parameter_file_name).wait();
  }


  if (!this->dataPtr->controller_manager_->has_parameter("update_rate")) {
    RCLCPP_ERROR_STREAM(
      this->dataPtr->node_->get_logger(),
      "controller manager doesn't have an update_rate parameter");
    return;
  }

  this->dataPtr->update_rate =
    this->dataPtr->controller_manager_->get_parameter("update_rate").as_int();
  this->dataPtr->control_period_ = rclcpp::Duration(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / static_cast<double>(this->dataPtr->update_rate))));

  // Force setting of use_sim_time parameter
  this->dataPtr->controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  this->dataPtr->entity_ = _entity;
}

//////////////////////////////////////////////////
void IgnitionROS2ControlPlugin::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
  if (!this->dataPtr->controller_manager_) {
    return;
  }
  static bool warned{false};
  if (!warned) {
    rclcpp::Duration gazebo_period(_info.dt);

    // Check the period against the simulation period
    if (this->dataPtr->control_period_ < _info.dt) {
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
    warned = true;
  }

  rclcpp::Time sim_time_ros(std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime).count(), RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - this->dataPtr->last_update_sim_time_ros_;
  // Always set commands on joints, otherwise at low control frequencies the joints tremble
  // as they are updated at a fraction of gazebo sim time
  this->dataPtr->controller_manager_->write(sim_time_ros, sim_period);
}

//////////////////////////////////////////////////
void IgnitionROS2ControlPlugin::PostUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  const ignition::gazebo::EntityComponentManager & /*_ecm*/)
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
    auto ign_controller_manager =
      std::dynamic_pointer_cast<ign_ros2_control::IgnitionSystemInterface>(
      this->dataPtr->controller_manager_);
    this->dataPtr->controller_manager_->read(sim_time_ros, sim_period);
    this->dataPtr->controller_manager_->update(sim_time_ros, sim_period);
  }
}
}  // namespace ign_ros2_control

IGNITION_ADD_PLUGIN(
  ign_ros2_control::IgnitionROS2ControlPlugin,
  ignition::gazebo::System,
  ign_ros2_control::IgnitionROS2ControlPlugin::ISystemConfigure,
  ign_ros2_control::IgnitionROS2ControlPlugin::ISystemPreUpdate,
  ign_ros2_control::IgnitionROS2ControlPlugin::ISystemPostUpdate)
