// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <gtest/gtest.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ForceTorque.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Sensor.hh>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_component_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include "gz_ros2_control/gz_system.hpp"

namespace
{
constexpr unsigned int kUpdateRate = 100;

hardware_interface::InterfaceInfo make_interface(
  const std::string & name, const std::string & initial_value = "")
{
  hardware_interface::InterfaceInfo info;
  info.name = name;
  info.initial_value = initial_value;
  return info;
}

hardware_interface::ComponentInfo make_joint(
  const std::string & name, const std::vector<std::string> & command_interfaces)
{
  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.type = "joint";
  for (const auto & command_interface : command_interfaces) {
    joint.command_interfaces.push_back(make_interface(command_interface));
  }
  joint.state_interfaces = {
    make_interface(hardware_interface::HW_IF_POSITION),
    make_interface(hardware_interface::HW_IF_VELOCITY),
    make_interface(hardware_interface::HW_IF_EFFORT)};
  return joint;
}

hardware_interface::ComponentInfo make_sensor(
  const std::string & name, const std::vector<std::string> & state_interfaces)
{
  hardware_interface::ComponentInfo sensor;
  sensor.name = name;
  sensor.type = "sensor";
  for (const auto & state_interface : state_interfaces) {
    sensor.state_interfaces.push_back(make_interface(state_interface));
  }
  return sensor;
}

const std::vector<std::string> kImuInterfaces = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

const std::vector<std::string> kForceTorqueInterfaces = {
  "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};

template<typename HandleT>
std::shared_ptr<HandleT> find_interface(
  const std::vector<std::shared_ptr<HandleT>> & interfaces, const std::string & name)
{
  for (const auto & interface : interfaces) {
    if (interface->get_name() == name) {
      return interface;
    }
  }
  return nullptr;
}
}  // namespace

class TestGazeboSimSystem : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node_ = rclcpp::Node::make_shared("test_gz_system");
    node_->declare_parameter("hold_joints", true);
    node_->declare_parameter("position_proportional_gain", 0.1);
    info_.name = "test_system";
    info_.type = "system";
    info_.hardware_plugin_name = "gz_ros2_control/GazeboSimSystem";
  }

  /// \brief Create a single-axis revolute joint in the ECM and make it available to the system
  gz::sim::Entity add_sim_joint(const std::string & name)
  {
    const auto entity = ecm_.CreateEntity();
    ecm_.CreateComponent(entity, gz::sim::components::Joint());
    ecm_.CreateComponent(entity, gz::sim::components::Name(name));
    ecm_.CreateComponent(entity, gz::sim::components::JointType(sdf::JointType::REVOLUTE));
    sdf::JointAxis axis;
    EXPECT_TRUE(axis.SetXyz(gz::math::Vector3d::UnitZ).empty());
    ecm_.CreateComponent(entity, gz::sim::components::JointAxis(axis));
    sim_joints_[name] = entity;
    return entity;
  }

  /// \brief Run initSim and init, the same sequence GZResourceManager uses
  bool init_system()
  {
    if (!system_.initSim(node_, sim_joints_, info_, ecm_, kUpdateRate)) {
      return false;
    }
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info_;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    params.logger = node_->get_logger();
    if (system_.init(params) != gz_ros2_control::CallbackReturn::SUCCESS) {
      return false;
    }
    state_interfaces_.clear();
    for (const auto & state_interface : system_.on_export_state_interfaces()) {
      state_interfaces_.push_back(std::const_pointer_cast<hardware_interface::StateInterface>(
          state_interface));
    }
    command_interfaces_ = system_.on_export_command_interfaces();
    return true;
  }

  /// \brief Create a sensor in the ECM, SensorComponentT is the sensor type component
  template<typename SensorComponentT>
  gz::sim::Entity add_sim_sensor(const std::string & name, const std::string & topic = "")
  {
    const auto entity = ecm_.CreateEntity();
    ecm_.CreateComponent(entity, SensorComponentT(sdf::Sensor()));
    ecm_.CreateComponent(entity, gz::sim::components::Name(name));
    if (!topic.empty()) {
      ecm_.CreateComponent(entity, gz::sim::components::SensorTopic(topic));
    }
    return entity;
  }

  /// \brief Publish msg on the Gazebo topic until the state interface has the expected value
  template<typename MsgT>
  bool publish_until(
    const std::string & topic, const MsgT & msg, const std::string & state, double expected)
  {
    gz::transport::Node node;
    auto publisher = node.Advertise<MsgT>(topic);
    const auto state_interface = find_interface(state_interfaces_, state);
    if (!state_interface) {
      return false;
    }
    // Discovery is asynchronous, the first messages may be lost
    for (int i = 0; i < 100; ++i) {
      publisher.Publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      if (state_interface->get_optional<double>().value() == expected) {
        return true;
      }
    }
    return false;
  }

  /// \brief Set the joint state as the Gazebo physics system would after a step
  void set_sim_state(
    gz::sim::Entity joint, double position, double velocity, double torque_z)
  {
    ecm_.SetComponentData<gz::sim::components::JointPosition>(joint, {position});
    ecm_.SetComponentData<gz::sim::components::JointVelocity>(joint, {velocity});
    gz::msgs::Wrench wrench;
    wrench.mutable_torque()->set_z(torque_z);
    ecm_.SetComponentData<gz::sim::components::JointTransmittedWrench>(joint, wrench);
  }

  double state_value(const std::string & name)
  {
    const auto state_interface = find_interface(state_interfaces_, name);
    EXPECT_NE(state_interface, nullptr) << name;
    return state_interface ? state_interface->get_optional<double>().value() : 0.0;
  }

  void set_command(const std::string & name, double value)
  {
    const auto command_interface = find_interface(command_interfaces_, name);
    ASSERT_NE(command_interface, nullptr) << name;
    ASSERT_TRUE(command_interface->set_value(value, true));
  }

  std::vector<double> velocity_cmd(gz::sim::Entity joint)
  {
    const auto * component = ecm_.Component<gz::sim::components::JointVelocityCmd>(joint);
    return component ? component->Data() : std::vector<double>{};
  }

  void read()
  {
    ASSERT_EQ(
      system_.read(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 10000000)),
      hardware_interface::return_type::OK);
  }

  void write()
  {
    ASSERT_EQ(
      system_.write(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 10000000)),
      hardware_interface::return_type::OK);
  }

  rclcpp::Node::SharedPtr node_;
  gz::sim::EntityComponentManager ecm_;
  std::map<std::string, gz::sim::Entity> sim_joints_;
  hardware_interface::HardwareInfo info_;
  gz_ros2_control::GazeboSimSystem system_;
  std::vector<hardware_interface::StateInterface::SharedPtr> state_interfaces_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces_;
};

TEST_F(TestGazeboSimSystem, InitSimFailsWithoutJoints)
{
  EXPECT_FALSE(system_.initSim(node_, sim_joints_, info_, ecm_, kUpdateRate));
}

TEST_F(TestGazeboSimSystem, ExportsInterfacesFromHardwareInfo)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"position", "velocity", "effort"})};
  ASSERT_TRUE(init_system());

  EXPECT_EQ(state_interfaces_.size(), 3u);
  EXPECT_EQ(command_interfaces_.size(), 3u);
  EXPECT_NE(find_interface(state_interfaces_, "joint1/position"), nullptr);
  EXPECT_NE(find_interface(command_interfaces_, "joint1/effort"), nullptr);

  // initSim creates the components the physics system fills in
  const auto joint = sim_joints_["joint1"];
  EXPECT_NE(ecm_.Component<gz::sim::components::JointPosition>(joint), nullptr);
  EXPECT_NE(ecm_.Component<gz::sim::components::JointVelocity>(joint), nullptr);
  EXPECT_NE(ecm_.Component<gz::sim::components::JointTransmittedWrench>(joint), nullptr);
}

TEST_F(TestGazeboSimSystem, InitialValuesAreApplied)
{
  const auto joint = add_sim_joint("joint1");
  auto joint_info = make_joint("joint1", {"position"});
  joint_info.state_interfaces[0].initial_value = "0.5";
  info_.joints = {joint_info};
  ASSERT_TRUE(init_system());

  EXPECT_DOUBLE_EQ(state_value("joint1/position"), 0.5);
  const auto command = find_interface(command_interfaces_, "joint1/position");
  ASSERT_NE(command, nullptr);
  EXPECT_DOUBLE_EQ(command->get_optional<double>().value(), 0.5);

  const auto * reset = ecm_.Component<gz::sim::components::JointPositionReset>(joint);
  ASSERT_NE(reset, nullptr);
  ASSERT_EQ(reset->Data().size(), 1u);
  EXPECT_DOUBLE_EQ(reset->Data()[0], 0.5);
}

TEST_F(TestGazeboSimSystem, InvalidInitialValueThrows)
{
  // GZResourceManager catches this and reports a failed initialization
  add_sim_joint("joint1");
  auto joint_info = make_joint("joint1", {"position"});
  joint_info.state_interfaces[0].initial_value = "not_a_number";
  info_.joints = {joint_info};
  EXPECT_THROW(
    system_.initSim(node_, sim_joints_, info_, ecm_, kUpdateRate), std::invalid_argument);
}

TEST_F(TestGazeboSimSystem, ReadCopiesSimulationState)
{
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"velocity"})};
  ASSERT_TRUE(init_system());

  set_sim_state(joint, 1.2, 0.3, 2.0);
  read();

  EXPECT_DOUBLE_EQ(state_value("joint1/position"), 1.2);
  EXPECT_DOUBLE_EQ(state_value("joint1/velocity"), 0.3);
  // Effort is the transmitted torque projected on the joint axis
  EXPECT_DOUBLE_EQ(state_value("joint1/effort"), 2.0);
}

TEST_F(TestGazeboSimSystem, ReadSkipsJointsWithoutSimulationData)
{
  add_sim_joint("joint1");
  auto joint_info = make_joint("joint1", {"velocity"});
  joint_info.state_interfaces[0].initial_value = "0.25";
  info_.joints = {joint_info};
  ASSERT_TRUE(init_system());

  // Components exist but are empty until physics runs, the state must not change
  read();
  EXPECT_DOUBLE_EQ(state_value("joint1/position"), 0.25);
}

TEST_F(TestGazeboSimSystem, VelocityCommandIsForwarded)
{
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"velocity"})};
  ASSERT_TRUE(init_system());

  ASSERT_EQ(
    system_.perform_command_mode_switch({"joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  set_command("joint1/velocity", 0.7);
  write();

  EXPECT_EQ(velocity_cmd(joint), std::vector<double>{0.7});
}

TEST_F(TestGazeboSimSystem, PositionCommandUsesProportionalGain)
{
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"position"})};
  ASSERT_TRUE(init_system());

  set_sim_state(joint, 0.0, 0.0, 0.0);
  read();
  ASSERT_EQ(
    system_.perform_command_mode_switch({"joint1/position"}, {}),
    hardware_interface::return_type::OK);
  set_command("joint1/position", 1.0);
  write();

  // gain * position error * update rate
  const auto cmd = velocity_cmd(joint);
  ASSERT_EQ(cmd.size(), 1u);
  EXPECT_DOUBLE_EQ(cmd[0], 0.1 * 1.0 * kUpdateRate);
}

TEST_F(TestGazeboSimSystem, EffortCommandIsForwarded)
{
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"effort"})};
  ASSERT_TRUE(init_system());

  ASSERT_EQ(
    system_.perform_command_mode_switch({"joint1/effort"}, {}),
    hardware_interface::return_type::OK);
  set_command("joint1/effort", 3.5);
  // The first write creates the component, the second one sets the command
  write();
  write();

  const auto * force = ecm_.Component<gz::sim::components::JointForceCmd>(joint);
  ASSERT_NE(force, nullptr);
  EXPECT_EQ(force->Data(), std::vector<double>{3.5});
}

TEST_F(TestGazeboSimSystem, HoldsActuatedJointsWithoutActiveCommand)
{
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"velocity"})};
  ASSERT_TRUE(init_system());

  ASSERT_EQ(
    system_.perform_command_mode_switch({"joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  set_command("joint1/velocity", 0.7);
  write();
  ASSERT_EQ(
    system_.perform_command_mode_switch({}, {"joint1/velocity"}),
    hardware_interface::return_type::OK);
  write();

  EXPECT_EQ(velocity_cmd(joint), std::vector<double>{0.0});
}

TEST_F(TestGazeboSimSystem, DoesNotHoldJointsWhenDisabled)
{
  node_->set_parameter(rclcpp::Parameter("hold_joints", false));
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"velocity"})};
  ASSERT_TRUE(init_system());

  write();
  EXPECT_EQ(ecm_.Component<gz::sim::components::JointVelocityCmd>(joint), nullptr);
}

TEST_F(TestGazeboSimSystem, DoesNotHoldPassiveJoints)
{
  const auto joint = add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};
  ASSERT_TRUE(init_system());

  write();
  EXPECT_EQ(ecm_.Component<gz::sim::components::JointVelocityCmd>(joint), nullptr);
}

TEST_F(TestGazeboSimSystem, JointMissingInSimulationIsSkipped)
{
  add_sim_joint("joint1");
  info_.joints = {
    make_joint("joint1", {"velocity"}),
    make_joint("not_in_gazebo", {"velocity"})};
  ASSERT_TRUE(init_system());

  ASSERT_EQ(
    system_.perform_command_mode_switch({"not_in_gazebo/velocity"}, {}),
    hardware_interface::return_type::OK);
  read();
  write();
}

TEST_F(TestGazeboSimSystem, MimicJointFollowsMimickedJoint)
{
  const auto joint1 = add_sim_joint("joint1");
  const auto joint2 = add_sim_joint("joint2");
  info_.joints = {make_joint("joint1", {"position"}), make_joint("joint2", {})};
  hardware_interface::MimicJoint mimic;
  mimic.joint_index = 1;
  mimic.mimicked_joint_index = 0;
  mimic.multiplier = 2.0;
  mimic.offset = 0.1;
  info_.mimic_joints = {mimic};
  ASSERT_TRUE(init_system());

  // Before physics has run the position components are empty and must be skipped
  write();
  EXPECT_TRUE(velocity_cmd(joint2).empty());

  set_sim_state(joint1, 1.0, 0.0, 0.0);
  set_sim_state(joint2, 0.0, 0.0, 0.0);
  write();

  // -(mimic position - (mimicked position * multiplier + offset)) * update rate
  const auto cmd = velocity_cmd(joint2);
  ASSERT_EQ(cmd.size(), 1u);
  EXPECT_DOUBLE_EQ(cmd[0], (1.0 * 2.0 + 0.1) * kUpdateRate);
}

TEST_F(TestGazeboSimSystem, MimicJointMissingInSimulationIsSkipped)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {"position"}), make_joint("not_in_gazebo", {})};
  hardware_interface::MimicJoint mimic;
  mimic.joint_index = 1;
  mimic.mimicked_joint_index = 0;
  info_.mimic_joints = {mimic};
  ASSERT_TRUE(init_system());

  set_sim_state(sim_joints_["joint1"], 1.0, 0.0, 0.0);
  write();
}

TEST_F(TestGazeboSimSystem, UnknownSensorInterfaceIsSkipped)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};

  const auto imu = ecm_.CreateEntity();
  ecm_.CreateComponent(imu, gz::sim::components::Imu(sdf::Sensor()));
  ecm_.CreateComponent(imu, gz::sim::components::Name("imu_sensor"));
  hardware_interface::ComponentInfo sensor;
  sensor.name = "imu_sensor";
  sensor.type = "sensor";
  sensor.state_interfaces = {make_interface("orientation.x"), make_interface("orientation.X")};
  info_.sensors = {sensor};

  ASSERT_TRUE(init_system());
  EXPECT_NE(find_interface(state_interfaces_, "imu_sensor/orientation.x"), nullptr);
  EXPECT_EQ(find_interface(state_interfaces_, "imu_sensor/orientation.X"), nullptr);
}

TEST_F(TestGazeboSimSystem, ImuMessagesUpdateStateInterfaces)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};
  const std::string topic = "/test_gz_system/imu";
  add_sim_sensor<gz::sim::components::Imu>("imu_sensor", topic);
  info_.sensors = {make_sensor("imu_sensor", kImuInterfaces)};
  ASSERT_TRUE(init_system());
  for (const auto & name : kImuInterfaces) {
    EXPECT_NE(find_interface(state_interfaces_, "imu_sensor/" + name), nullptr) << name;
  }

  // read() subscribes to the sensor topic
  read();
  gz::msgs::IMU msg;
  msg.mutable_orientation()->set_x(0.1);
  msg.mutable_orientation()->set_y(0.2);
  msg.mutable_orientation()->set_z(0.3);
  msg.mutable_orientation()->set_w(0.4);
  msg.mutable_angular_velocity()->set_x(1.1);
  msg.mutable_angular_velocity()->set_y(1.2);
  msg.mutable_angular_velocity()->set_z(1.3);
  msg.mutable_linear_acceleration()->set_x(2.1);
  msg.mutable_linear_acceleration()->set_y(2.2);
  msg.mutable_linear_acceleration()->set_z(9.8);
  ASSERT_TRUE(publish_until(topic, msg, "imu_sensor/linear_acceleration.z", 9.8));

  EXPECT_DOUBLE_EQ(state_value("imu_sensor/orientation.x"), 0.1);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/orientation.y"), 0.2);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/orientation.z"), 0.3);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/orientation.w"), 0.4);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/angular_velocity.x"), 1.1);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/angular_velocity.y"), 1.2);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/angular_velocity.z"), 1.3);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/linear_acceleration.x"), 2.1);
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/linear_acceleration.y"), 2.2);
}

TEST_F(TestGazeboSimSystem, ForceTorqueMessagesUpdateStateInterfaces)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};
  const std::string topic = "/test_gz_system/force_torque";
  // The sensors system sets the topic after the plugin is initialized
  const auto ft = add_sim_sensor<gz::sim::components::ForceTorque>("ft_sensor");
  info_.sensors = {make_sensor("ft_sensor", kForceTorqueInterfaces)};
  ASSERT_TRUE(init_system());
  for (const auto & name : kForceTorqueInterfaces) {
    EXPECT_NE(find_interface(state_interfaces_, "ft_sensor/" + name), nullptr) << name;
  }

  // Without topic there is nothing to subscribe to yet
  read();
  ecm_.CreateComponent(ft, gz::sim::components::SensorTopic(topic));
  read();

  gz::msgs::Wrench msg;
  msg.mutable_force()->set_x(1.0);
  msg.mutable_force()->set_y(2.0);
  msg.mutable_force()->set_z(3.0);
  msg.mutable_torque()->set_x(4.0);
  msg.mutable_torque()->set_y(5.0);
  msg.mutable_torque()->set_z(6.0);
  ASSERT_TRUE(publish_until(topic, msg, "ft_sensor/torque.z", 6.0));

  EXPECT_DOUBLE_EQ(state_value("ft_sensor/force.x"), 1.0);
  EXPECT_DOUBLE_EQ(state_value("ft_sensor/force.y"), 2.0);
  EXPECT_DOUBLE_EQ(state_value("ft_sensor/force.z"), 3.0);
  EXPECT_DOUBLE_EQ(state_value("ft_sensor/torque.x"), 4.0);
  EXPECT_DOUBLE_EQ(state_value("ft_sensor/torque.y"), 5.0);
}

TEST_F(TestGazeboSimSystem, OnlyListedSensorInterfacesAreUpdated)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};
  const std::string topic = "/test_gz_system/imu_partial";
  add_sim_sensor<gz::sim::components::Imu>("imu_sensor", topic);
  info_.sensors = {make_sensor("imu_sensor", {"angular_velocity.z", "orientation.w"})};
  ASSERT_TRUE(init_system());

  // 3 joint state interfaces and the 2 listed sensor interfaces
  EXPECT_EQ(state_interfaces_.size(), 5u);

  read();
  gz::msgs::IMU msg;
  msg.mutable_orientation()->set_w(0.7);
  msg.mutable_angular_velocity()->set_z(-0.5);
  ASSERT_TRUE(publish_until(topic, msg, "imu_sensor/angular_velocity.z", -0.5));
  EXPECT_DOUBLE_EQ(state_value("imu_sensor/orientation.w"), 0.7);
}

TEST_F(TestGazeboSimSystem, SensorNotInHardwareInfoIsSkipped)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};
  add_sim_sensor<gz::sim::components::Imu>("other_imu", "/test_gz_system/other_imu");
  add_sim_sensor<gz::sim::components::ForceTorque>("other_ft", "/test_gz_system/other_ft");
  ASSERT_TRUE(init_system());

  EXPECT_EQ(state_interfaces_.size(), 3u);
  read();
}

TEST_F(TestGazeboSimSystem, SensorMissingInSimulationExportsNoInterfaces)
{
  add_sim_joint("joint1");
  info_.joints = {make_joint("joint1", {})};
  info_.sensors = {
    make_sensor("imu_sensor", kImuInterfaces),
    make_sensor("ft_sensor", kForceTorqueInterfaces)};
  ASSERT_TRUE(init_system());

  EXPECT_EQ(state_interfaces_.size(), 3u);
  read();
}

TEST(SafeEnum, FlagsCanBeCombinedAndCleared)
{
  using gz_ros2_control::GazeboSimSystemInterface;
  GazeboSimSystemInterface::ControlMethod method;
  EXPECT_FALSE(static_cast<bool>(method));

  method |= GazeboSimSystemInterface::POSITION;
  method |= GazeboSimSystemInterface::VELOCITY;
  EXPECT_TRUE(static_cast<bool>(method & GazeboSimSystemInterface::POSITION));
  EXPECT_TRUE(static_cast<bool>(method & GazeboSimSystemInterface::VELOCITY));
  EXPECT_FALSE(static_cast<bool>(method & GazeboSimSystemInterface::EFFORT));

  method &= static_cast<GazeboSimSystemInterface::ControlMethod_>(
    GazeboSimSystemInterface::VELOCITY | GazeboSimSystemInterface::EFFORT);
  EXPECT_FALSE(static_cast<bool>(method & GazeboSimSystemInterface::POSITION));

  // The const operators can be used on a const object
  const GazeboSimSystemInterface::ControlMethod const_method(method);
  EXPECT_TRUE(static_cast<bool>(const_method & GazeboSimSystemInterface::VELOCITY));
}
