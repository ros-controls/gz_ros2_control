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

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/resource_manager_params.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

#include "gz_resource_manager.hpp"
#include "gz_ros2_control/gz_system_interface.hpp"

namespace
{
constexpr unsigned int kUpdateRate = 100;

/// \brief What the fake system does in initSim
enum class InitSimResult
{
  kSuccess,
  kFailure,
  kThrow,
};

/// \brief Calls seen by the fake systems, shared with the test
struct SystemCalls
{
  std::vector<std::string> loaded;
  std::vector<std::string> initialized;
  unsigned int update_rate{0};
  std::map<std::string, gz::sim::Entity> joints;
  gz::sim::EntityComponentManager * ecm{nullptr};
};

class FakeSystem : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  FakeSystem(SystemCalls & calls, InitSimResult result)
  : calls_(calls), result_(result) {}

  bool initSim(
    rclcpp::Node::SharedPtr &,
    std::map<std::string, gz::sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    gz::sim::EntityComponentManager & ecm,
    unsigned int update_rate) override
  {
    if (result_ == InitSimResult::kThrow) {
      throw std::runtime_error("initSim exception");
    }
    calls_.initialized.push_back(hardware_info.name);
    calls_.update_rate = update_rate;
    calls_.joints = joints;
    calls_.ecm = &ecm;
    return result_ == InitSimResult::kSuccess;
  }

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
  {
    return hardware_interface::return_type::OK;
  }

private:
  SystemCalls & calls_;
  InitSimResult result_;
};

/// \brief ros2_control tag of a system with one joint
std::string ros2_control_tag(
  const std::string & name, const std::string & plugin, const std::string & joint)
{
  return
    "<ros2_control name=\"" + name + "\" type=\"system\">"
    "  <hardware><plugin>" + plugin + "</plugin></hardware>"
    "  <joint name=\"" + joint + "\">"
    "    <command_interface name=\"position\"/>"
    "    <state_interface name=\"position\"/>"
    "  </joint>"
    "</ros2_control>";
}

std::string robot_description(const std::vector<std::string> & ros2_control_tags)
{
  std::string urdf =
    "<?xml version=\"1.0\"?><robot name=\"robot\">"
    "<link name=\"base\"/><link name=\"link1\"/><link name=\"link2\"/>"
    "<joint name=\"joint1\" type=\"continuous\">"
    "  <parent link=\"base\"/><child link=\"link1\"/>"
    "</joint>"
    "<joint name=\"joint2\" type=\"continuous\">"
    "  <parent link=\"link1\"/><child link=\"link2\"/>"
    "</joint>";
  for (const auto & tag : ros2_control_tags) {
    urdf += tag;
  }
  return urdf + "</robot>";
}
}  // namespace

class TestGZResourceManager : public ::testing::Test
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
    node_ = rclcpp::Node::make_shared("test_gz_resource_manager");
    sim_joints_ = {{"joint1", ecm_.CreateEntity()}, {"joint2", ecm_.CreateEntity()}};
  }

  /// \brief Resource manager whose systems are FakeSystems, plugins named "throw" fail to load,
  /// plugins named "null" load as nullptr, and the init_results give the initSim results by
  /// plugin name
  std::unique_ptr<gz_ros2_control::GZResourceManager> make_resource_manager(
    const std::map<std::string, InitSimResult> & init_results = {})
  {
    return std::make_unique<gz_ros2_control::GZResourceManager>(
      node_, ecm_, sim_joints_,
      [this, init_results](const std::string & plugin_name)
      -> std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface> {
        calls_.loaded.push_back(plugin_name);
        if (plugin_name == "throw") {
          throw std::runtime_error("plugin load exception");
        }
        if (plugin_name == "null") {
          return nullptr;
        }
        const auto it = init_results.find(plugin_name);
        return std::make_unique<FakeSystem>(
          calls_, it == init_results.end() ? InitSimResult::kSuccess : it->second);
      });
  }

  bool load(
    gz_ros2_control::GZResourceManager & resource_manager, const std::string & description)
  {
    hardware_interface::ResourceManagerParams params;
    params.robot_description = description;
    params.update_rate = kUpdateRate;
    params.clock = node_->get_clock();
    params.logger = node_->get_logger();
    return resource_manager.load_and_initialize_components(params);
  }

  rclcpp::Node::SharedPtr node_;
  gz::sim::EntityComponentManager ecm_;
  std::map<std::string, gz::sim::Entity> sim_joints_;
  SystemCalls calls_;
};

TEST_F(TestGZResourceManager, LoadsAndInitializesAllSystems)
{
  auto resource_manager = make_resource_manager();
  ASSERT_TRUE(
    load(
      *resource_manager, robot_description(
        {ros2_control_tag("system1", "fake1", "joint1"),
          ros2_control_tag("system2", "fake2", "joint2")})));

  EXPECT_TRUE(resource_manager->are_components_initialized());
  EXPECT_EQ(calls_.loaded, (std::vector<std::string>{"fake1", "fake2"}));
  EXPECT_EQ(calls_.initialized, (std::vector<std::string>{"system1", "system2"}));
  EXPECT_EQ(calls_.update_rate, kUpdateRate);
  EXPECT_EQ(calls_.joints, sim_joints_);
  EXPECT_EQ(calls_.ecm, &ecm_);

  const auto & status = resource_manager->get_components_status();
  EXPECT_EQ(status.size(), 2u);
  EXPECT_TRUE(status.count("system1"));
  EXPECT_TRUE(status.count("system2"));
  EXPECT_TRUE(resource_manager->state_interface_exists("joint1/position"));
  EXPECT_TRUE(resource_manager->command_interface_exists("joint2/position"));
}

TEST_F(TestGZResourceManager, InvalidRobotDescriptionFails)
{
  auto resource_manager = make_resource_manager();
  EXPECT_FALSE(load(*resource_manager, "<robot"));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_TRUE(calls_.loaded.empty());
}

TEST_F(TestGZResourceManager, EmptyRobotDescriptionFails)
{
  auto resource_manager = make_resource_manager();
  EXPECT_FALSE(load(*resource_manager, ""));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_TRUE(calls_.loaded.empty());
}

TEST_F(TestGZResourceManager, PluginLoadExceptionFails)
{
  auto resource_manager = make_resource_manager();
  EXPECT_FALSE(
    load(
      *resource_manager, robot_description({ros2_control_tag("system1", "throw", "joint1")})));
  EXPECT_EQ(calls_.loaded, (std::vector<std::string>{"throw"}));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_TRUE(resource_manager->get_components_status().empty());
}

TEST_F(TestGZResourceManager, PluginLoadReturningNullFails)
{
  auto resource_manager = make_resource_manager();
  EXPECT_FALSE(
    load(
      *resource_manager, robot_description({ros2_control_tag("system1", "null", "joint1")})));
  EXPECT_EQ(calls_.loaded, (std::vector<std::string>{"null"}));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_TRUE(resource_manager->get_components_status().empty());
}

TEST_F(TestGZResourceManager, InitSimFailureFails)
{
  auto resource_manager = make_resource_manager({{"fake1", InitSimResult::kFailure}});
  EXPECT_FALSE(
    load(
      *resource_manager, robot_description({ros2_control_tag("system1", "fake1", "joint1")})));
  EXPECT_EQ(calls_.loaded, (std::vector<std::string>{"fake1"}));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_TRUE(resource_manager->get_components_status().empty());
}

TEST_F(TestGZResourceManager, InitSimExceptionFails)
{
  auto resource_manager = make_resource_manager({{"fake1", InitSimResult::kThrow}});
  EXPECT_FALSE(
    load(
      *resource_manager, robot_description({ros2_control_tag("system1", "fake1", "joint1")})));
  EXPECT_EQ(calls_.loaded, (std::vector<std::string>{"fake1"}));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_TRUE(resource_manager->get_components_status().empty());
}

TEST_F(TestGZResourceManager, FailureStopsLoadingTheRemainingSystems)
{
  auto resource_manager = make_resource_manager({{"fake2", InitSimResult::kFailure}});
  EXPECT_FALSE(
    load(
      *resource_manager, robot_description(
        {ros2_control_tag("system1", "fake1", "joint1"),
          ros2_control_tag("system2", "fake2", "joint2"),
          ros2_control_tag("system3", "fake3", "joint2")})));
  EXPECT_FALSE(resource_manager->are_components_initialized());
  EXPECT_EQ(calls_.loaded, (std::vector<std::string>{"fake1", "fake2"}));

  // The systems initialized before the failure stay imported
  const auto & status = resource_manager->get_components_status();
  EXPECT_EQ(status.size(), 1u);
  EXPECT_TRUE(status.count("system1"));
}

TEST_F(TestGZResourceManager, UnknownPluginFailsWithPluginlib)
{
  // No system loader given, plugins are loaded with pluginlib
  gz_ros2_control::GZResourceManager resource_manager(node_, ecm_, sim_joints_);
  EXPECT_FALSE(
    load(
      resource_manager, robot_description(
        {ros2_control_tag("system1", "gz_ros2_control/DoesNotExist", "joint1")})));
  EXPECT_FALSE(resource_manager.are_components_initialized());
  EXPECT_TRUE(resource_manager.get_components_status().empty());
}
