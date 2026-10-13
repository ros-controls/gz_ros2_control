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
#include <optional>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <sdf/Element.hh>
#include <sdf/Joint.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>

#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>

#include "gz_ros2_control_plugin_utils.hpp"

namespace
{
using gz_ros2_control::detail::BuildRosArguments;
using gz_ros2_control::detail::CheckPeriodMismatch;
using gz_ros2_control::detail::GetEnabledJoints;
using gz_ros2_control::detail::NormalizeNamespace;
using gz_ros2_control::detail::ParsePluginConfig;
using gz_ros2_control::detail::PeriodMismatch;
using gz_ros2_control::detail::PluginConfig;

// Parse a <plugin> element with the given contents the same way Gazebo does
class PluginSdf
{
public:
  explicit PluginSdf(const std::string & plugin_contents)
  {
    const std::string sdf_string =
      "<?xml version='1.0'?>"
      "<sdf version='1.10'>"
      "  <model name='robot'>"
      "    <link name='base'/>"
      "    <plugin filename='gz_ros2_control-system'"
      "            name='gz_ros2_control::GazeboSimROS2ControlPlugin'>" +
      plugin_contents +
      "    </plugin>"
      "  </model>"
      "</sdf>";
    const auto errors = root_.LoadSdfString(sdf_string);
    EXPECT_TRUE(errors.empty());
  }

  sdf::ElementConstPtr Element() const
  {
    return root_.Model()->Element()->FindElement("plugin");
  }

private:
  sdf::Root root_;
};

std::optional<PluginConfig> Parse(const std::string & plugin_contents, std::string & error)
{
  const PluginSdf sdf(plugin_contents);
  return ParsePluginConfig(sdf.Element(), error);
}

std::optional<PluginConfig> Parse(const std::string & plugin_contents)
{
  std::string error;
  return Parse(plugin_contents, error);
}

class GetEnabledJointsTest : public ::testing::Test
{
protected:
  gz::sim::Entity CreateModel(const std::string & name)
  {
    const auto model = ecm_.CreateEntity();
    ecm_.CreateComponent(model, gz::sim::components::Model());
    ecm_.CreateComponent(model, gz::sim::components::Name(name));
    return model;
  }

  gz::sim::Entity CreateJoint(
    const gz::sim::Entity & model, const std::string & name, sdf::JointType type)
  {
    const auto joint = ecm_.CreateEntity();
    ecm_.CreateComponent(joint, gz::sim::components::Joint());
    ecm_.CreateComponent(joint, gz::sim::components::Name(name));
    ecm_.CreateComponent(joint, gz::sim::components::JointType(type));
    ecm_.CreateComponent(joint, gz::sim::components::ParentEntity(model));
    return joint;
  }

  gz::sim::EntityComponentManager ecm_;
  rclcpp::Logger logger_{rclcpp::get_logger("test_gz_ros2_control_plugin")};
};
}  // namespace

TEST(ParsePluginConfig, DefaultsWithOnlyParameters)
{
  const auto config = Parse("<parameters>controllers.yaml</parameters>");
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->parameter_files, std::vector<std::string>{"controllers.yaml"});
  EXPECT_EQ(config->controller_manager_name, "controller_manager");
  EXPECT_TRUE(config->hold_joints);
  EXPECT_DOUBLE_EQ(config->position_proportional_gain, 0.1);
  EXPECT_EQ(config->ns, "/");
  EXPECT_TRUE(config->remappings.empty());
}

TEST(ParsePluginConfig, AllTagsOverridden)
{
  const auto config = Parse(
    "<parameters>controllers.yaml</parameters>"
    "<controller_manager_name>my_cm</controller_manager_name>"
    "<hold_joints>false</hold_joints>"
    "<position_proportional_gain>2.5</position_proportional_gain>"
    "<ros>"
    "  <namespace>/robot</namespace>"
    "  <remapping>/cmd:=/robot/cmd</remapping>"
    "</ros>");
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->controller_manager_name, "my_cm");
  EXPECT_FALSE(config->hold_joints);
  EXPECT_DOUBLE_EQ(config->position_proportional_gain, 2.5);
  EXPECT_EQ(config->ns, "/robot");
  EXPECT_EQ(config->remappings, std::vector<std::string>{"/cmd:=/robot/cmd"});
}

TEST(ParsePluginConfig, MultipleParametersKeptInOrder)
{
  const auto config = Parse(
    "<parameters>first.yaml</parameters>"
    "<parameters>second.yaml</parameters>"
    "<parameters>third.yaml</parameters>");
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(
    config->parameter_files,
    (std::vector<std::string>{"first.yaml", "second.yaml", "third.yaml"}));
}

TEST(ParsePluginConfig, MissingParametersFails)
{
  std::string error;
  EXPECT_FALSE(Parse("<hold_joints>false</hold_joints>", error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST(ParsePluginConfig, EmptyParametersFails)
{
  std::string error;
  EXPECT_FALSE(Parse("<parameters></parameters>", error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST(ParsePluginConfig, LaterEmptyParametersFails)
{
  std::string error;
  EXPECT_FALSE(
    Parse(
      "<parameters>controllers.yaml</parameters>"
      "<parameters></parameters>", error).has_value());
  EXPECT_FALSE(error.empty());
}

TEST(ParsePluginConfig, MultipleRemappingsKeptInOrder)
{
  const auto config = Parse(
    "<parameters>controllers.yaml</parameters>"
    "<ros>"
    "  <remapping>a:=b</remapping>"
    "  <remapping>c:=d</remapping>"
    "</ros>");
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->remappings, (std::vector<std::string>{"a:=b", "c:=d"}));
}

TEST(ParsePluginConfig, RosWithoutNamespaceKeepsRoot)
{
  const auto config = Parse(
    "<parameters>controllers.yaml</parameters>"
    "<ros><remapping>a:=b</remapping></ros>");
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->ns, "/");
}

TEST(ParsePluginConfig, RelativeNamespaceMadeAbsolute)
{
  const auto config = Parse(
    "<parameters>controllers.yaml</parameters>"
    "<ros><namespace>robot</namespace></ros>");
  ASSERT_TRUE(config.has_value());
  EXPECT_EQ(config->ns, "/robot");
}

TEST(NormalizeNamespace, AddsLeadingSlash)
{
  EXPECT_EQ(NormalizeNamespace(""), "/");
  EXPECT_EQ(NormalizeNamespace("ns"), "/ns");
  EXPECT_EQ(NormalizeNamespace("/ns"), "/ns");
  EXPECT_EQ(NormalizeNamespace("a/b"), "/a/b");
}

TEST(NormalizeNamespace, RemovesTrailingSlash)
{
  EXPECT_EQ(NormalizeNamespace("/"), "/");
  EXPECT_EQ(NormalizeNamespace("//"), "/");
  EXPECT_EQ(NormalizeNamespace("ns/"), "/ns");
  EXPECT_EQ(NormalizeNamespace("/a/b//"), "/a/b");
}

TEST(BuildRosArguments, ParameterFilesOnly)
{
  PluginConfig config;
  config.parameter_files = {"a.yaml", "b.yaml"};
  EXPECT_EQ(
    BuildRosArguments(config),
    (std::vector<std::string>{
    "--ros-args", "--params-file", "a.yaml", "--params-file", "b.yaml"}));
}

TEST(BuildRosArguments, ParameterFilesAndRemappings)
{
  PluginConfig config;
  config.parameter_files = {"a.yaml"};
  config.remappings = {"x:=y", "z:=w"};
  EXPECT_EQ(
    BuildRosArguments(config),
    (std::vector<std::string>{
    "--ros-args", "--params-file", "a.yaml", "--remap", "x:=y", "--remap", "z:=w"}));
}

TEST(BuildRosArguments, NoParameterFiles)
{
  EXPECT_EQ(BuildRosArguments(PluginConfig{}), std::vector<std::string>{"--ros-args"});
}

TEST_F(GetEnabledJointsTest, SingleAxisJointsKept)
{
  const auto model = CreateModel("robot");
  const auto revolute = CreateJoint(model, "revolute", sdf::JointType::REVOLUTE);
  const auto prismatic = CreateJoint(model, "prismatic", sdf::JointType::PRISMATIC);
  const auto continuous = CreateJoint(model, "continuous", sdf::JointType::CONTINUOUS);
  const auto gearbox = CreateJoint(model, "gearbox", sdf::JointType::GEARBOX);

  const std::map<std::string, gz::sim::Entity> expected = {
    {"revolute", revolute},
    {"prismatic", prismatic},
    {"continuous", continuous},
    {"gearbox", gearbox}};
  EXPECT_EQ(GetEnabledJoints(model, ecm_, logger_), expected);
}

TEST_F(GetEnabledJointsTest, FixedJointsSkipped)
{
  const auto model = CreateModel("robot");
  const auto revolute = CreateJoint(model, "revolute", sdf::JointType::REVOLUTE);
  CreateJoint(model, "fixed", sdf::JointType::FIXED);

  const std::map<std::string, gz::sim::Entity> expected = {{"revolute", revolute}};
  EXPECT_EQ(GetEnabledJoints(model, ecm_, logger_), expected);
}

TEST_F(GetEnabledJointsTest, MultiAxisJointsSkipped)
{
  const auto model = CreateModel("robot");
  CreateJoint(model, "ball", sdf::JointType::BALL);
  CreateJoint(model, "screw", sdf::JointType::SCREW);
  CreateJoint(model, "universal", sdf::JointType::UNIVERSAL);
  CreateJoint(model, "revolute2", sdf::JointType::REVOLUTE2);

  EXPECT_TRUE(GetEnabledJoints(model, ecm_, logger_).empty());
}

TEST_F(GetEnabledJointsTest, OtherModelsIgnored)
{
  const auto robot = CreateModel("robot");
  const auto other = CreateModel("other");
  const auto joint = CreateJoint(robot, "joint", sdf::JointType::REVOLUTE);
  CreateJoint(other, "other_joint", sdf::JointType::REVOLUTE);

  const std::map<std::string, gz::sim::Entity> expected = {{"joint", joint}};
  EXPECT_EQ(GetEnabledJoints(robot, ecm_, logger_), expected);
}

TEST_F(GetEnabledJointsTest, JointWithoutNameSkipped)
{
  const auto model = CreateModel("robot");
  const auto revolute = CreateJoint(model, "revolute", sdf::JointType::REVOLUTE);
  const auto unnamed = CreateJoint(model, "unnamed", sdf::JointType::REVOLUTE);
  ecm_.RemoveComponent<gz::sim::components::Name>(unnamed);

  const std::map<std::string, gz::sim::Entity> expected = {{"revolute", revolute}};
  EXPECT_EQ(GetEnabledJoints(model, ecm_, logger_), expected);
}

TEST_F(GetEnabledJointsTest, JointWithoutTypeSkipped)
{
  const auto model = CreateModel("robot");
  const auto revolute = CreateJoint(model, "revolute", sdf::JointType::REVOLUTE);
  const auto untyped = CreateJoint(model, "untyped", sdf::JointType::REVOLUTE);
  ecm_.RemoveComponent<gz::sim::components::JointType>(untyped);

  const std::map<std::string, gz::sim::Entity> expected = {{"revolute", revolute}};
  EXPECT_EQ(GetEnabledJoints(model, ecm_, logger_), expected);
}

TEST_F(GetEnabledJointsTest, ModelWithoutJoints)
{
  const auto model = CreateModel("robot");
  EXPECT_TRUE(GetEnabledJoints(model, ecm_, logger_).empty());
}

TEST(CheckPeriodMismatch, ComparesPeriods)
{
  const rclcpp::Duration one_ms(0, 1000000);
  const rclcpp::Duration ten_ms(0, 10000000);
  EXPECT_EQ(CheckPeriodMismatch(one_ms, one_ms), PeriodMismatch::kNone);
  EXPECT_EQ(CheckPeriodMismatch(one_ms, ten_ms), PeriodMismatch::kControlFaster);
  EXPECT_EQ(CheckPeriodMismatch(ten_ms, one_ms), PeriodMismatch::kControlSlower);
}
