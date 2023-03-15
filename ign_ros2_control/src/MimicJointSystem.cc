/*
 * Copyright 2023 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2023-03-15
 *
 */
//----------------------------------------------------------------------

#include "ign_ros2_control/MimicJointSystem.hh"

#include <string>
#include <unordered_set>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/JointType.hh"

#include "ignition/gazebo/Model.hh"

using namespace ign_ros2_control;

class ign_ros2_control::MimicJointSystemPrivate {

/// \brief Joint Entity

public:
  ignition::gazebo::Entity jointEntity {ignition::gazebo::kNullEntity};

public:
  ignition::gazebo::Entity mimicJointEntity {ignition::gazebo::kNullEntity};

/// \brief Joint name

public:
  std::string jointName;

public:
  std::string mimicJointName;

/// \brief Commanded joint position

public:
  double jointPosCmd {0.0};

public:
  double multiplier {1.0};

public:
  double offset {0.0};

/// \brief Model interface

public:
  ignition::gazebo::Model model {ignition::gazebo::kNullEntity};

/// \brief Position PID controller.

public:
  ignition::math::PID posPid;

/// \brief Joint index to be used.

public:
  unsigned int jointIndex = 0u;

public:
  unsigned int mimicJointIndex = 0u;

public:
  double deadZone = 1e-6;

  // approach adopted from https://github.com/gazebosim/gz-sim/blob/330eaf2f135301e90096fe91897f052cdaa77013/src/systems/joint_position_controller/JointPositionController.cc#L69-L77
/// \brief Operation modes
  enum OperationMode
  {
    /// \brief Use PID to achieve positional control
    PID,
    /// \brief Bypass PID completely. This means the joint will move to that
    /// position bypassing the physics engine.
    ABS
  };

/// \brief Joint position mode

public:
  OperationMode mode = OperationMode::PID;

};

MimicJointSystem::MimicJointSystem()
: dataPtr(std::make_unique < ign_ros2_control::MimicJointSystemPrivate > ())
{
}

void MimicJointSystem::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr < const sdf::Element > & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /* _eventMgr*/)
{

  // Make sure the controller is attached to a valid model
  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {

    ignerr << "MimicJointSystem  Failed to initialize because [" <<
      this->dataPtr->model.Name(_ecm) <<
      "] (Entity=" << _entity << ")] is not a model.. ";
    return;
  }

  // Get params from SDF
  this->dataPtr->jointName = _sdf->Get < std::string > ("joint_name");

  if (this->dataPtr->jointName.empty()) {
    ignerr << "MimicJointSystem found an empty joint_name parameter. "
           << "Failed to initialize.";
    return;
  }

  this->dataPtr->mimicJointName = _sdf->Get < std::string > ("mimic_joint_name");
  if (this->dataPtr->mimicJointName.empty()) {
    ignerr << "MimicJointSystem found an empty mimic_joint_name parameter. "
           << "Failed to initialize.";
    return;
  }

  if (_sdf->HasElement("multiplier")) {
    this->dataPtr->multiplier = _sdf->Get < double > ("multiplier");
  }

  if (_sdf->HasElement("offset")) {
    this->dataPtr->offset = _sdf->Get < double > ("offset");
  }

  if (_sdf->HasElement("joint_index")) {
    this->dataPtr->jointIndex = _sdf->Get < unsigned int > ("joint_index");
  }

  if (_sdf->HasElement("mimic_joint_index")) {
    this->dataPtr->mimicJointIndex = _sdf->Get < unsigned int > ("mimic_joint_index");
  }

  if (_sdf->HasElement("dead_zone")) {
    this->dataPtr->deadZone = _sdf->Get < double > ("dead_zone");
  }

  // PID parameters
  double p = 1;
  double i = 0.1;
  double d = 0.01;
  double iMax = 1;
  double iMin = -1;
  double cmdMax = 1000;
  double cmdMin = -1000;
  double cmdOffset = 0;

  if (_sdf->HasElement("p_gain")) {
    p = _sdf->Get < double > ("p_gain");
  }
  if (_sdf->HasElement("i_gain")) {
    i = _sdf->Get < double > ("i_gain");
  }
  if (_sdf->HasElement("d_gain")) {
    d = _sdf->Get < double > ("d_gain");
  }
  if (_sdf->HasElement("i_max")) {
    iMax = _sdf->Get < double > ("i_max");
  }
  if (_sdf->HasElement("i_min")) {
    iMin = _sdf->Get < double > ("i_min");
  }
  if (_sdf->HasElement("cmd_max")) {
    cmdMax = _sdf->Get < double > ("cmd_max");
  }
  if (_sdf->HasElement("cmd_min")) {
    cmdMin = _sdf->Get < double > ("cmd_min");
  }
  if (_sdf->HasElement("cmd_offset")) {
    cmdOffset = _sdf->Get < double > ("cmd_offset");
  }
  if (_sdf->HasElement("use_velocity_commands")) {
    auto useVelocityCommands = _sdf->Get < bool > ("use_velocity_commands");
    if (useVelocityCommands) {
      this->dataPtr->mode =
        MimicJointSystemPrivate::OperationMode::ABS;
    }
  }

  this->dataPtr->posPid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin, cmdOffset);


  if (_sdf->HasElement("initial_position")) {
    this->dataPtr->jointPosCmd = _sdf->Get < double > ("initial_position");
  }

  igndbg << "[MimicJointSystem] system parameters:\n";
  igndbg << "p_gain: [" << p << "]\n";
  igndbg << "i_gain: [" << i << "]\n";
  igndbg << "d_gain: [" << d << "]\n";
  igndbg << "i_max: [" << iMax << "]\n";
  igndbg << "i_min: [" << iMin << "]\n";
  igndbg << "cmd_max: [" << cmdMax << "]\n";
  igndbg << "cmd_min: [" << cmdMin << "]\n";
  igndbg << "cmd_offset: [" << cmdOffset << "]\n";
  igndbg << "initial_position: [" << this->dataPtr->jointPosCmd << "]\n";

}

void MimicJointSystem::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{

  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn << "Detected jump back in time ["
            << std::chrono::duration_cast < std::chrono::seconds > (_info.dt).count()
            << "s]. System may not work properly." << std::endl;
  }

  // If the joint hasn't been identified yet, look for it
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity ||
    this->dataPtr->mimicJointEntity == ignition::gazebo::kNullEntity)
  {

    auto jointEntities = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), ignition::gazebo::components::Joint());

    // Iterate over all joints and verify whether they can be enabled or not
    for (const auto & jointEntity : jointEntities) {
      const auto jointName = _ecm.Component < ignition::gazebo::components::Name > (
        jointEntity)->Data();
      if (jointName == this->dataPtr->jointName) {
        this->dataPtr->jointEntity = jointEntity;
      } else if (jointName == this->dataPtr->mimicJointName) {
        this->dataPtr->mimicJointEntity = jointEntity;
      }
    }
  }

  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity)
    return;


  if (this->dataPtr->mimicJointEntity == ignition::gazebo::kNullEntity)
    return;


  if (_info.paused) {
    return;
  }

  auto jointPosComp = _ecm.Component < ignition::gazebo::components::JointPosition > (
    this->dataPtr->jointEntity);
  if (!jointPosComp) {
    _ecm.CreateComponent(
      this->dataPtr->jointEntity,
      ignition::gazebo::components::JointPosition());
  }

  auto mimicJointPosComp = _ecm.Component < ignition::gazebo::components::JointPosition > (
    this->dataPtr->mimicJointEntity);
  if (!mimicJointPosComp) {
    _ecm.CreateComponent(
      this->dataPtr->mimicJointEntity,
      ignition::gazebo::components::JointPosition());
  }

  if (jointPosComp == nullptr || jointPosComp->Data().empty() || mimicJointPosComp == nullptr ||
    mimicJointPosComp->Data().empty() )
  {
    return;
  }

  // Sanity check: Make sure that the joint index is valid.
  if (this->dataPtr->jointIndex >= jointPosComp->Data().size()) {
    static std::unordered_set < ignition::gazebo::Entity > reported;
    if (reported.find(this->dataPtr->jointEntity) == reported.end()) {
      ignerr << "[MimicJointSystem]: Detected an invalid <joint_index> "
             << "parameter. The index specified is ["
             << this->dataPtr->jointIndex << "] but joint ["
             << this->dataPtr->jointName << "] only has ["
             << jointPosComp->Data().size() << "] index[es]. "
             << "This controller will be ignored" << std::endl;
      reported.insert(this->dataPtr->jointEntity);
    }
    return;
  }

  // Sanity check: Make sure that the mimic joint index is valid.
  if (this->dataPtr->mimicJointIndex >= mimicJointPosComp->Data().size()) {
    static std::unordered_set < ignition::gazebo::Entity > mimic_reported;
    if (mimic_reported.find(this->dataPtr->mimicJointEntity) == mimic_reported.end()) {
      ignerr << "[MimicJointSystem]: Detected an invalid <joint_index> "
             << "parameter. The index specified is ["
             << this->dataPtr->mimicJointIndex << "] but joint ["
             << this->dataPtr->mimicJointName << "] only has ["
             << mimicJointPosComp->Data().size() << "] index[es]. "
             << "This controller will be ignored" << std::endl;
      mimic_reported.insert(this->dataPtr->mimicJointEntity);
    }
    return;
  }

  // Get error in position
  double error = jointPosComp->Data().at(this->dataPtr->jointIndex) -
                 (mimicJointPosComp->Data().at(this->dataPtr->mimicJointIndex) *
                  this->dataPtr->multiplier + this->dataPtr->offset);

  if (fabs(error) > this->dataPtr->deadZone) {

    // Check if the mode is ABS
    if (this->dataPtr->mode ==
      MimicJointSystemPrivate::OperationMode::ABS)
    {
      // Calculate target velcity
      double targetVel = 0;

      // Get time in seconds
      auto dt = std::chrono::duration < double > (_info.dt).count();

      // Get the maximum amount in m that this joint may move
      auto maxMovement = this->dataPtr->posPid.CmdMax() * dt;

      // Limit the maximum change to maxMovement
      if (abs(error) > maxMovement) {
        targetVel = (error < 0) ? this->dataPtr->posPid.CmdMax() :
          -this->dataPtr->posPid.CmdMax();
      } else {
        targetVel = -error;
      }

      // Update velocity command.
      auto vel = _ecm.Component < ignition::gazebo::components::JointVelocityCmd >
        (this->dataPtr->jointEntity);

      if (vel == nullptr) {
        _ecm.CreateComponent(
          this->dataPtr->jointEntity,
          ignition::gazebo::components::JointVelocityCmd({targetVel}));
      } else {
        *vel = ignition::gazebo::components::JointVelocityCmd({targetVel});
      }
      return;
    }

    // Update force command.
    double force = this->dataPtr->posPid.Update(error, _info.dt);

    auto forceComp =
      _ecm.Component < ignition::gazebo::components::JointForceCmd >
      (this->dataPtr->jointEntity);
    if (forceComp == nullptr) {
      _ecm.CreateComponent(
        this->dataPtr->jointEntity,
        ignition::gazebo::components::JointForceCmd({force}));
    } else {
      *forceComp = ignition::gazebo::components::JointForceCmd({force});
    }
  }
}

void MimicJointSystem::Update(
  const ignition::gazebo::UpdateInfo & /*_info*/,
  ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
//  ignmsg << "MimicJointSystem::Update" << std::endl;
}

void MimicJointSystem::PostUpdate(
  const ignition::gazebo::UpdateInfo & /*_info*/,
  const ignition::gazebo::EntityComponentManager & /*_ecm*/)
{
//  ignmsg << "MimicJointSystem::PostUpdate" << std::endl;
}

//! [registerMimicJointSystem]

IGNITION_ADD_PLUGIN(
  ign_ros2_control::MimicJointSystem,
  ignition::gazebo::System,
  MimicJointSystem::ISystemConfigure,
  MimicJointSystem::ISystemPreUpdate,
  MimicJointSystem::ISystemUpdate,
  MimicJointSystem::ISystemPostUpdate)
//! [registerMimicJointSystem]
