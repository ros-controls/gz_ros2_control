# ign_ros2_control

ROS2 Distro | Build Status | Package build |
:---------: | :----: | :----------: |
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) |  [![Build Status](http://build.ros2.org/buildStatus/icon?job=Hdev__gz_ros2_control__ubuntu_focal_amd64)](http://build.ros2.org/job/Hdev__gz_ros2_control__ubuntu_focal_amd64) |  [![Build Status](http://build.ros2.org/buildStatus/icon?job=Hbin_uF64__gz_ros2_control__ubuntu_focal_amd64__binary)](http://build.ros2.org/job/Hbin_uF64__gz_ros2_control__ubuntu_focal_amd64__binary) |

This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Ignition Gazebo](http://ignitionrobotics.org/) simulator.
More information about `ros2_control` can be found here: https://control.ros.org/

This package provides an Ignition Gazebo system plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

ROS version | Gazebo version | Branch | Binaries hosted at
-- | -- | -- | --
Foxy | Citadel | [foxy](https://github.com/ros-controls/gz_ros2_control/tree/foxy) | https://packages.ros.org
Foxy | Edifice | [foxy](https://github.com/ros-controls/gz_ros2_control/tree/foxy) | only from source
Galactic | Edifice | [galactic](https://github.com/ros-controls/gz_ros2_control/tree/galactic) | https://packages.ros.org
Galactic | Fortress | [galactic](https://github.com/ros-controls/gz_ros2_control/tree/galactic) | only from source
Humble | Fortress | [ros2](https://github.com/ros-controls/gz_ros2_control/tree/master) | https://packages.ros.org
Rolling | Edifice | [ros2](https://github.com/ros-controls/gz_ros2_control/tree/master) | only from source
Rolling | Fortress | [ros2](https://github.com/ros-controls/gz_ros2_control/tree/master) | https://packages.ros.org
Rolling | Garden (not released) | [ros2](https://github.com/ros-controls/gz_ros2_control/tree/master) | only from source

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`master`](https://github.com/ros-controls/gz_ros2_control/tree/master) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci.yaml) | [Documentation](https://control.ros.org/master/index.html) <br /> [API Reference](https://control.ros.org/master/doc/api/index.html)
**Iron** | [`master`](https://github.com/ros-controls/gz_ros2_control/tree/master) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci.yaml/badge.svg?branch=master)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci.yaml) | [Documentation](https://control.ros.org/master/index.html) <br /> [API Reference](https://control.ros.org/master/doc/api/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/gz_ros2_control/tree/humble) | [![Gazebo ros2 control CI](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci.yaml/badge.svg?branch=humble)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci.yaml) | [Documentation](https://control.ros.org/humble/index.html) <br /> [API Reference](https://control.ros.org/humble/doc/api/index.html)
## Documentation
See the [documentation file](doc/index.rst) or [control.ros.org](https://control.ros.org/master/doc/simulators/gz_ros2_control/doc/index.html)

# Compile from source

If you want compile this from source, you should choose the Ignition version. The default one is `citadel`:

```bash
export IGNITION_VERSION=citadel
export IGNITION_VERSION=edifice
export IGNITION_VERSION=fortress
```

Then create a workspace, clone the repo and compile it:

```bash
mkdir -p ~/ros_ign/src
cd ~/ros_ign/src
git clone https://github.com/ros-controls/gz_ros2_control
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/ros2_ign
colcon build
```
