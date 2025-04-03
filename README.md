# gz_ros2_control

ROS2 Distro | Build Status | Package build |
:---------: | :----: | :----------: |
[![Licence](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__gz_ros2_control__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__gz_ros2_control__ubuntu_jammy_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__gz_ros2_control__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__gz_ros2_control__ubuntu_jammy_amd64__binary/) |

This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Ignition Gazebo](http://ignitionrobotics.org/) simulator.
More information about `ros2_control` can be found here: https://control.ros.org/

This package provides an Ignition Gazebo system plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

ROS version | Gazebo version | Branch | Binaries hosted at | APT key
-- | -- | -- | -- | --
Humble | Fortress | [humble](https://github.com/ros-controls/gz_ros2_control/tree/humble) | https://packages.ros.org | `ros-humble-gz-ros2-control`
Humble | Harmonic | [humble](https://github.com/ros-controls/gz_ros2_control/tree/humble) | build from source | -
Jazzy | Harmonic | [jazzy](https://github.com/ros-controls/gz_ros2_control/tree/jazzy) | https://packages.ros.org | `ros-jazzy-gz-ros2-control`
Rolling | Ionic | [rolling](https://github.com/ros-controls/gz_ros2_control/tree/rolling) | https://packages.ros.org | `ros-rolling-gz-ros2-control`

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`rolling`](https://github.com/ros-controls/gz_ros2_control/tree/rolling) | [![gazebo_ros2_control CI - Rolling](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-rolling.yaml/badge.svg?branch=rolling)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-rolling.yaml) | [Documentation](https://control.ros.org/rolling/index.html) <br /> [API Reference](https://control.ros.org/rolling/doc/api/index.html)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/gz_ros2_control/tree/jazzy) | [![gazebo_ros2_control CI - Jazzy](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-jazzy.yaml/badge.svg?branch=rolling)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-jazzy.yaml) | [Documentation](https://control.ros.org/jazzy/index.html) <br /> [API Reference](https://control.ros.org/jazzy/doc/api/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/gz_ros2_control/tree/humble) | [![gz_ros2_control CI - Humble](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-humble.yaml/badge.svg?branch=humble)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-humble.yaml) | [Documentation](https://control.ros.org/humble/index.html) <br /> [API Reference](https://control.ros.org/humble/doc/api/index.html)

## Documentation
See the [documentation file](doc/index.rst) or [control.ros.org](https://control.ros.org/master/doc/gz_ros2_control/doc/index.html)

# Compile from source
If you want compile this from source, you have to choose the Gazebo version first. The default one is `fortress`, and you simply can compile it with

```bash
mkdir -p ~/gz_ros2_control_ws/src
cd ~/gz_ros2_control_ws/src
git clone https://github.com/ros-controls/gz_ros2_control -b humble
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
cd ~/gz_ros2_control_ws
colcon build
```

If you want to use `harmonic`, then follow the instructions in the [official Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic/ros_installation/#gazebo-harmonic-with-ros-2-humble-or-rolling-use-with-caution) how to install Gazebo Harmonic on ROS 2 humble, i.e, `apt-get install gz-harmonic ros-humble-ros-gzharmonic ros-humble-ros-gzharmonic-bridge`. Additionally, you need to [install the rosdep rules](https://github.com/osrf/osrf-rosdep#installing-rosdep-rules-to-resolve-gazebo-harmonic-libraries) for gazebo harmonic.

Then create a workspace, clone the correct branch of this repo and compile it by setting the environment variable `GZ_VERSION`:

```bash
mkdir -p ~/gz_ros2_control_ws/src
cd ~/gz_ros2_control_ws/src
git clone https://github.com/ros-controls/gz_ros2_control -b humble
export GZ_VERSION=harmonic
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y --skip-keys="ros_gz_bridge ros_gz_sim"
cd ~/gz_ros2_control_ws
colcon build
```
