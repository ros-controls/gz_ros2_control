# gz_ros2_control

License | Build Status | Package build
:---------: | :----: | :----------:
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hdev__gz_ros2_control__ubuntu_jammy_amd64)](https://build.ros2.org/job/Hdev__gz_ros2_control__ubuntu_jammy_amd64/) |  [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__gz_ros2_control__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__gz_ros2_control__ubuntu_jammy_amd64__binary/)

This is a ROS 2 package for integrating the `ros2_control` controller architecture with the [Gazebo](http://gazebosim.org/) simulator.
More information about `ros2_control` can be found [here](https://control.ros.org/).

This package provides an Ignition Gazebo system plugin which instantiates a `ros2_control` controller manager and connects it to a Gazebo model.

## Compatibility Matrix

ROS version | Gazebo version | Branch | Binaries hosted at | APT key
-- | -- | -- | -- | --
Rolling | Ionic | [rolling](https://github.com/ros-controls/gz_ros2_control/tree/rolling) | [packages.ros.org](https://packages.ros.org) | `ros-rolling-gz-ros2-control`
Jazzy | Harmonic | [jazzy](https://github.com/ros-controls/gz_ros2_control/tree/jazzy) | [packages.ros.org](https://packages.ros.org) | `ros-jazzy-gz-ros2-control`
Humble | Fortress | [humble](https://github.com/ros-controls/gz_ros2_control/tree/humble) | [packages.ros.org](https://packages.ros.org) | `ros-humble-gz-ros2-control`
Humble | Harmonic | [humble](https://github.com/ros-controls/gz_ros2_control/tree/humble) | build from source | -

## Documentation

See the [documentation file](doc/index.rst) or [control.ros.org](https://control.ros.org/rolling/doc/gz_ros2_control/doc/index.html)

## Contributing

As an open-source project, we welcome each contributor, regardless of their background and experience. Pick a [PR](https://github.com/ros-controls/gz_ros2_control/pulls) and review it, or [create your own](https://github.com/ros-controls/gz_ros2_control/contribute)!
If you are new to the project, please read the [contributing guide](https://control.ros.org/rolling/doc/contributing/contributing.html) for more information on how to get started. We are happy to help you with your first contribution.

## Build status

ROS 2 Distro | Branch | Build status | Documentation
:----------: | :----: | :----------: | :-----------:
**Rolling** | [`rolling`](https://github.com/ros-controls/gz_ros2_control/tree/rolling) | [![gazebo_ros2_control CI - Rolling](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-rolling.yaml/badge.svg?branch=rolling)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-rolling.yaml) | [Documentation](https://control.ros.org/rolling/index.html) <br> [API Reference](https://control.ros.org/rolling/doc/api/index.html)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/gz_ros2_control/tree/jazzy) | [![gazebo_ros2_control CI - Jazzy](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-jazzy.yaml/badge.svg?branch=rolling)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-jazzy.yaml) | [Documentation](https://control.ros.org/jazzy/index.html) <br> [API Reference](https://control.ros.org/jazzy/doc/api/index.html)
**Humble** | [`humble`](https://github.com/ros-controls/gz_ros2_control/tree/humble) | [![ign_ros2_control CI - Humble](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-humble.yaml/badge.svg?branch=humble)](https://github.com/ros-controls/gz_ros2_control/actions/workflows/ci-humble.yaml) | [Documentation](https://control.ros.org/humble/index.html) <br> [API Reference](https://control.ros.org/humble/doc/api/index.html)

## Acknowledgements

The project has received major contributions from companies and institutions [listed on control.ros.org](https://control.ros.org/rolling/doc/acknowledgements/acknowledgements.html)
