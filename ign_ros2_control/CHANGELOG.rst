^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ign_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.5 (2024-01-24)
------------------
* Load the URDF to the resource_manager before parsing it to CM (`#222 <https://github.com/ros-controls/gz_ros2_control/issues/222>`_) (`#225 <https://github.com/ros-controls/gz_ros2_control/issues/225>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 5a948050563881dba20effec3ccb678e4f375529)
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
* Contributors: mergify[bot]

0.7.3 (2024-01-04)
------------------

0.7.2 (2024-01-04)
------------------
* Add controller name parameter (`#212 <https://github.com/ros-controls/gz_ros2_control/issues/212>`_)
* Fix stuck passive joints (`#184 <https://github.com/ros-controls/gz_ros2_control/issues/184>`_) (`#206 <https://github.com/ros-controls/gz_ros2_control/issues/206>`_)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  (cherry picked from commit 0a8d4464e15e4f13ea0673311c60a891d6a836ec)
  Co-authored-by: Johannes Huemer <johannes.huemer@ait.ac.at>
* Contributors: Jakub Delicat, mergify[bot]

0.7.1 (2023-08-23)
------------------
* Catch pluginlib exceptions (backport `#175 <https://github.com/ros-controls/gz_ros2_control/issues/175>`_) (`#176 <https://github.com/ros-controls/gz_ros2_control/issues/176>`_)
* Fixed install include (`#162 <https://github.com/ros-controls/gz_ros2_control/issues/162>`_)
* Install include directory since it is exported (`#127 <https://github.com/ros-controls/gz_ros2_control/issues/127>`_) (`#161 <https://github.com/ros-controls/gz_ros2_control/issues/161>`_)
  (cherry picked from commit ab33f76d158e533c21537bc408f636831819e875)
  Co-authored-by: Tim Clephas <tim.clephas@nobleo.nl>
* Remove plugin export (ROS 1 syntax) (backport `#158 <https://github.com/ros-controls/gz_ros2_control/issues/158>`_) (`#159 <https://github.com/ros-controls/gz_ros2_control/issues/159>`_)
  * Remove plugin export from ROS 1 (`#158 <https://github.com/ros-controls/gz_ros2_control/issues/158>`_)
  (cherry picked from commit 223086dea282ea3ff2c432414396952b9cfeec88)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, Christoph Fröhlich, mergify[bot]

0.5.0 (2023-05-23)
------------------
* Fixed segmentation fault with logger (backport `#136 <https://github.com/ros-controls/gz_ros2_control/issues/136>`_) (`#140 <https://github.com/ros-controls/gz_ros2_control/issues/140>`_)
* Contributors: mergify[bot]

0.4.4 (2023-03-28)
------------------
* Context and Namespace Handling for Multi-Robot Sim (`#92 <https://github.com/ros-controls/gz_ros2_control/issues/92>`_)
* Various bug fixes (`#114 <https://github.com/ros-controls/gz_ros2_control/issues/114>`_)
* Force setting use_sim_time parameter when using plugin. (`#100 <https://github.com/ros-controls/gz_ros2_control/issues/100>`_)
* Enable loading params from multiple yaml files (`#94 <https://github.com/ros-controls/gz_ros2_control/issues/94>`_)
* Add support for mimic joints. (`#33 <https://github.com/ros-controls/gz_ros2_control/issues/33>`_)
* Set right initial velocity (`#81 <https://github.com/ros-controls/gz_ros2_control/issues/81>`_)
* Fix setting initial values if command interfaces are not defined. (`#73 <https://github.com/ros-controls/gz_ros2_control/issues/73>`_)
* activated all hardware by default and improved variable naming (`#74 <https://github.com/ros-controls/gz_ros2_control/issues/74>`_)
* Implemented perform_command_mode_switch override in GazeboSystem (`#76 <https://github.com/ros-controls/gz_ros2_control/issues/76>`_)
* Remove warnings (`#72 <https://github.com/ros-controls/gz_ros2_control/issues/72>`_)
* change component name for ignition (`#69 <https://github.com/ros-controls/gz_ros2_control/issues/69>`_)
* Added logic for activating hardware interfaces (`#68 <https://github.com/ros-controls/gz_ros2_control/issues/68>`_)
* Force setting use_sim_time parameter when using plugin. (`#100 <https://github.com/ros-controls/gz_ros2_control/issues/100>`_) (`#102 <https://github.com/ros-controls/gz_ros2_control/issues/102>`_)
* Contributors: Alejandro Hernández Cordero, Andy Zelenak, Bence Magyar, Denis Štogl, Lovro, Tianyu Li, sp-sophia-labs

0.4.3 (2023-02-16)
------------------
* Fix example demos in humble branch `#118 <https://github.com/ros-controls/gz_ros2_control/issues/118>`_ from iche033/iche033/fix_humble_demos
* Remove URDF dependency (`#56 <https://github.com/ros-controls/gz_ros2_control/issues/56>`_)
* Adapt to ROS 2 Humble
* typo in citadel name (`#54 <https://github.com/ros-controls/gz_ros2_control/issues/54>`_)
* ros2_control is now having usings under its namespace. (`#43 <https://github.com/ros-controls/gz_ros2_control/issues/43>`_)
* Fix default ign gazebo version Rolling (`#45 <https://github.com/ros-controls/gz_ros2_control/issues/45>`_)
* Fix ignition version in package.xml - Rolling (`#41 <https://github.com/ros-controls/gz_ros2_control/issues/41>`_)
* Add support for initial_values for hardware interfaces when starting simulation. (`#27 <https://github.com/ros-controls/gz_ros2_control/issues/27>`_)
* Contributors: Alejandro Hernández Cordero, Denis Štogl, Guillaume Beuzeboc, Tianyu Li

0.4.0 (2022-03-18)
------------------
* Fix default ign gazebo version Galactic (`#44 <https://github.com/ignitionrobotics/ign_ros2_control/issues/44>`_)
* Contributors: Alejandro Hernández Cordero

0.3.0 (2022-03-16)
------------------
* Fix ignition version in package.xml (`#40 <https://github.com/ignitionrobotics/ign_ros2_control/issues/40>`_)
* Contributors: Alejandro Hernández Cordero

0.2.0 (2022-02-17)
------------------
* Merge pull request `#36 <https://github.com/ignitionrobotics/ign_ros2_control/issues/36>`_ from ignitionrobotics/ahcorde/foxy_to_galactic
  Foxy -> Galactic
* Merge remote-tracking branch 'origin/foxy' into ahcorde/foxy_to_galactic
* typo fix. (`#25 <https://github.com/ignitionrobotics/ign_ros2_control/issues/25>`_)
* Contributors: Alejandro Hernández Cordero, Tomoya Fujita

0.1.2 (2022-02-14)
------------------
* Fixed position control (`#29 <https://github.com/ignitionrobotics/ign_ros2_control/issues/29>`_) (`#34 <https://github.com/ignitionrobotics/ign_ros2_control/issues/34>`_)
* typo fix. (`#25 <https://github.com/ignitionrobotics/ign_ros2_control/issues/25>`_) (`#26 <https://github.com/ignitionrobotics/ign_ros2_control/issues/26>`_)
  Co-authored-by: Tomoya Fujita <Tomoya.Fujita@sony.com>
* Contributors: Alejandro Hernández Cordero

0.1.1 (2022-01-07)
------------------
* Change package names from ignition\_ to ign\_ (`#19 <https://github.com/ignitionrobotics/ign_ros2_control/pull/22>`_)
  * Change package names from ignition\_ to ign\_
* Contributors: Alejandro Hernández Cordero

0.1.0 (2022-01-07)
------------------
* Ignition ros2 control (`#1 <https://github.com/ignitionrobotics/ign_ros2_control/issues/1>`_)
  Co-authored-by: ahcorde <ahcorde@gmail.com>
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Contributors: Alejandro Hernández Cordero, Louise Poubel, Vatan Aksoy Tezer
