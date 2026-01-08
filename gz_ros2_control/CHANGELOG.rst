^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ign_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.18 (2026-01-08)
-------------------
* Minor code consistency fix (`#732 <https://github.com/ros-controls/gz_ros2_control/issues/732>`_) (`#735 <https://github.com/ros-controls/gz_ros2_control/issues/735>`_)
* Contributors: mergify[bot]

0.7.17 (2025-09-29)
-------------------
* Remove outdated comment (`#687 <https://github.com/ros-controls/gz_ros2_control/issues/687>`_) (`#691 <https://github.com/ros-controls/gz_ros2_control/issues/691>`_)
* Don't remove the node at destruction (`#683 <https://github.com/ros-controls/gz_ros2_control/issues/683>`_) (`#686 <https://github.com/ros-controls/gz_ros2_control/issues/686>`_)
* Fix compiler warnings (backport `#674 <https://github.com/ros-controls/gz_ros2_control/issues/674>`_) (`#677 <https://github.com/ros-controls/gz_ros2_control/issues/677>`_)
* Contributors: mergify[bot]

0.7.16 (2025-08-18)
-------------------
* Provide force-torque sensor data through gz_system to controller_manager - fixes to original PR for Humble (`#637 <https://github.com/ros-controls/gz_ros2_control/issues/637>`_)
* use gz-physics`#283 <https://github.com/ros-controls/gz_ros2_control/issues/283>`_ to implement joint_states/effort feedback (backport `#186 <https://github.com/ros-controls/gz_ros2_control/issues/186>`_) (`#607 <https://github.com/ros-controls/gz_ros2_control/issues/607>`_)
* Contributors: Bartłomiej Krajewski, Christoph Fröhlich

0.7.15 (2025-05-23)
-------------------

0.7.14 (2025-04-21)
-------------------

0.7.13 (2025-04-04)
-------------------
* Rename ign to gz (backport `#67 <https://github.com/ros-controls/gz_ros2_control/issues/67>`_) (`#515 <https://github.com/ros-controls/gz_ros2_control/issues/515>`_)
* Contributors: Christoph Fröhlich

0.7.12 (2025-02-20)
-------------------
* Remove comments in robot_description string (`#505 <https://github.com/ros-controls/gz_ros2_control/issues/505>`_)
* Contributors: Christoph Fröhlich

0.7.11 (2025-02-07)
-------------------

0.7.10 (2025-02-05)
-------------------
* Set robot description parameter for controllers (`#477 <https://github.com/ros-controls/gz_ros2_control/issues/477>`_)
* Contributors: AB

0.7.9 (2024-07-02)
------------------
* Don't crash if a wrong config was detected (`#324 <https://github.com/ros-controls/gz_ros2_control/issues/324>`_) (`#330 <https://github.com/ros-controls/gz_ros2_control/issues/330>`_)
  (cherry picked from commit ec1b95893fa933cb3e2cc5341bb65dd621645785)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Harden behavior if a joint is not found in the model (backport `#325 <https://github.com/ros-controls/gz_ros2_control/issues/325>`_) (`#326 <https://github.com/ros-controls/gz_ros2_control/issues/326>`_)
  * Don't crash if a joint does not exist
* Changed to use spin instead of spin_once to enable multithreading with MultiThreadedExecutor (`#315 <https://github.com/ros-controls/gz_ros2_control/issues/315>`_) (`#320 <https://github.com/ros-controls/gz_ros2_control/issues/320>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 45035076408e6014cc911e8d4bc169572a25008c)
  Co-authored-by: Takashi Sato <t.sato17123@gmail.com>
* Contributors: Christoph Fröhlich, mergify[bot]

0.7.8 (2024-05-14)
------------------
* Fixed target of ament_export_libraries (`#300 <https://github.com/ros-controls/gz_ros2_control/issues/300>`_)
* Added parameters robot_param and robot_param_node (`#275 <https://github.com/ros-controls/gz_ros2_control/issues/275>`_)
* Update precommit config (backport `#271 <https://github.com/ros-controls/gz_ros2_control/issues/271>`_) (`#278 <https://github.com/ros-controls/gz_ros2_control/issues/278>`_)
  * Update precommit config (`#271 <https://github.com/ros-controls/gz_ros2_control/issues/271>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 492ed646010fc55a6acc32d07138ddda8824aff5)
  * make linters happy
  ---------
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, mergify[bot]

0.7.7 (2024-04-09)
------------------
* Fix `#259 <https://github.com/ros-controls/gz_ros2_control/issues/259>`_ - `ParameterAlreadyDeclaredException` for parameter `position_proportional_gain` (backport `#261 <https://github.com/ros-controls/gz_ros2_control/issues/261>`_) (`#262 <https://github.com/ros-controls/gz_ros2_control/issues/262>`_)
  Co-authored-by: Patrick Roncagliolo <ronca.pat@gmail.com>
  Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

0.7.6 (2024-03-21)
------------------
* Fix typo (`#253 <https://github.com/ros-controls/gz_ros2_control/issues/253>`_) (`#254 <https://github.com/ros-controls/gz_ros2_control/issues/254>`_)
  (cherry picked from commit a98cb2a8b72827b7c1669987d6a12d3f0b30a41e)
  Co-authored-by: Stephanie Eng <stephanie-eng@users.noreply.github.com>
* Fix `#247 <https://github.com/ros-controls/gz_ros2_control/issues/247>`_ (`#248 <https://github.com/ros-controls/gz_ros2_control/issues/248>`_) (`#250 <https://github.com/ros-controls/gz_ros2_control/issues/250>`_)
  (cherry picked from commit 94745e6f5f051214ac9862051f9a918685f2c6b9)
  Co-authored-by: Graziato Davide <85335579+Fixit-Davide@users.noreply.github.com>
* Fix `initial_value` not working (backport `#241 <https://github.com/ros-controls/gz_ros2_control/issues/241>`_) (`#243 <https://github.com/ros-controls/gz_ros2_control/issues/243>`_)
  * Reset Gazebo with initial joint positions and velocities (`#241 <https://github.com/ros-controls/gz_ros2_control/issues/241>`_)
  (cherry picked from commit c5b0b9049ce75410e75d1828242c1dfd5b19bb80)
  Co-authored-by: Ruddick Lawrence <679360+mrjogo@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Use portable versio for usleep (backport `#237 <https://github.com/ros-controls/gz_ros2_control/issues/237>`_) (`#238 <https://github.com/ros-controls/gz_ros2_control/issues/238>`_)
  * Use portable versio for usleep (`#237 <https://github.com/ros-controls/gz_ros2_control/issues/237>`_)
  (cherry picked from commit 0bdf13e6986c613c99a595889a587da1db6d7f69)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Fix crashing due to an invalid parameter in the initial value (backport `#233 <https://github.com/ros-controls/gz_ros2_control/issues/233>`_) (`#234 <https://github.com/ros-controls/gz_ros2_control/issues/234>`_)
  * Fix crashing due to an invalid parameter in the initial value (`#233 <https://github.com/ros-controls/gz_ros2_control/issues/233>`_)
  (cherry picked from commit a3beadb014f62e0808033de2c5ad84e2428c36e9)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, Ruddick Lawrence, Stephanie Eng, Graziato Davide, mergify[bot]

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
