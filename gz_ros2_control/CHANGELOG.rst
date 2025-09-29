^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ign_ros2_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.16 (2025-09-29)
-------------------
* Modernize CMakeLists (backport `#688 <https://github.com/ros-controls/gz_ros2_control/issues/688>`_) (`#698 <https://github.com/ros-controls/gz_ros2_control/issues/698>`_)
* Remove outdated comment (`#687 <https://github.com/ros-controls/gz_ros2_control/issues/687>`_) (`#690 <https://github.com/ros-controls/gz_ros2_control/issues/690>`_)
* Don't remove the node at destruction (`#683 <https://github.com/ros-controls/gz_ros2_control/issues/683>`_) (`#685 <https://github.com/ros-controls/gz_ros2_control/issues/685>`_)
* Fix compiler warnings (`#674 <https://github.com/ros-controls/gz_ros2_control/issues/674>`_) (`#676 <https://github.com/ros-controls/gz_ros2_control/issues/676>`_)
* Contributors: mergify[bot]

1.2.15 (2025-08-18)
-------------------

1.2.14 (2025-07-09)
-------------------
* Provide force-torque sensor data through gz_system to controller_manager - fixes to original PR  (backport `#610 <https://github.com/ros-controls/gz_ros2_control/issues/610>`_) (`#624 <https://github.com/ros-controls/gz_ros2_control/issues/624>`_)
* Bump CMake minimum version (`#601 <https://github.com/ros-controls/gz_ros2_control/issues/601>`_) (`#603 <https://github.com/ros-controls/gz_ros2_control/issues/603>`_)
* Use ros2_control_cmake (`#588 <https://github.com/ros-controls/gz_ros2_control/issues/588>`_) (`#593 <https://github.com/ros-controls/gz_ros2_control/issues/593>`_)
* Contributors: mergify[bot]

1.2.13 (2025-05-23)
-------------------

1.2.12 (2025-04-04)
-------------------
* Set `use_sim_time` through CM NodeOptions (`#533 <https://github.com/ros-controls/gz_ros2_control/issues/533>`_) (`#538 <https://github.com/ros-controls/gz_ros2_control/issues/538>`_)
* Make the system backward compatible with the old ign* plugins (`#520 <https://github.com/ros-controls/gz_ros2_control/issues/520>`_)
* Contributors: Christoph Fröhlich, mergify[bot]

1.2.11 (2025-02-19)
-------------------
* Add remap option to controller manager (`#442 <https://github.com/ros-controls/gz_ros2_control/issues/442>`_)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: Tatsuro Sakaguchi

1.2.10 (2025-01-15)
-------------------

1.2.9 (2024-12-11)
------------------

* use gz-physics`#283 <https://github.com/ros-controls/gz_ros2_control/issues/283>`_ to implement joint_states/effort feedback (`#186 <https://github.com/ros-controls/gz_ros2_control/issues/186>`_)
* Contributors: Andreas Bihlmaier

1.2.8 (2024-10-28)
------------------
* Parse `position_proportional_gain` parameter from URDF and update docs (`#393 <https://github.com/ros-controls/gz_ros2_control//issues/393>`_) (`#410 <https://github.com/ros-controls/gz_ros2_control//issues/410>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 8cecc69b7aa698dfd996ae545c186a42b6799d87)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* propagate gazebo remapping and other arguments to the controller node (`#396 <https://github.com/ros-controls/gz_ros2_control//issues/396>`_) (`#397 <https://github.com/ros-controls/gz_ros2_control//issues/397>`_)
  (cherry picked from commit cbdd3a3fc6dcb49072b501984c3294a20872dcd8)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Contributors: mergify[bot]

1.2.6 (2024-07-09)
------------------
* Propagate the node clock and logging interface (`#368 <https://github.com/ros-controls/gz_ros2_control/issues/368>`_) (`#373 <https://github.com/ros-controls/gz_ros2_control/issues/373>`_)
  (cherry picked from commit a1d9bd46fc491c0de35f86f9c14c1620cbcdb037)
  Co-authored-by: Sai Kishor Kothakota <sai.kishor@pal-robotics.com>
* Contributors: mergify[bot]

1.2.5 (2024-07-09)
------------------
* Simplify access for robot description from CM by overriding RM. (`#265 <https://github.com/ros-controls/gz_ros2_control/issues/265>`_) (`#364 <https://github.com/ros-controls/gz_ros2_control/issues/364>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  Co-authored-by: Sai Kishor Kothakota <saisastra3@gmail.com>
  (cherry picked from commit ced470bf20d4e9313f76582eda4a28f7fc6a6a11)
  Co-authored-by: Dr. Denis <denis@stoglrobotics.de>
* Update docs and cleanup member of `GazeboSimROS2ControlPluginPrivate` (`#363 <https://github.com/ros-controls/gz_ros2_control/issues/363>`_) (`#367 <https://github.com/ros-controls/gz_ros2_control/issues/367>`_)
  (cherry picked from commit 9257ad3973e2aebf9756c7a8154efb9673ed1a43)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

1.2.4 (2024-06-02)
------------------
* Don't crash if a wrong config was detected (`#324 <https://github.com/ros-controls/gz_ros2_control/issues/324>`_) (`#331 <https://github.com/ros-controls/gz_ros2_control/issues/331>`_)
  (cherry picked from commit ec1b95893fa933cb3e2cc5341bb65dd621645785)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Harden behavior if a joint is not found in the model (`#325 <https://github.com/ros-controls/gz_ros2_control/issues/325>`_) (`#333 <https://github.com/ros-controls/gz_ros2_control/issues/333>`_)
  * Don't crash if a joint does not exist
  (cherry picked from commit 5d2d5eb6c867875c3c8d00a03cd472eac176e67c)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Changed to use spin instead of spin_once to enable multithreading with MultiThreadedExecutor (`#315 <https://github.com/ros-controls/gz_ros2_control/issues/315>`_) (`#319 <https://github.com/ros-controls/gz_ros2_control/issues/319>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 45035076408e6014cc911e8d4bc169572a25008c)
  Co-authored-by: Takashi Sato <t.sato17123@gmail.com>
* Contributors: mergify[bot]

1.2.3 (2024-05-14)
------------------
* Use Gazebo ROS vendor packages (`#277 <https://github.com/ros-controls/gz_ros2_control/issues/277>`_)
* fixed target of ament_export_libraries (`#295 <https://github.com/ros-controls/gz_ros2_control/issues/295>`_)
* fixed install include (`#294 <https://github.com/ros-controls/gz_ros2_control/issues/294>`_)
* Added parameters robot_param and robot_param_node (`#275 <https://github.com/ros-controls/gz_ros2_control/issues/275>`_) (`#280 <https://github.com/ros-controls/gz_ros2_control/issues/280>`_)
  (cherry picked from commit 53b6c74b02bf85860854a37f429b6e2ecf22a4be)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Rewrite mimic joints (`#276 <https://github.com/ros-controls/gz_ros2_control/issues/276>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Fixed linter (`#264 <https://github.com/ros-controls/gz_ros2_control/issues/264>`_)
* Fix `#259 <https://github.com/ros-controls/gz_ros2_control/issues/259>`_ - `ParameterAlreadyDeclaredException` for parameter `position_proportional_gain` (`#261 <https://github.com/ros-controls/gz_ros2_control/issues/261>`_)
* Contributors: Addisu Z. Taddese, Alejandro Hernández Cordero, Christoph Fröhlich, Patrick Roncagliolo, Takashi Sato, mergify[bot]

1.2.2 (2024-03-21)
------------------
* Fix typo (`#253 <https://github.com/ros-controls/gz_ros2_control/issues/253>`_)
* Fix `#247 <https://github.com/ros-controls/gz_ros2_control/issues/247>`_ (`#248 <https://github.com/ros-controls/gz_ros2_control/issues/248>`_)
* Reset Gazebo with initial joint positions and velocities (`#241 <https://github.com/ros-controls/gz_ros2_control/issues/241>`_)
* Use portable versio for usleep (`#237 <https://github.com/ros-controls/gz_ros2_control/issues/237>`_)
* Fix crashing due to an invalid parameter in the initial value (`#233 <https://github.com/ros-controls/gz_ros2_control/issues/233>`_)
* Contributors: Alejandro Hernández Cordero, Graziato Davide, Ruddick Lawrence, Stephanie Eng

1.2.1 (2024-01-24)
------------------
* Load the URDF to the resource_manager before parsing it to CM (`#222 <https://github.com/ros-controls/gz_ros2_control/issues/222>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Use own implementation of `stod()` (`#220 <https://github.com/ros-controls/gz_ros2_control/issues/220>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Christoph Fröhlich, Sai Kishor Kothakota

1.2.0 (2024-01-04)
------------------
* Add controller name parameter (backport `#212 <https://github.com/ros-controls/gz_ros2_control/issues/212>`_) (`#216 <https://github.com/ros-controls/gz_ros2_control/issues/216>`_)
  Co-authored-by: Jakub Delicat <109142865+delihus@users.noreply.github.com>
  Co-authored-by: Alejandro Hernandez Cordero <ahcorde@gmail.com>
* Add hold_joints parameter (`#213 <https://github.com/ros-controls/gz_ros2_control/issues/213>`_)
* Fix stuck passive joints (`#184 <https://github.com/ros-controls/gz_ros2_control/issues/184>`_)
  * Fix stuck passive joints
  * Update comment
  * Fix variable naming
  ---------
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Support Harmonic (`#185 <https://github.com/ros-controls/gz_ros2_control/issues/185>`_)
* Contributors: Alejandro Hernández Cordero, Johannes Huemer, mergify[bot]

1.1.2 (2023-08-23)
------------------
* Catch pluginlib exceptions (`#175 <https://github.com/ros-controls/gz_ros2_control/issues/175>`_)
* Contributors: Alejandro Hernández Cordero

1.1.1 (2023-07-13)
------------------
* Remove plugin export from ROS 1 (`#158 <https://github.com/ros-controls/gz_ros2_control//issues/158>`_)
* Fixed default gazebo version in CMakeLists.txt (`#156 <https://github.com/ros-controls/gz_ros2_control//issues/156>`_)
* Compile master with iron and rolling (`#142 <https://github.com/ros-controls/gz_ros2_control//issues/142>`_)
* Update package.xml (`#141 <https://github.com/ros-controls/gz_ros2_control//issues/141>`_)
* Contributors: Alejandro Hernández Cordero, Bence Magyar, Christoph Fröhlich

1.1.0 (2023-05-23)
------------------
* Fixed segmentation fault with logger (`#136 <https://github.com/ros-controls/gz_ros2_control/issues/136>`_)
* Disable ROS signal handlers (`#129 <https://github.com/ros-controls/gz_ros2_control/issues/129>`_)
* Contributors: Alejandro Hernández Cordero, Carlo Rizzardo

1.0.0 (2023-03-28)
------------------
* Context and Namespace Handling for Multi-Robot Sim (`#128 <https://github.com/ros-controls/gz_ros2_control/issues/128>`_)
* Install include directory since it is exported (`#127 <https://github.com/ros-controls/gz_ros2_control/issues/127>`_)
* Renamed ign to gz (`#67 <https://github.com/ros-controls/gz_ros2_control/issues/67>`_)
* Contributors: Alejandro Hernández Cordero, Roni Kreinin, Tim Clephas

0.6.1 (2023-02-07)
------------------
* Various bug fixes (`#114 <https://github.com/ros-controls/gz_ros2_control/issues/114>`_)
* Contributors: AndyZe

0.6.0 (2023-01-06)
------------------
* Fix API braking of hardware plugin name. (`#108 <https://github.com/ros-controls/gz_ros2_control/issues/108>`_)
* Galactic to master -- Merge pull request `#103 <https://github.com/ros-controls/gz_ros2_control/issues/103>`_ from ros-controls/ahcorde/galactic_to_main_25_11_2022
* Force setting use_sim_time parameter when using plugin. (`#100 <https://github.com/ros-controls/gz_ros2_control/issues/100>`_) (`#102 <https://github.com/ros-controls/gz_ros2_control/issues/102>`_)
* Force setting use_sim_time parameter when using plugin. (`#100 <https://github.com/ros-controls/gz_ros2_control/issues/100>`_)
* Enable loading params from multiple yaml files (`#94 <https://github.com/ros-controls/gz_ros2_control/issues/94>`_)
* Add support for mimic joints. (`#33 <https://github.com/ros-controls/gz_ros2_control/issues/33>`_)
* Set right initial velocity (`#81 <https://github.com/ros-controls/gz_ros2_control/issues/81>`_)
* Contributors: Alejandro Hernández Cordero, Denis Štogl, Lovro Ivanov

0.5.0 (2022-08-09)
------------------
* Fix setting initial values if command interfaces are not defined. (`#73 <https://github.com/ros-controls/gz_ros2_control/issues/73>`_)
* activated all hardware by default and improved variable naming (`#74 <https://github.com/ros-controls/gz_ros2_control/issues/74>`_)
* Implemented perform_command_mode_switch override in GazeboSystem (`#76 <https://github.com/ros-controls/gz_ros2_control/issues/76>`_)
* Remove warnings (`#72 <https://github.com/ros-controls/gz_ros2_control/issues/72>`_)
* change component name for ignition (`#69 <https://github.com/ros-controls/gz_ros2_control/issues/69>`_)
* Added logic for activating hardware interfaces (`#68 <https://github.com/ros-controls/gz_ros2_control/issues/68>`_)
* Merge branch 'foxy' into ahcorde/foxy_to_galactic_27_05_2022
* Adapt to ROS 2 Humble
* typo in citadel name (`#51 <https://github.com/ros-controls/gz_ros2_control/issues/51>`_)
* ros2_control is now having usings under its namespace. (`#43 <https://github.com/ros-controls/gz_ros2_control/issues/43>`_)
* Fix default ign gazebo version Rolling (`#45 <https://github.com/ros-controls/gz_ros2_control/issues/45>`_)
* Fix ignition version in package.xml - Rolling (`#41 <https://github.com/ros-controls/gz_ros2_control/issues/41>`_)
* Add support for initial_values for hardware interfaces when starting simulation. (`#27 <https://github.com/ros-controls/gz_ros2_control/issues/27>`_)
* Contributors: Alejandro Hernández Cordero, Denis Štogl, Guillaume Beuzeboc, Tianyu Li

0.4.1 (2022-06-06)
------------------
* Remove URDF dependency (`#56 <https://github.com/ignitionrobotics/ign_ros2_control/issues/56>`_)
* typo in citadel name (`#54 <https://github.com/ignitionrobotics/ign_ros2_control/issues/54>`_)
* Contributors: Alejandro Hernández Cordero, Guillaume Beuzeboc, ahcorde

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
