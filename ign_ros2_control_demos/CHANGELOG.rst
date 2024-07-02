^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ign_ros2_control_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.9 (2024-07-02)
------------------
* [Backport humble]  Ackermann steering example (`#352 <https://github.com/ros-controls/gz_ros2_control/issues/352>`_)
  Co-authored-by: huzaifa <84243533+huzzu7@users.noreply.github.com>
* Fixed pendulum examples (`#353 <https://github.com/ros-controls/gz_ros2_control/issues/353>`_)
* Rename variable in launch file (`#327 <https://github.com/ros-controls/gz_ros2_control/issues/327>`_) (`#338 <https://github.com/ros-controls/gz_ros2_control/issues/338>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit cd0b002c49e71be459f4e9f0a063b97fed195b28)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* added color definitions (`#310 <https://github.com/ros-controls/gz_ros2_control/issues/310>`_) (`#312 <https://github.com/ros-controls/gz_ros2_control/issues/312>`_)
  (cherry picked from commit 7cb6fd901f373d6fcfa75ef23e43c6b9d7b186a7)
  Co-authored-by: Reza Kermani <kermani.areza@gmail.com>
* Contributors: Alejandro Hernández Cordero, mergify[bot]

0.7.8 (2024-05-14)
------------------
* Update pendulum-example  (`#301 <https://github.com/ros-controls/gz_ros2_control/issues/301>`_) (`#303 <https://github.com/ros-controls/gz_ros2_control/issues/303>`_)
  * Change initial pose of pendulum
  * Make position and effort version of pendulum equal
  (cherry picked from commit 1e7721409e5e3d2c583868353a09929ca37bf860)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Add cart-pole demo (`#289 <https://github.com/ros-controls/gz_ros2_control/issues/289>`_) (`#290 <https://github.com/ros-controls/gz_ros2_control/issues/290>`_)
  (cherry picked from commit 27af2108e77420dc46c83ac31658fccb67e33911)
  # Conflicts:
  #	ign_ros2_control_demos/launch/pendulum_example_effort.launch.py
  #	ign_ros2_control_demos/launch/pendulum_example_position.launch.py
  #	ign_ros2_control_demos/urdf/test_pendulum_effort.xacro.urdf
  #	ign_ros2_control_demos/urdf/test_pendulum_position.xacro.urdf
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

0.7.7 (2024-04-09)
------------------

0.7.6 (2024-03-21)
------------------
* Add `ros_gz_bridge` as dependency to demos (backport `#256 <https://github.com/ros-controls/gz_ros2_control/issues/256>`_) (`#257 <https://github.com/ros-controls/gz_ros2_control/issues/257>`_)
  * Add dep (`#256 <https://github.com/ros-controls/gz_ros2_control/issues/256>`_)
  (cherry picked from commit b35100db16e80ffb574c0266321800e2197136c3)
  # Conflicts:
  #	ign_ros2_control_demos/package.xml
  * fixed merge
  ---------
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: mergify[bot]

0.7.4 (2024-01-24)
------------------

0.7.3 (2024-01-04)
------------------
* Removed cartpole files
* Rename cartpole with cart (backport `#214 <https://github.com/ros-controls/gz_ros2_control/issues/214>`_) (`#217 <https://github.com/ros-controls/gz_ros2_control/issues/217>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, mergify[bot]

0.7.2 (2024-01-04)
------------------
* Update diff_drive_example.launch.py (`#207 <https://github.com/ros-controls/gz_ros2_control/issues/207>`_)
* Cleanup controller config (backport `#180 <https://github.com/ros-controls/gz_ros2_control/issues/180>`_) (`#181 <https://github.com/ros-controls/gz_ros2_control/issues/181>`_)
  * Cleanup controller config (`#180 <https://github.com/ros-controls/gz_ros2_control/issues/180>`_)
  (cherry picked from commit 223174515a64008b2ffef5b730c4c61cc78ff8bc)
* Contributors: Jakub Delicat, mergify[bot]

0.7.1 (2023-08-23)
------------------
* Set C++ version to 17 (`#171 <https://github.com/ros-controls/gz_ros2_control/issues/171>`_) (`#174 <https://github.com/ros-controls/gz_ros2_control/issues/174>`_)
  (cherry picked from commit 353b5a001733612241049557a907f835b53f35ac)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Update diff_drive_controller_velocity.yaml (`#172 <https://github.com/ros-controls/gz_ros2_control/issues/172>`_) (`#173 <https://github.com/ros-controls/gz_ros2_control/issues/173>`_)
  (cherry picked from commit 7559d9bfe9dc836a7fc4907cc234554600e54b14)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
* Contributors: Alejandro Hernández Cordero, mergify[bot]

0.5.0 (2023-05-23)
------------------
* Clean shutdown example position (`#135 <https://github.com/ros-controls/gz_ros2_control/issues/135>`_) (`#139 <https://github.com/ros-controls/gz_ros2_control/issues/139>`_)
* Fixed /clock with gz_ros2_bridge (`#137 <https://github.com/ros-controls/gz_ros2_control/issues/137>`_) (`#138 <https://github.com/ros-controls/gz_ros2_control/issues/138>`_)
* Contributors: mergify[bot]

0.4.4 (2023-03-28)
------------------
* Merge pull request `#121 <https://github.com/ros-controls/gz_ros2_control/issues/121>`_ from livanov93/port-master-to-humble
* Fix gripper mimic joint example.
* Pre-commit fix for tricycle configuration.
* Replace ros_ign_gazebo with ros_gz_sim
* use ros_gz_sim
* Fix Docker entrypoint and add launch CLI to dependencites (`#84 <https://github.com/ros-controls/gz_ros2_control/issues/84>`_)
* Add support for mimic joints. (`#33 <https://github.com/ros-controls/gz_ros2_control/issues/33>`_)
* Add tricycle demo (`#80 <https://github.com/ros-controls/gz_ros2_control/issues/80>`_)
* Fix setting initial values if command interfaces are not defined. (`#73 <https://github.com/ros-controls/gz_ros2_control/issues/73>`_)
* fix demo launch (`#75 <https://github.com/ros-controls/gz_ros2_control/issues/75>`_)
* Contributors: Alejandro Hernández Cordero, Andrej Orsula, Bence Magyar, Denis Štogl, Ian Chen, Krzysztof Wojciechowski, Lovro Ivanov, Maciej Bednarczyk, Polgár András, Tony Najjar

0.4.3 (2023-02-16)
------------------
* Add tricycle example to the `humble` branch `#119 <https://github.com/ros-controls/gz_ros2_control/issues/119>`_ from azazdeaz/humble
* Replace ros_ign_gazebo with ros_gz_sim
* Add tricycle demo (`#80 <https://github.com/ros-controls/gz_ros2_control/issues/80>`_)
* Fix example demos in humble branch `#118 <https://github.com/ros-controls/gz_ros2_control/issues/118>`_ from iche033/iche033/fix_humble_demos
* use ros_gz_sim
* fix demo launch (`#75 <https://github.com/ros-controls/gz_ros2_control/issues/75>`_)
* Adjust URLs (`#65 <https://github.com/ros-controls/gz_ros2_control/issues/65>`_)
* ign_ros2_control_demos: Install urdf dir (`#61 <https://github.com/ros-controls/gz_ros2_control/issues/61>`_)
* Remove URDF dependency (`#56 <https://github.com/ros-controls/gz_ros2_control/issues/56>`_)
* Use Ubuntu Jammy in CI (`#47 <https://github.com/ros-controls/gz_ros2_control/issues/47>`_)
* Add support for initial_values for hardware interfaces when starting simulation. (`#27 <https://github.com/ros-controls/gz_ros2_control/issues/27>`_)
* Contributors: Alejandro Hernández Cordero, Andrej Orsula, Bence Magyar, Denis Štogl, Maciej Bednarczyk, ahcorde

0.4.0 (2022-03-18)
------------------

0.3.0 (2022-03-16)
------------------

0.2.0 (2022-02-17)
------------------
* Merge pull request `#36 <https://github.com/ignitionrobotics/ign_ros2_control/issues/36>`_ from ignitionrobotics/ahcorde/foxy_to_galactic
  Foxy -> Galactic
* Fixed galactic dependency
* Merge remote-tracking branch 'origin/foxy' into ahcorde/foxy_to_galactic
* Contributors: Alejandro Hernández Cordero

0.1.2 (2022-02-14)
------------------
* Updated docs and renamed diff drive launch file (`#32 <https://github.com/ignitionrobotics/ign_ros2_control/issues/32>`_)
  Co-authored-by: Denis Štogl <denis@stogl.de>
* Added Diff drive example (`#28 <https://github.com/ignitionrobotics/ign_ros2_control/issues/28>`_)
* Contributors: Alejandro Hernández Cordero

0.1.1 (2022-01-07)
------------------
* Change package names from ignition\_ to ign\_ (`#19 <https://github.com/ignitionrobotics/ign_ros2_control/issues/19>`_)
  * Change package names from ignition\_ to ign\_
* Added missing dependencies to package.xml (`#18 <https://github.com/ignitionrobotics/ign_ros2_control/pull/21>`_)
* Contributors: Alejandro Hernández Cordero

0.1.0 (2022-01-05)
------------------
* Ignition ros2 control (`#1 <https://github.com/ignitionrobotics/ign_ros2_control/issues/1>`_)
  Co-authored-by: ahcorde <ahcorde@gmail.com>
  Co-authored-by: Louise Poubel <louise@openrobotics.org>
  Co-authored-by: Vatan Aksoy Tezer <vatan@picknik.ai>
* Contributors: Alejandro Hernández Cordero, Louise Poubel, Vatan Aksoy Tezer
