^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ign_ros2_control_demos
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
-----------

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
