^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package gz_ros2_control_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.16 (2025-09-29)
-------------------

1.2.15 (2025-08-18)
-------------------

1.2.14 (2025-07-09)
-------------------
* Provide force-torque sensor data through gz_system to controller_manager - fixes to original PR  (backport `#610 <https://github.com/ros-controls/gz_ros2_control/issues/610>`_) (`#624 <https://github.com/ros-controls/gz_ros2_control/issues/624>`_)
* Bump CMake minimum version (`#601 <https://github.com/ros-controls/gz_ros2_control/issues/601>`_) (`#603 <https://github.com/ros-controls/gz_ros2_control/issues/603>`_)
* Contributors: mergify[bot]

1.2.13 (2025-05-23)
-------------------

1.2.12 (2025-04-04)
-------------------

1.2.11 (2025-02-19)
-------------------
* Change the order of tests to fix timing issues (`#498 <https://github.com/ros-controls/gz_ros2_control/issues/498>`_) (`#501 <https://github.com/ros-controls/gz_ros2_control/issues/501>`_)
* Update cart demos to use joint_trajectory_controller (`#486 <https://github.com/ros-controls/gz_ros2_control/issues/486>`_) (`#489 <https://github.com/ros-controls/gz_ros2_control/issues/489>`_)
* Use files from demos for testing (`#485 <https://github.com/ros-controls/gz_ros2_control/issues/485>`_) (`#487 <https://github.com/ros-controls/gz_ros2_control/issues/487>`_)
* Fix the test criterion (`#481 <https://github.com/ros-controls/gz_ros2_control/issues/481>`_) (`#483 <https://github.com/ros-controls/gz_ros2_control/issues/483>`_)
* Contributors: mergify[bot]

1.2.10 (2025-01-15)
-------------------

1.2.9 (2024-12-11)
------------------
* Add missing bridge for simulation time (`#443 <https://github.com/ros-controls/gz_ros2_control/issues/443>`_) (`#445 <https://github.com/ros-controls/gz_ros2_control/issues/445>`_)
  (cherry picked from commit 301ca580d0772b9952579a783632500eeca7e53b)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

1.2.8 (2024-10-28)
------------------
* Use spawner with `--params-file` argument instead of cli verbs (`#399 <https://github.com/ros-controls/gz_ros2_control//issues/399>`_) (`#409 <https://github.com/ros-controls/gz_ros2_control//issues/409>`_)
  Co-authored-by: Alejandro Hernández Cordero <ahcorde@gmail.com>
  (cherry picked from commit 30e67055bcd76e198805926997d01fefcc347255)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Contributors: mergify[bot]

1.2.6 (2024-07-09)
------------------

1.2.5 (2024-07-09)
------------------

1.2.4 (2024-06-02)
------------------

1.2.3 (2024-05-14)
------------------
* Use Gazebo ROS vendor packages (`#277 <https://github.com/ros-controls/gz_ros2_control/issues/277>`_)
* Contributors: Addisu Z. Taddese

1.2.2 (2024-03-21)
------------------

1.2.1 (2024-01-24)
------------------
* Include testing packages on CI (`#223 <https://github.com/ros-controls/gz_ros2_control/issues/223>`_)
* Contributors: Alejandro Hernández Cordero

1.2.0 (2024-01-04)
------------------
* Rename cartpole with cart (`#214 <https://github.com/ros-controls/gz_ros2_control/issues/214>`_)
  Co-authored-by: Christoph Fröhlich <christophfroehlich@users.noreply.github.com>
* Support Harmonic (`#185 <https://github.com/ros-controls/gz_ros2_control/issues/185>`_)
* Contributors: Alejandro Hernández Cordero

1.1.1 (2023-07-13)
------------------
* Run end to end test in CI (`#152 <https://github.com/ros-controls/gz_ros2_control//issues/152>`_)
* Add test to check position controller (`#134 <https://github.com/ros-controls/gz_ros2_control//issues/134>`_)
* Contributors: Alejandro Hernández Cordero
