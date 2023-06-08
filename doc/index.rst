:github_url: https://github.com/ros-controls/gz_ros2_control/blob/{REPOS_FILE_BRANCH}/doc/index.rst

.. _gz_ros2_control:

=====================
gz_ros2_control
=====================

This is a ROS 2 package for integrating the *ros2_control* controller architecture with the `Gazebo <https://gazebosim.org/home>`__ simulator.

This package provides a Gazebo-Sim system plugin which instantiates a *ros2_control* controller manager and connects it to a Gazebo model.

.. image:: img/gz_ros2_control.gif
  :alt: Cart

.. image:: img/diff_drive.gif
  :alt: DiffBot

Usage
======


Modifying or building your own
---------------------------------

.. code-block:: shell

  cd Dockerfile
  docker build -t gz_ros2_control .

To run the demo
---------------------------------

1. Using Docker

  Docker allows us to run the demo without the GUI if configured properly. The following command runs the demo without the GUI:

  .. code-block:: shell

    docker run -it --rm --name gz_ros2_control_demo --net host gz_ros2_control ros2 launch gz_ros2_control_demos cart_example_position.launch.py gui:=false

  Then on your local machine, you can run the Gazebo client:

  .. code-block:: shell

    ign gazebo -g


2. Using Rocker

  To run the demo with GUI we are going to use `rocker <https://github.com/osrf/rocker/>`__ which is a tool to run docker
  images with customized local support injected for things like nvidia support. Rocker also supports user id specific files for cleaner
  mounting file permissions. You can install this tool with the following `instructions <https://github.com/osrf/rocker/#installation>`__. (make sure you meet all of the `prerequisites <https://github.com/osrf/rocker/#prerequisites>`__.

  The following command will launch Gazebo:

  .. code-block:: xml

    rocker --x11 --nvidia --name gz_ros2_control_demo gz_ros2_control:latest

  The following commands allow the cart to be moved along the rail:

  .. code-block:: xml

    docker exec -it gz_ros2_control_demo bash
    source /home/ros2_ws/install/setup.bash
    ros2 run gz_ros2_control_demos example_position


Add ros2_control tag to a URDF
==========================================

Simple setup
-----------------------------------------------------------

To use *ros2_control* with your robot, you need to add some additional elements to your URDF.
You should include the tag ``<ros2_control>`` to access and control the robot interfaces. We should
include:

* a specific ``<plugin>`` for our robot
* ``<joint>`` tag including the robot controllers: commands and states.

.. code-block:: xml

  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    <joint name="slider_to_cart">
      <command_interface name="effort">
        <param name="min">-1000</param>
        <param name="max">1000</param>
      </command_interface>
      <state_interface name="position">
        <param name="initial_value">1.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>
  </ros2_control>

Using mimic joints in simulation
-----------------------------------------------------------

To use ``mimic`` joints in *gz_ros2_control* you should define its parameters to your URDF.
We should include:

* ``<mimic>`` tag to the mimicked joint `detailed manual <https://wiki.ros.org/urdf/XML/joint>`__
* ``mimic`` and ``multiplier`` parameters to joint definition in ``<ros2_control>`` tag

.. code-block:: xml

  <joint name="left_finger_joint" type="prismatic">
    <mimic joint="right_finger_joint"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.48 1" rpy="0.0 0.0 3.1415926535"/>
    <parent link="base"/>
    <child link="finger_left"/>
    <limit effort="1000.0" lower="0" upper="0.38" velocity="10"/>
  </joint>

.. code-block:: xml

  <joint name="left_finger_joint">
    <param name="mimic">right_finger_joint</param>
    <param name="multiplier">1</param>
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>


Add the gz_ros2_control plugin
==========================================

In addition to the *ros2_control* tags, a Gazebo plugin needs to be added to your URDF that
actually parses the *ros2_control* tags and loads the appropriate hardware interfaces and
controller manager. By default the *gz_ros2_control* plugin is very simple, though it is also
extensible via an additional plugin architecture to allow power users to create their own custom
robot hardware interfaces between *ros2_control* and Gazebo.

.. code-block:: xml

  <gazebo>
      <plugin filename="libgz_ros2_control-system.so" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
        <robot_param>robot_description</robot_param>
        <robot_param_node>robot_state_publisher</robot_param_node>
        <parameters>$(find gz_ros2_control_demos)/config/cartpole_controller.yaml</parameters>
      </plugin>
  </gazebo>

The *gz_ros2_control* ``<plugin>`` tag also has the following optional child elements:

* ``<parameters>``: YAML file with the configuration of the controllers

Default gz_ros2_control Behavior
-----------------------------------------------------------

By default, without a ``<plugin>`` tag, *gz_ros2_control* will attempt to get all of the information it needs to interface with a ros2_control-based controller out of the URDF. This is sufficient for most cases, and good for at least getting started.

The default behavior provides the following ros2_control interfaces:

* hardware_interface::JointStateInterface
* hardware_interface::EffortJointInterface
* hardware_interface::VelocityJointInterface

Advanced: custom gz_ros2_control Simulation Plugins
-----------------------------------------------------------

The *gz_ros2_control* Gazebo plugin also provides a pluginlib-based interface to implement custom interfaces between Gazebo and *ros2_control* for simulating more complex mechanisms (nonlinear springs, linkages, etc).

These plugins must inherit the ``gz_ros2_control::GazeboSimSystemInterface``, which implements a simulated *ros2_control*
``hardware_interface::SystemInterface``. SystemInterface provides API-level access to read and command joint properties.

The respective GazeboSimSystemInterface sub-class is specified in a URDF model and is loaded when the
robot model is loaded. For example, the following XML will load the default plugin:

.. code-block:: xml

  <ros2_control name="GazeboSimSystem" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>
    ...
  <ros2_control>
  <gazebo>
    <plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin" filename="libgz_ros2_control-system">
      ...
    </plugin>
  </gazebo>

Set up controllers
-----------------------------------------------------------

Use the tag ``<parameters>`` inside ``<plugin>`` to set the YAML file with the controller configuration
and use the tag ``<controller_manager_prefix_node_name>`` to set the controller manager node name.

.. code-block:: xml

  <gazebo>
    <plugin name="gz_ros2_control::GazeboSimROS2ControlPlugin" filename="libgz_ros2_control-system">
      <parameters>$(find gz_ros2_control_demos)/config/cartpole_controller.yaml</parameters>
      <controller_manager_prefix_node_name>controller_manager</controller_manager_prefix_node_name>
    </plugin>
  <gazebo>

This controller publishes the state of all resources registered to a
``hardware_interface::StateInterface`` to a topic of type ``sensor_msgs/msg/JointState``.
The following is a basic configuration of the controller.

.. code-block:: yaml

  joint_state_controller:
    ros__parameters:
      type: joint_state_controller/JointStateController

This controller creates an action called ``/cart_pole_controller/follow_joint_trajectory`` of type ``control_msgs::action::FollowJointTrajectory``.

.. code-block:: yaml

  cart_pole_controller:
    ros__parameters:
      type: joint_trajectory_controller/JointTrajectoryController
      joints:
        - slider_to_cart
      write_op_modes:
        - slider_to_cart


gz_ros2_control_demos
==========================================

There are some examples in the *gz_ros2_control_demos* package. These examples allow to launch a cart in a 30 meter rail.

You can run some of the example configurations by running the following commands:

.. code-block:: shell

  ros2 launch gz_ros2_control_demos cart_example_position.launch.py
  ros2 launch gz_ros2_control_demos cart_example_velocity.launch.py
  ros2 launch gz_ros2_control_demos cart_example_effort.launch.py
  ros2 launch gz_ros2_control_demos diff_drive_example.launch.py
  ros2 launch gz_ros2_control_demos tricycle_drive_example.launch.py

When the Gazebo world is launched, you can run some of the following commands to move the cart.

.. code-block:: shell

  ros2 run gz_ros2_control_demos example_position
  ros2 run gz_ros2_control_demos example_velocity
  ros2 run gz_ros2_control_demos example_effort
  ros2 run gz_ros2_control_demos example_diff_drive
  ros2 run gz_ros2_control_demos example_tricycle_drive

The following example shows parallel gripper with mimic joint:

.. code-block:: shell

  ros2 launch gz_ros2_control_demos gripper_mimic_joint_example.launch.py


Send example commands:


.. code-block:: shell

  ros2 run gz_ros2_control_demos example_gripper
