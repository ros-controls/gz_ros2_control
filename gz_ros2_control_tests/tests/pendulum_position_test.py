#!/usr/bin/env python3
# Copyright 2025 ros2_control Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import (
    check_controllers_running,
    check_if_js_published,
    check_node_running
)
from controller_manager_msgs.srv import ListControllers
from test_utils import (
    read_joint_state,
    observe_joint_state_window,
    assert_joint_initial_position,
    wait_for_pendulum_steady_state,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import launch_testing
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from launch_testing_ros import WaitForTopics
import psutil
import pytest
import rclpy
from rosgraph_msgs.msg import Clock


# This function specifies the processes to be run for our test
@pytest.mark.rostest
def generate_test_description():
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gz_ros2_control_demos'),
                'launch/pendulum_example_position.launch.py',
            )
        ),
        launch_arguments={'gz_args': '--headless-rendering -s'}.items(),
    )

    return LaunchDescription([
        launch_include,
        KeepAliveProc(),
        ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        for proc in psutil.process_iter():
            # check whether the process name matches
            if proc.name() == 'ruby' or 'gz sim' in proc.name():
                # up to version 9 of gz-sim
                proc.kill()
            if 'gz-sim' in proc.name():
                # from version 10 of gz-sim
                proc.kill()
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_controller_manager(self, timeout=10.0):
        cli = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        end = self.node.get_clock().now().nanoseconds + int(timeout * 1e9)

        while not cli.wait_for_service(timeout_sec=0.1):
            if self.node.get_clock().now().nanoseconds > end:
                self.fail('controller_manager service not available in time')

    def test_node_start(self, proc_output):
        check_node_running(self.node, 'robot_state_publisher')

    def test_clock(self):
        topic_list = [('/clock', Clock)]
        with WaitForTopics(topic_list, timeout=10.0):
            print('/clock is receiving messages!')

    def test_check_if_msgs_published(self):
        check_if_js_published(
            '/joint_states',
            ['slider_to_cart', 'cart_to_pendulum'],
        )

    def _observe_pendulum_motion(
        self,
        baseline_position,
        movement_tolerance=2.0,
    ):
        joint_name = 'cart_to_pendulum'

        def stop_when(samples):
            if not samples:
                return False

            position, _, _ = samples[-1]
            displacement = abs(position - baseline_position)
            return displacement > movement_tolerance

        samples = observe_joint_state_window(
            self.node,
            joint_name,
            duration=5.0,
            sample_period=0.05,
            stop_when=stop_when,
        )

        maximum_displacement = 0.0
        for position, _, _ in samples:
            maximum_displacement = max(
                maximum_displacement,
                abs(position - baseline_position),
            )

        self.assertTrue(
            samples,
            f"No position samples were received for '{joint_name}' while the cart was moving.",
        )

        return maximum_displacement

    def _check_pendulum_initial_motion(self):
        """
        Verify that physics has affected a pendulum spawned outside equilibrium.

        The first state observed by this test is not guaranteed to be the first
        state produced by the simulation.
        """
        joint_name = 'cart_to_pendulum'

        urdf_default_position = 1.57
        position_tolerance = 0.01
        velocity_tolerance = 0.01
        effort_tolerance = 0.01

        position, velocity, effort = read_joint_state(self.node, joint_name)

        at_urdf_default = (
            abs(position - urdf_default_position)
            < position_tolerance
        )
        at_zero_default = abs(position) < position_tolerance
        apparently_static = (
            abs(velocity) < velocity_tolerance and
            abs(effort) < effort_tolerance
        )

        self.assertFalse(
            (at_urdf_default or at_zero_default) and apparently_static,
            (
                "The first observed pendulum state is effectively static and still looks "
                f"like an uninitialized/default state: "
                f"position={position:.4f}, velocity={velocity:.4f}, effort={effort:.4f}"
            )
        )

        print("Pendulum correctly initialized")

    # ---------------------------------------------------------
    # Main test
    # ---------------------------------------------------------
    def test_pendulum_joint_position(self, launch_service, proc_info, proc_output):
        # 1. Check the configured initial slider position.
        assert_joint_initial_position(self.node, 'slider_to_cart', 1.0)

        # 2. Verify that physics affected the pendulum after spawning.
        self._check_pendulum_initial_motion()

        # 3. Establish a stable baseline before measuring cart-induced motion
        position_before, velocity_before, effort_before = (
            wait_for_pendulum_steady_state(self.node)
        )

        print(
            'Stable baseline established: '
            f'position={position_before:.4f}, '
            f'velocity={velocity_before:.4f}, '
            f'effort={effort_before:.4f}'
        )

        # 4. Wait for controller_manager and verify the controllers.
        self._wait_for_controller_manager()

        controller_names = [
            'joint_trajectory_controller',
            'joint_state_broadcaster',
        ]
        check_controllers_running(self.node, controller_names)

        # 5. Start the process that moves the cart.
        process_action = Node(
            package='gz_ros2_control_demos',
            executable='example_position',
            output='screen',
        )

        movement_tolerance = 2.0  # radians

        with launch_testing.tools.launch_process(
            launch_service, process_action, proc_info, proc_output,
        ):
            maximum_displacement = self._observe_pendulum_motion(
                baseline_position=position_before,
                movement_tolerance=movement_tolerance,
            )

            proc_info.assertWaitForShutdown(
                process=process_action,
                timeout=300
            )

            launch_testing.asserts.assertExitCodes(
                proc_info,
                process=process_action,
                allowable_exit_codes=[0],
            )

        self.assertGreater(
            maximum_displacement,
            movement_tolerance,
            "Moving the cart did not induce sufficient pendulum motion: "
            f"maximum displacement={maximum_displacement:.4f}, "
            f"minimum expected={movement_tolerance:.4f}.",
        )
