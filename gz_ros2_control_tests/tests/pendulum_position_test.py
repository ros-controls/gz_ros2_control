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

    return LaunchDescription([launch_include, KeepAliveProc(), ReadyToTest()])


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

    # ---------------------------------------------------------
    # Helper: check initial pendulum position BEFORE any motion
    # ---------------------------------------------------------
    def _check_initial_cart_position(self):
        from sensor_msgs.msg import JointState
        msg = None

        def callback(m):
            nonlocal msg
            msg = m

        sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            callback,
            10
        )

        end_time = self.node.get_clock().now().nanoseconds + int(10e9)
        while msg is None and self.node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)

        self.assertIsNotNone(msg, 'No joint_state message received')
        self.assertIn('cart_to_pendulum', msg.name)

        joint_idx = msg.name.index('cart_to_pendulum')
        expected_initial_value = 1.57
        actual_value = msg.position[joint_idx]

        self.assertAlmostEqual(
            actual_value,
            expected_initial_value,
            places=2,
            msg=f'Initial position mismatch: expected {expected_initial_value}, got {actual_value}'
        )

        print(f'Initial value verified: {actual_value} ≈ {expected_initial_value}')

    # ---------------------------------------------------------
    # Main test
    # ---------------------------------------------------------
    def test_arm(self, launch_service, proc_info, proc_output):

        # 1) Check initial position BEFORE any motion
        self._check_initial_cart_position()

        # 2) Wait for controller_manager to be ready
        self._wait_for_controller_manager()

        # 3) Check controllers
        cnames = [
            'joint_trajectory_controller',
            'joint_state_broadcaster',
        ]
        check_controllers_running(self.node, cnames)

        # 4) Launch the node that moves the joint
        proc_action = Node(
            package='gz_ros2_control_demos',
            executable='example_position',
            output='screen',
        )

        with launch_testing.tools.launch_process(
            launch_service, proc_action, proc_info, proc_output
        ):
            proc_info.assertWaitForShutdown(process=proc_action, timeout=300)
            launch_testing.asserts.assertExitCodes(
                proc_info,
                process=proc_action,
                allowable_exit_codes=[0]
            )
