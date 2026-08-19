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

import rclpy
from sensor_msgs.msg import JointState


def wait_for_pendulum_steady_state(
    node,
    joint_name='cart_to_pendulum',
    vel_eps=0.05,
    eff_eps=0.05,
    timeout_ns=int(10e9),
    stable_required=5
):
    """
    Wait until the pendulum joint reaches steady-state.

    Startup is not deterministic unless the initial value is captured
    before physics. See issue #836 for reference.
    """
    last_msg = None

    def callback(msg):
        nonlocal last_msg
        last_msg = msg

    sub = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10
    )

    start = node.get_clock().now().nanoseconds
    stable_count = 0

    pos = None
    vel = None
    eff = None

    while node.get_clock().now().nanoseconds - start < timeout_ns:
        rclpy.spin_once(node, timeout_sec=0.1)

        if last_msg is None:
            continue

        if joint_name not in last_msg.name:
            continue

        idx = last_msg.name.index(joint_name)
        vel = last_msg.velocity[idx]
        eff = last_msg.effort[idx]
        pos = last_msg.position[idx]

        # Convergence condition with hysteresis
        if abs(vel) < vel_eps and abs(eff) < eff_eps:
            stable_count += 1
            if stable_count >= stable_required:
                node.destroy_subscription(sub)
                return pos, vel, eff
        else:
            stable_count = 0

    node.destroy_subscription(sub)
    raise AssertionError('Pendulum did not converge within timeout')
