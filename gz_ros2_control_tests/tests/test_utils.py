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


def read_joint_state(node, joint_name, timeout=10.0):
    joint_state = None

    def callback(message):
        nonlocal joint_state

        if joint_name not in message.name:
            return

        index = message.name.index(joint_name)

        if (
            index >= len(message.position)
            or index >= len(message.velocity)
            or index >= len(message.effort)
        ):
            return

        joint_state = (
            message.position[index],
            message.velocity[index],
            message.effort[index],
        )

    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10,
    )

    end_time = node.get_clock().now().nanoseconds + int(timeout * 1e9)

    try:
        while joint_state is None and node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_subscription(subscription)

    if joint_state is None:
        raise AssertionError(
            f"No complete joint state was received for '{joint_name}'."
        )

    return joint_state


def observe_joint_state_window(
    node,
    joint_name,
    duration=5.0,
    sample_period=0.1,
    stop_when=None,
):
    samples = []

    def callback(message):
        if joint_name not in message.name:
            return

        index = message.name.index(joint_name)

        if (
            index >= len(message.position)
            or index >= len(message.velocity)
            or index >= len(message.effort)
        ):
            return

        samples.append((
            message.position[index],
            message.velocity[index],
            message.effort[index],
        ))

        if stop_when is not None and stop_when(samples):
            raise RuntimeError("stop condition reached")

    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10,
    )

    end_time = node.get_clock().now().nanoseconds + int(duration * 1e9)

    try:
        while node.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(node, timeout_sec=sample_period)
    except RuntimeError:
        pass
    finally:
        node.destroy_subscription(subscription)

    if not samples:
        raise AssertionError(
            f"No joint state samples were received for '{joint_name}' "
            f"during the {duration}s observation window."
        )

    return samples


def wait_for_pendulum_steady_state(
    node,
    vel_eps=0.05,
    eff_eps=0.05,
    timeout_ns=int(10e9),
    stable_required=5,
):
    joint_name = 'cart_to_pendulum'

    start = node.get_clock().now().nanoseconds
    stable_count = 0

    def stop_when(samples):
        nonlocal stable_count

        if not samples:
            return False

        _, vel, eff = samples[-1]

        if abs(vel) < vel_eps and abs(eff) < eff_eps:
            stable_count += 1
            return stable_count >= stable_required

        stable_count = 0
        return False

    while node.get_clock().now().nanoseconds - start < timeout_ns:
        samples = observe_joint_state_window(
            node,
            joint_name,
            duration=0.5,
            sample_period=0.01,
            stop_when=stop_when,
        )

        if samples and abs(samples[-1][1]) < vel_eps and abs(samples[-1][2]) < eff_eps:
            if stable_count >= stable_required:
                return samples[-1]

    raise AssertionError('Pendulum did not converge within timeout')


def assert_joint_initial_position(node, joint_name, expected_position):
    actual_position, _, _ = read_joint_state(node, joint_name)

    if abs(actual_position - expected_position) > 0.01:
        raise AssertionError(
                f"Initial position mismatch for '{joint_name}': "
                f"expected {expected_position}, "
                f"got {actual_position}"
        )

    print(
        f"Initial position for '{joint_name}' asserted: "
        f"{actual_position:.4f} ~= {expected_position:.4f}"
    )
