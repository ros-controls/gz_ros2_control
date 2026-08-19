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
