#!/usr/bin/env python3
"""Print the arm's joint positions in a readable table.

Subscribes to /joint_states and prints both radians and degrees.
Run this alongside any test to watch what the arm is actually doing.

    ros2 run robot_arm_control joint_monitor
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']


class JointMonitor(Node):

    def __init__(self) -> None:
        super().__init__('joint_monitor')

        self.declare_parameter('rate_hz', 2.0)
        rate = float(self.get_parameter('rate_hz').value)

        self._latest = None

        # Must be stored on self, or Python garbage-collects the
        # subscription and the callback silently never fires.
        self._subscription = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self._timer = self.create_timer(1.0 / rate, self._print_table)

        self.get_logger().info('Waiting for /joint_states...')

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest = msg

    def _print_table(self) -> None:
        if self._latest is None:
            return

        # Message order is not guaranteed, so index by name.
        lookup = dict(zip(self._latest.name, self._latest.position))

        lines = ['', '  joint      radians    degrees',
                 '  ---------------------------------']
        for name in JOINT_ORDER:
            if name not in lookup:
                lines.append(f'  {name}    (not reported)')
                continue
            rad = lookup[name]
            lines.append(f'  {name}   {rad:+8.4f}   {math.degrees(rad):+8.2f}')

        print('\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

