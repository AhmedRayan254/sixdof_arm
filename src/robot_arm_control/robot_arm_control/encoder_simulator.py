#!/usr/bin/env python3
"""Publish a realistic encoder view of the arm's joint states.

In ros2_control the joint position/velocity state interfaces ARE the
encoder. joint_state_broadcaster already publishes them, perfectly
noiseless, on /joint_states.

A real absolute encoder is quantised to a finite number of counts per
revolution and carries a small amount of noise. This node republishes
/joint_states with those effects applied, on a SEPARATE topic.

    ros2 run robot_arm_control encoder_simulator

CRITICAL: this node must never publish to /joint_states. Two publishers
on that topic means robot_state_publisher receives contradictory angles
and the TF tree flickers. Output goes to /encoder_states only.
"""

import math
import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# Counts per revolution of the absolute encoder on each joint output.
# Distal joints carry lighter, cheaper encoders in most real arms.
COUNTS_PER_REV = {
    'joint_1': 262144,   # 18-bit
    'joint_2': 262144,
    'joint_3': 131072,   # 17-bit
    'joint_4': 65536,    # 16-bit
    'joint_5': 65536,
    'joint_6': 65536,
}


class EncoderSimulator(Node):

    def __init__(self) -> None:
        super().__init__('encoder_simulator')

        self.declare_parameter('position_noise_stddev', 2.0e-5)
        self.declare_parameter('velocity_noise_stddev', 1.0e-3)
        self.declare_parameter('output_topic', '/encoder_states')

        self._pos_noise = float(
            self.get_parameter('position_noise_stddev').value)
        self._vel_noise = float(
            self.get_parameter('velocity_noise_stddev').value)
        output_topic = str(self.get_parameter('output_topic').value)

        # Precompute the angular resolution of one encoder count.
        self._resolution = {
            name: (2.0 * math.pi) / counts
            for name, counts in COUNTS_PER_REV.items()
        }

        self._publisher = self.create_publisher(JointState, output_topic, 10)

        # Must be stored on self, or Python garbage-collects the
        # subscription and the callback silently never fires.
        self._subscription = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self.get_logger().info(
            f'Encoder simulation active. Publishing on {output_topic}')

    def _quantise(self, name: str, value: float) -> float:
        """Snap an angle to the nearest whole encoder count."""
        step = self._resolution.get(name)
        if step is None or step <= 0.0:
            return value
        return round(value / step) * step

    def _on_joint_state(self, msg: JointState) -> None:
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)

        positions = list(msg.position)
        velocities = list(msg.velocity)
        efforts = list(msg.effort)

        new_positions = []
        for index, name in enumerate(out.name):
            if index >= len(positions):
                break
            raw = positions[index]
            noisy = raw + random.gauss(0.0, self._pos_noise)
            new_positions.append(self._quantise(name, noisy))

        new_velocities = [
            v + random.gauss(0.0, self._vel_noise) for v in velocities
        ]

        out.position = new_positions
        out.velocity = new_velocities
        out.effort = efforts

        self._publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderSimulator()
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
