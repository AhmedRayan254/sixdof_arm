#!/usr/bin/env python3
"""Simulate end-of-travel and home switches on every joint.

Real industrial arms carry a hard limit switch near each end of travel
and a home/index switch used during the homing sequence. Both are
position-triggered, so a node watching /joint_states reproduces them
exactly. A Gazebo contact sensor is the wrong tool here: it detects
physical touch, not joint travel.

    ros2 run robot_arm_control limit_switches

Publishes, per joint:
    /limit_switch/<joint>/lower   std_msgs/Bool
    /limit_switch/<joint>/upper   std_msgs/Bool
    /home_switch/<joint>          std_msgs/Bool
And one combined:
    /limit_switch_diagnostics     diagnostic_msgs/DiagnosticArray

No Gazebo plugin, no bridge, no change to the control stack.
"""

import math

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# Travel limits in radians. Must match the URDF joint limits exactly.
LIMITS = {
    'joint_1': (-3.14159, 3.14159),
    'joint_2': (-2.0944, 2.0944),
    'joint_3': (-2.6180, 2.6180),
    'joint_4': (-3.14159, 3.14159),
    'joint_5': (-2.0944, 2.0944),
    'joint_6': (-3.14159, 3.14159),
}

# Home position of every joint, radians.
HOME = {name: 0.0 for name in JOINT_ORDER}


class LimitSwitches(Node):

    def __init__(self) -> None:
        super().__init__('limit_switches')

        # A real switch trips slightly before the mechanical stop.
        self.declare_parameter('limit_margin_rad', 0.035)
        self.declare_parameter('home_window_rad', 0.010)
        self.declare_parameter('update_rate_hz', 20.0)

        self._limit_margin = float(
            self.get_parameter('limit_margin_rad').value)
        self._home_window = float(
            self.get_parameter('home_window_rad').value)
        rate = float(self.get_parameter('update_rate_hz').value)

        self._position = {name: 0.0 for name in JOINT_ORDER}
        self._seen = False

        self._lower_pubs = {
            name: self.create_publisher(
                Bool, f'/limit_switch/{name}/lower', 10)
            for name in JOINT_ORDER
        }
        self._upper_pubs = {
            name: self.create_publisher(
                Bool, f'/limit_switch/{name}/upper', 10)
            for name in JOINT_ORDER
        }
        self._home_pubs = {
            name: self.create_publisher(
                Bool, f'/home_switch/{name}', 10)
            for name in JOINT_ORDER
        }
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/limit_switch_diagnostics', 10)

        self._subscription = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self._timer = self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            f'Limit switches active. Trip margin '
            f'{math.degrees(self._limit_margin):.2f} deg from each stop')

    def _on_joint_state(self, msg: JointState) -> None:
        for index, name in enumerate(msg.name):
            if name in self._position and index < len(msg.position):
                self._position[name] = msg.position[index]
        self._seen = True

    def _update(self) -> None:
        if not self._seen:
            return

        stamp = self.get_clock().now().to_msg()
        statuses = []

        for name in JOINT_ORDER:
            position = self._position[name]
            low, high = LIMITS[name]

            lower_tripped = position <= (low + self._limit_margin)
            upper_tripped = position >= (high - self._limit_margin)
            at_home = abs(position - HOME[name]) <= self._home_window

            self._lower_pubs[name].publish(Bool(data=lower_tripped))
            self._upper_pubs[name].publish(Bool(data=upper_tripped))
            self._home_pubs[name].publish(Bool(data=at_home))

            status = DiagnosticStatus()
            status.name = f'limit_switch/{name}'
            status.hardware_id = name

            if lower_tripped or upper_tripped:
                status.level = DiagnosticStatus.WARN
                status.message = (
                    'Lower limit reached' if lower_tripped
                    else 'Upper limit reached')
            elif at_home:
                status.level = DiagnosticStatus.OK
                status.message = 'At home'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'In travel'

            status.values = [
                KeyValue(key='position_rad', value=f'{position:.5f}'),
                KeyValue(key='position_deg',
                         value=f'{math.degrees(position):.2f}'),
                KeyValue(key='lower_tripped', value=str(lower_tripped)),
                KeyValue(key='upper_tripped', value=str(upper_tripped)),
                KeyValue(key='at_home', value=str(at_home)),
            ]
            statuses.append(status)

        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = statuses
        self._diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LimitSwitches()
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
