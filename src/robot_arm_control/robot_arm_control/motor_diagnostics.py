#!/usr/bin/env python3
"""Derive motor current and winding temperature from joint effort.

Gazebo Fortress has no motor current or temperature sensor, and it does
not need one: both quantities follow from the joint torque that
ros2_control already reports on /joint_states.

    current      I = tau / Kt
    heating      P = I^2 * R
    temperature  C * dT/dt = P - (T - T_ambient) / R_th

The thermal model is a standard first-order lumped model. Time
constants of 20 to 30 minutes are typical for industrial servo motors,
so the temperature rises slowly and realistically under sustained load.

    ros2 run robot_arm_control motor_diagnostics

Publishes, per joint:
    /motor_current/<joint>      std_msgs/Float64          amperes
    /motor_temperature/<joint>  sensor_msgs/Temperature   degrees C
And one combined:
    /motor_diagnostics          diagnostic_msgs/DiagnosticArray

No Gazebo plugin, no bridge, no change to the control stack.
"""

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState, Temperature
from std_msgs.msg import Float64

JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# Per-joint motor parameters. Scaled with the effort limits declared in
# the URDF (60, 60, 40, 20, 15, 10 N.m), so the larger proximal joints
# carry higher torque constants and larger thermal mass.
#
#   kt     torque constant           N.m per amp
#   r      winding resistance        ohms
#   c_th   thermal capacitance       J per degree C
#   r_th   thermal resistance        degrees C per watt
MOTOR_PARAMS = {
    'joint_1': {'kt': 1.16, 'r': 1.10, 'c_th': 900.0, 'r_th': 1.30},
    'joint_2': {'kt': 1.16, 'r': 1.10, 'c_th': 900.0, 'r_th': 1.30},
    'joint_3': {'kt': 0.83, 'r': 1.60, 'c_th': 620.0, 'r_th': 1.70},
    'joint_4': {'kt': 0.45, 'r': 2.40, 'c_th': 330.0, 'r_th': 2.40},
    'joint_5': {'kt': 0.36, 'r': 2.90, 'c_th': 260.0, 'r_th': 2.80},
    'joint_6': {'kt': 0.28, 'r': 3.40, 'c_th': 200.0, 'r_th': 3.20},
}

# Above this the drive would derate; below it everything is nominal.
WARN_TEMPERATURE_C = 70.0
ERROR_TEMPERATURE_C = 90.0


class MotorDiagnostics(Node):

    def __init__(self) -> None:
        super().__init__('motor_diagnostics')

        self.declare_parameter('ambient_temperature', 25.0)
        self.declare_parameter('update_rate_hz', 5.0)

        self._ambient = float(
            self.get_parameter('ambient_temperature').value)
        rate = float(self.get_parameter('update_rate_hz').value)

        # Every motor starts at ambient.
        self._temperature = {name: self._ambient for name in JOINT_ORDER}
        self._current = {name: 0.0 for name in JOINT_ORDER}
        self._effort = {name: 0.0 for name in JOINT_ORDER}

        self._current_pubs = {
            name: self.create_publisher(
                Float64, f'/motor_current/{name}', 10)
            for name in JOINT_ORDER
        }
        self._temperature_pubs = {
            name: self.create_publisher(
                Temperature, f'/motor_temperature/{name}', 10)
            for name in JOINT_ORDER
        }
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/motor_diagnostics', 10)

        self._subscription = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self._dt = 1.0 / rate
        self._timer = self.create_timer(self._dt, self._update)

        self.get_logger().info(
            f'Motor diagnostics active. Ambient {self._ambient:.1f} C')

    def _on_joint_state(self, msg: JointState) -> None:
        # Message order is not guaranteed, so index by name.
        for index, name in enumerate(msg.name):
            if name in self._effort and index < len(msg.effort):
                self._effort[name] = msg.effort[index]

    def _update(self) -> None:
        stamp = self.get_clock().now().to_msg()
        statuses = []

        for name in JOINT_ORDER:
            params = MOTOR_PARAMS[name]

            # Current from torque
            current = abs(self._effort[name]) / params['kt']
            self._current[name] = current

            # First-order thermal model
            heating = current * current * params['r']
            cooling = (self._temperature[name] - self._ambient) / params['r_th']
            self._temperature[name] += (
                (heating - cooling) / params['c_th'] * self._dt)

            current_msg = Float64()
            current_msg.data = current
            self._current_pubs[name].publish(current_msg)

            temp_msg = Temperature()
            temp_msg.header.stamp = stamp
            temp_msg.header.frame_id = name
            temp_msg.temperature = self._temperature[name]
            temp_msg.variance = 0.0
            self._temperature_pubs[name].publish(temp_msg)

            status = DiagnosticStatus()
            status.name = f'motor/{name}'
            status.hardware_id = name

            if self._temperature[name] >= ERROR_TEMPERATURE_C:
                status.level = DiagnosticStatus.ERROR
                status.message = 'Over temperature'
            elif self._temperature[name] >= WARN_TEMPERATURE_C:
                status.level = DiagnosticStatus.WARN
                status.message = 'Elevated temperature'
            else:
                status.level = DiagnosticStatus.OK
                status.message = 'Nominal'

            status.values = [
                KeyValue(key='torque_nm', value=f'{self._effort[name]:.4f}'),
                KeyValue(key='current_a', value=f'{current:.4f}'),
                KeyValue(key='temperature_c',
                         value=f'{self._temperature[name]:.2f}'),
            ]
            statuses.append(status)

        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = statuses
        self._diagnostics_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorDiagnostics()
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
