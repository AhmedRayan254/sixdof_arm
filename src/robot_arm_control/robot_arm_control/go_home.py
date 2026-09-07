#!/usr/bin/env python3
"""Return all six joints to zero. Run this after any failed test.

    ros2 run robot_arm_control go_home
    ros2 run robot_arm_control go_home --ros-args -p duration:=5.0
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']


class GoHome(Node):

    def __init__(self) -> None:
        super().__init__('go_home')

        self.declare_parameter('duration', 3.0)
        self._duration = float(self.get_parameter('duration').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return 1

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(JOINT_NAMES)
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1.0) * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(f'Returning home over {self._duration:.1f} s')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._duration + 10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('Timed out')
            return 1

        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Home position reached')
            return 0

        self.get_logger().error(f'Failed: {result.result.error_string}')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoHome()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
