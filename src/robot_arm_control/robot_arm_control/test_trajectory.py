#!/usr/bin/env python3
"""Run a multi-waypoint trajectory with all six joints moving together.

This is the Phase 9 test. Unlike test_all_joints, which moves one
joint at a time, here every joint is interpolated simultaneously
between waypoints - which is what a real motion looks like.

    ros2 run robot_arm_control test_trajectory
    ros2 run robot_arm_control test_trajectory --ros-args -p loops:=3
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# (label, [six angles in DEGREES], seconds from trajectory start)
WAYPOINTS = [
    ('home',        [  0,   0,   0,   0,   0,  0],  2.0),
    ('reach out',   [  0, -40,  70,   0,  35,  0],  5.0),
    ('swing left',  [ 60, -40,  70,   0,  35,  0],  8.0),
    ('twist wrist', [ 60, -40,  70,  90,  35, 45], 11.0),
    ('swing right', [-60, -30,  60, -90,  30, -45], 15.0),
    ('return home', [  0,   0,   0,   0,   0,  0], 19.0),
]


class TrajectoryTest(Node):

    def __init__(self) -> None:
        super().__init__('test_trajectory')

        self.declare_parameter('loops', 1)
        self._loops = int(self.get_parameter('loops').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _build_trajectory(self) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        for label, degrees, t in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = [math.radians(d) for d in degrees]
            # Zero velocity at each waypoint means the arm settles
            # briefly. Omit for smoother continuous motion.
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            trajectory.points.append(point)

        return trajectory

    def run(self) -> int:
        self.get_logger().info(f'Waiting for {ACTION_NAME}...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. '
                'Check: ros2 control list_controllers')
            return 1

        print()
        self.get_logger().info('Trajectory waypoints:')
        for label, degrees, t in WAYPOINTS:
            angles = ' '.join(f'{d:+4.0f}' for d in degrees)
            self.get_logger().info(f'  t={t:5.1f}s  [{angles}]  {label}')
        print()

        total = WAYPOINTS[-1][2]

        for loop in range(1, self._loops + 1):
            self.get_logger().info(f'--- Loop {loop}/{self._loops} ---')

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = self._build_trajectory()

            send_future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self, send_future, timeout_sec=10.0)

            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error('Goal rejected')
                return 1

            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=total + 15.0)

            result = result_future.result()
            if result is None:
                self.get_logger().error('Timed out waiting for result')
                return 1

            if result.result.error_code != \
                    FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f'Trajectory failed: {result.result.error_string}')
                return 1

            self.get_logger().info(f'Loop {loop} completed')

        self.get_logger().info('All trajectories completed successfully')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryTest()
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
