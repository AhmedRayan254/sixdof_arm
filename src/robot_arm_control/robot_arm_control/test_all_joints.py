#!/usr/bin/env python3
"""Exercise every joint in turn: move out, hold, return to zero.

This is the systematic Phase 8 test. Watch Gazebo while it runs and
check each joint against the design table.

    ros2 run robot_arm_control test_all_joints
    ros2 run robot_arm_control test_all_joints --ros-args -p angle_deg:=20.0
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

# joint name -> what you should see happen
EXPECTED = [
    ('joint_1', 'whole arm spins around the vertical axis'),
    ('joint_2', 'arm tilts forward/back from the shoulder'),
    ('joint_3', 'arm bends at the elbow'),
    ('joint_4', 'wrist twists around its own length'),
    ('joint_5', 'wrist bends'),
    ('joint_6', 'tool flange twists'),
]


class AllJointsTest(Node):

    def __init__(self) -> None:
        super().__init__('test_all_joints')

        self.declare_parameter('angle_deg', 30.0)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('hold', 1.0)

        self._angle_deg = float(self.get_parameter('angle_deg').value)
        self._duration = float(self.get_parameter('duration').value)
        self._hold = float(self.get_parameter('hold').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _move(self, joint: str, angle_deg: float) -> bool:
        """Send one joint to one angle. Returns True on success."""
        point = JointTrajectoryPoint()
        point.positions = [math.radians(angle_deg)]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1.0) * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = [joint]
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{joint}: goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._duration + 10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f'{joint}: timed out')
            return False

        ok = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not ok:
            self.get_logger().error(
                f'{joint}: failed, {result.result.error_string}')
        return ok

    def run(self) -> int:
        self.get_logger().info(f'Waiting for {ACTION_NAME}...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. '
                'Check: ros2 control list_controllers')
            return 1

        passed, failed = [], []

        for joint, description in EXPECTED:
            print()
            self.get_logger().info(f'=== {joint} ===')
            self.get_logger().info(f'Expect: {description}')

            ok = self._move(joint, self._angle_deg)
            time.sleep(self._hold)

            # Always return to zero, even if the outward move failed,
            # so the next joint starts from a known pose.
            ok = self._move(joint, 0.0) and ok
            time.sleep(0.3)

            (passed if ok else failed).append(joint)

        print()
        self.get_logger().info('================ SUMMARY ================')
        self.get_logger().info(f'Passed: {len(passed)}/6  {", ".join(passed)}')
        if failed:
            self.get_logger().error(f'Failed: {", ".join(failed)}')
            return 1

        self.get_logger().info('All joints moved successfully')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AllJointsTest()
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
