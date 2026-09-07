#!/usr/bin/env python3
"""Move one joint to one angle. The most basic control test.

Uses the FollowJointTrajectory action so we get a real result back
rather than firing a message into the void.

    ros2 run robot_arm_control test_single_joint --ros-args \\
      -p joint:=joint_2 -p angle_deg:=45.0 -p duration:=2.0

Angles are given in DEGREES for convenience and converted to
radians internally. ROS 2 itself always uses radians.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

# Safety clamps, radians. Matches the URDF joint limits.
LIMITS = {
    'joint_1': (-3.14159, 3.14159),
    'joint_2': (-2.0944, 2.0944),
    'joint_3': (-2.6180, 2.6180),
    'joint_4': (-3.14159, 3.14159),
    'joint_5': (-2.0944, 2.0944),
    'joint_6': (-3.14159, 3.14159),
}


class SingleJointTest(Node):

    def __init__(self) -> None:
        super().__init__('test_single_joint')

        self.declare_parameter('joint', 'joint_2')
        self.declare_parameter('angle_deg', 45.0)
        self.declare_parameter('duration', 2.0)

        self._joint = str(self.get_parameter('joint').value)
        self._angle_deg = float(self.get_parameter('angle_deg').value)
        self._duration = float(self.get_parameter('duration').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def run(self) -> int:
        if self._joint not in LIMITS:
            self.get_logger().error(
                f'Unknown joint "{self._joint}". '
                f'Valid: {", ".join(LIMITS)}')
            return 1

        angle_rad = math.radians(self._angle_deg)
        low, high = LIMITS[self._joint]

        if not low <= angle_rad <= high:
            self.get_logger().error(
                f'{self._angle_deg:.1f} deg ({angle_rad:.4f} rad) is outside '
                f'the limits of {self._joint}: '
                f'{math.degrees(low):.1f} to {math.degrees(high):.1f} deg')
            return 1

        self.get_logger().info(f'Waiting for {ACTION_NAME}...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. Is arm_controller running? '
                'Check: ros2 control list_controllers')
            return 1

        # Build a two-point trajectory: where we are conceptually
        # heading, over the requested duration.
        point = JointTrajectoryPoint()
        point.positions = [angle_rad]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1.0) * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = [self._joint]
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f'Moving {self._joint} to {self._angle_deg:+.1f} deg '
            f'({angle_rad:+.4f} rad) over {self._duration:.1f} s')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the controller')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._duration + 10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('No result returned (timed out)')
            return 1

        code = result.result.error_code
        if code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Motion completed successfully')
            return 0

        self.get_logger().error(
            f'Motion failed, error_code={code}: {result.result.error_string}')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SingleJointTest()
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
