"""Spawn the 6-DOF arm in Gazebo Fortress. Model only, no controllers.

Data flow:

    robot.urdf.xacro
          |  xacro expands it
          v
    robot_description  (a parameter holding the full URDF text)
          |
          +--> robot_state_publisher  --> publishes /robot_description + TF
          |
          +--> ros_gz_sim create      --> reads /robot_description,
                                          builds the model inside Fortress

The arm has no controllers here, so it collapses when physics runs.
That is expected. Use robot_arm_control/arm_control.launch.py for the
controlled arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:

    pkg_share = get_package_share_directory('robot_arm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    default_world = os.path.join(pkg_share, 'worlds', 'arm_world.sdf')

    # ros2_control is disabled here: this launch shows the model only.
    # Sensors stay enabled so /imu and /wrist_ft can be inspected
    # without the control stack running.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=false',
            ' use_sensors:=true',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    # Fortress uses the 'ign' command, not 'gz'.
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', LaunchConfiguration('world')],
        output='screen',
    )

    # Simulation time must cross from Ignition into ROS 2 or every node
    # with use_sim_time waits on a clock that never ticks.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/wrist_ft@geometry_msgs/msg/Wrench[ignition.msgs.Wrench',
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot_arm',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot_arm',
            '-z', '0.0',
        ],
    )

    return LaunchDescription([
        # Ignition does not search ROS 2's library path by default.
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/humble/lib',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='SDF world file for Fortress to load',
        ),
        gazebo,
        clock_bridge,
        sensor_bridge,
        robot_state_publisher,
        spawn_robot,
    ])
