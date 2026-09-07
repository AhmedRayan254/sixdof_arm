"""Full simulation stack: Fortress + ros2_control + controllers.

Startup order matters and is enforced with event handlers:

    ign gazebo starts
         |
    robot_state_publisher publishes /robot_description
         |
    ros_gz_sim create spawns the robot
         |   (the Gazebo plugin starts controller_manager here)
         v
    joint_state_broadcaster spawner
         |
         v
    arm_controller spawner

Spawning a controller before the entity exists fails with
"controller manager not available". Spawning arm_controller before
the broadcaster is active produces a controller with no state.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
def generate_launch_description() -> LaunchDescription:

    description_share = get_package_share_directory('robot_arm_description')
    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=true']),
        value_type=str,
    )

    # --- Simulator -------------------------------------------------
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen',
    )

    # --- Clock bridge ----------------------------------------------
    # Without this every node with use_sim_time:=true waits forever
    # for a clock that never ticks, and nothing moves.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # --- Robot state publisher -------------------------------------
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

    # --- Spawn into the simulator ----------------------------------
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

    # --- Controllers -----------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='arm_controller_spawner',
        output='screen',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # --- Ordering --------------------------------------------------
    start_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    start_arm_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return LaunchDescription([
        # Ignition does not search ROS 2's library path by default, so it
        # cannot find libign_ros2_control-system.so without this. The
        # symptom is "Failed to load system plugin ... couldn't find
        # shared library" followed by the spawner waiting forever for a
        # controller_manager that was never started.
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/humble/lib',
        ),
        DeclareLaunchArgument('world', default_value='empty.sdf'),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_robot,
        start_broadcaster_after_spawn,
        start_arm_after_broadcaster,
    ])
