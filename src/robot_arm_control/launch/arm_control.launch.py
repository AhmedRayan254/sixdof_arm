"""Full simulation stack: Fortress + ros2_control + controllers + sensors.

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
         |
         v
    sensor nodes (encoder, motor diagnostics, limit switches)

Spawning a controller before the entity exists fails with
"controller manager not available". Spawning arm_controller before
the broadcaster is active produces a controller with no state.

The sensor nodes are pure ROS 2 subscribers on /joint_states. They add
no Gazebo plugins and touch nothing in the control path, so the
existing test suite is unaffected by them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:

    description_share = get_package_share_directory('robot_arm_description')
    xacro_file = os.path.join(description_share, 'urdf', 'robot.urdf.xacro')
    default_world = os.path.join(description_share, 'worlds', 'arm_world.sdf')

    use_sensors = LaunchConfiguration('use_sensors')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' use_ros2_control:=true',
            ' use_sensors:=', use_sensors,
        ]),
        value_type=str,
    )

    # === Simulator =================================================
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', LaunchConfiguration('world')],
        output='screen',
    )

    # === Clock bridge ==============================================
    # Without this every node with use_sim_time:=true waits forever
    # for a clock that never ticks, and nothing moves.
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # === Sensor bridge =============================================
    # Only the IMU and wrist force-torque sensors live inside Gazebo.
    # Encoders, motor current, motor temperature and limit switches are
    # derived in ROS 2 from /joint_states and need no bridge.
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        condition=IfCondition(use_sensors),
        arguments=[
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/wrist_ft@geometry_msgs/msg/Wrench[ignition.msgs.Wrench',
        ],
    )

    # === Robot state publisher =====================================
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

    # === Spawn into the simulator ==================================
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

    # === Controllers ===============================================
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

    # === Derived sensor nodes ======================================
    encoder_simulator = Node(
        package='robot_arm_control',
        executable='encoder_simulator',
        name='encoder_simulator',
        output='screen',
        condition=IfCondition(use_sensors),
        parameters=[{'use_sim_time': True}],
    )

    motor_diagnostics = Node(
        package='robot_arm_control',
        executable='motor_diagnostics',
        name='motor_diagnostics',
        output='screen',
        condition=IfCondition(use_sensors),
        parameters=[{'use_sim_time': True}],
    )

    limit_switches = Node(
        package='robot_arm_control',
        executable='limit_switches',
        name='limit_switches',
        output='screen',
        condition=IfCondition(use_sensors),
        parameters=[{'use_sim_time': True}],
    )

    # === Ordering ==================================================
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

    # Sensor nodes only make sense once joint states are flowing.
    start_sensors_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[encoder_simulator, motor_diagnostics, limit_switches],
        )
    )

    return LaunchDescription([
        # Ignition does not search ROS 2's library path by default, so it
        # cannot find libign_ros2_control-system.so without this. The
        # symptom is "Failed to load system plugin ... couldn't find
        # shared library" followed by the spawner waiting forever for a
        # controller_manager that was never started.
        #
        # This MUST come before the gazebo action, or the ign process
        # inherits the old environment.
        SetEnvironmentVariable(
            name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
            value='/opt/ros/humble/lib',
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='SDF world file for Fortress to load',
        ),
        DeclareLaunchArgument(
            'use_sensors',
            default_value='true',
            description='Enable IMU, force-torque and derived sensor nodes',
        ),
        gazebo,
        clock_bridge,
        sensor_bridge,
        robot_state_publisher,
        spawn_robot,
        start_broadcaster_after_spawn,
        start_arm_after_broadcaster,
        start_sensors_after_broadcaster,
    ])
