"""Spawn the 6-DOF arm in Gazebo Fortress.

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
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:

    # Locate the installed package (NOT the src/ folder - this is why the
    # install(DIRECTORY ...) line in CMakeLists.txt matters).
    pkg_share = get_package_share_directory('robot_arm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')

    # Run xacro at launch time and keep the result as a string parameter.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # Publishes /robot_description and the TF tree for every link.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Start Gazebo Fortress. Note: 'ign', not 'gz' - Fortress uses the
    # ign command. Starting PAUSED on purpose, see the README note below.
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', LaunchConfiguration('world')],
        output='screen',
    )

    # Read the URDF off the topic and build the model inside the simulator.
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
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='SDF world file for Fortress to load',
        ),
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])
