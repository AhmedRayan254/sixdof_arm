"""View the 6-DOF arm in RViz with interactive joint sliders.

Data flow:

    robot.urdf.xacro
          |  xacro expands it
          v
    robot_description parameter
          |
          +--> joint_state_publisher_gui
          |       one slider per movable joint
          |       publishes /joint_states
          |
          +--> robot_state_publisher
          |       reads /joint_states + the URDF
          |       publishes /tf for every link
          |
          +--> rviz2
                  draws each link at the pose TF reports

No Gazebo here. No physics, no gravity, no controllers. This shows the
kinematic model only, which is exactly what we want to verify.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:

    pkg_share = get_package_share_directory('robot_arm_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'display.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=false']),
        value_type=str,
    )    # Slider window. Publishes /joint_states for every non-fixed joint.
    # Without this, /joint_states stays empty and TF only shows the two
    # fixed joints - which is exactly what view_frames reported earlier.
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # Combines the URDF with the current joint angles to compute where
    # every link actually is, and publishes that as TF.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        joint_state_publisher_gui,
        robot_state_publisher,
        rviz,
    ])
