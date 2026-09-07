import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_arm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Required or ros2 launch cannot find launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Required or the Gazebo plugin cannot find controllers.yaml
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ahmed',
    maintainer_email='ahmed.rafat.rayan@gmail.com',
    description='Controller configuration and test nodes for the 6-DOF arm.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_single_joint = robot_arm_control.test_single_joint:main',
            'test_all_joints   = robot_arm_control.test_all_joints:main',
            'test_trajectory   = robot_arm_control.test_trajectory:main',
            'go_home           = robot_arm_control.go_home:main',
            'joint_monitor     = robot_arm_control.joint_monitor:main',
        ],
    },
)
