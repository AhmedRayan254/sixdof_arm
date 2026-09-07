# 6-DOF Robotic Arm — ROS 2 Humble + Gazebo Fortress

A 6-degree-of-freedom serial robotic arm, built from scratch and simulated in
Gazebo Fortress. Built incrementally as a learning project.

## Software Stack

- Ubuntu 22.04 LTS (Jammy)
- ROS 2 Humble Hawksbill
- Gazebo Fortress (Ignition)
- ros2_control
- URDF / Xacro
- Python 3

## Robot Concept

A serial arm with six revolute joints in an open chain:

- Joints 1-3 position the end effector in space
- Joints 4-6 orient it once positioned

Approximate reach: 0.85 m. All link dimensions and masses are engineering
assumptions and can be adjusted in a single Xacro file.

## Workspace Structure

    ros2_robot_arm_ws/
    └── src/
        └── robot_arm_description/    URDF/Xacro model, launch, RViz config

## Build

    cd ~/ros2_robot_arm_ws
    colcon build --symlink-install
    source install/setup.bash

## Project Status

- [x] Workspace created
- [x] robot_arm_description package created
- [ ] URDF/Xacro model
- [ ] Robot visible in RViz
- [ ] Robot spawned in Gazebo
- [ ] ros2_control
- [ ] Joint testing
- [ ] Trajectory control
- [ ] Forward kinematics
- [ ] Inverse kinematics
- [ ] Workspace analysis

## License

Apache-2.0
