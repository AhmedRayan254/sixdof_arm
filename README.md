# 6-DOF Robotic Arm — ROS 2 Humble + Gazebo Fortress

A 6-degree-of-freedom serial robotic arm, built from scratch and simulated in Gazebo Fortress. Built incrementally as a learning project.

## Software Stack

* Ubuntu 22.04 LTS (Jammy)
* ROS 2 Humble Hawksbill
* Gazebo Fortress (Ignition)
* ros_gz_sim / ros_gz_bridge
* ros2_control
* URDF / Xacro
* RViz2
* Python 3
* colcon / ament_cmake

## Robot Concept

A serial arm with six revolute joints in an open chain:

* Joints 1-3 position the end effector in space
* Joints 4-6 orient it once positioned

Approximate reach: 0.85 m. Fully extended length: 1.02 m. All link dimensions and masses are engineering assumptions and can be adjusted in a single Xacro file.

## Joints

| Joint | Function | Axis | Limits | Effort | Velocity |
|---|---|---|---|---|---|
| joint_1 | Base yaw | Z | ±180° | 60 N·m | 2.0 rad/s |
| joint_2 | Shoulder pitch | Y | ±120° | 60 N·m | 2.0 rad/s |
| joint_3 | Elbow pitch | Y | ±150° | 40 N·m | 2.5 rad/s |
| joint_4 | Wrist roll | Z | ±180° | 20 N·m | 3.0 rad/s |
| joint_5 | Wrist pitch | Y | ±120° | 15 N·m | 3.0 rad/s |
| joint_6 | Tool roll | Z | ±180° | 10 N·m | 3.0 rad/s |

All limits are stored in radians in the URDF. Degrees shown here for readability.

## Links

| Link | Radius | Length | Mass |
|---|---|---|---|
| base_link | 0.100 m | 0.06 m | 2.0 kg |
| link_1 | 0.060 m | 0.18 m | 1.2 kg |
| link_2 | 0.050 m | 0.30 m | 1.0 kg |
| link_3 | 0.045 m | 0.25 m | 0.8 kg |
| link_4 | 0.035 m | 0.12 m | 0.4 kg |
| link_5 | 0.032 m | 0.10 m | 0.3 kg |
| link_6 | 0.030 m | 0.07 m | 0.2 kg |
| end_effector | — | — | massless frame |

Mass and radius both decrease toward the tool. Distal mass loads every joint below it, so real arms are always front-light.

## Coordinate Frames

```
world -> base_link -> link_1 -> link_2 -> link_3 -> link_4 -> link_5 -> link_6 -> end_effector
```

REP 103 convention: X forward, Y left, Z up. Metres and radians throughout. Every link extends along its own +Z axis, so each joint origin sits at the parent link's length.

Nine frames total: two fixed joints (`world_to_base`, `link_6_to_end_effector`) and six revolute joints.

## Workspace Structure

```
ros2_robot_arm_ws/
├── README.md
├── LICENSE
├── .gitignore
├── docs/
│   ├── robot_design.md
│   ├── coordinate_frames.md
│   ├── gazebo_simulation.md
│   └── troubleshooting.md
└── src/
    └── robot_arm_description/
        ├── package.xml
        ├── CMakeLists.txt
        ├── urdf/
        │   └── robot.urdf.xacro
        ├── launch/
        │   ├── display.launch.py      RViz + joint sliders
        │   └── gazebo.launch.py       Gazebo Fortress spawn
        └── rviz/
            └── display.rviz
```

Generated directories (`build/`, `install/`, `log/`) are not tracked by git.

## Installation

```
sudo apt install ignition-fortress
sudo apt install ros-humble-xacro ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-ros-gz liburdfdom-tools
```

Verify Fortress works:

```
ign gazebo shapes.sdf
```

## Build

```
cd ~/ros2_robot_arm_ws
colcon build --symlink-install
source install/setup.bash
```

Verify the package installed its data directories:

```
ros2 pkg list | grep robot_arm_description
ls install/robot_arm_description/share/robot_arm_description/
```

Expected: `launch  package.xml  rviz  urdf`

The workspace must be sourced in every terminal, every time.

## Run — RViz

```
ros2 launch robot_arm_description display.launch.py
```

Opens RViz with the robot model plus a slider window with one slider per joint. Dragging a slider rotates that joint immediately. No gravity, no physics — pure kinematics.

This is the fastest way to verify joint axes and limits are correct.

| Slider | Expected motion |
|---|---|
| joint_1 | Whole arm spins around the vertical axis |
| joint_2 | Arm tilts forward/backward from the shoulder |
| joint_3 | Arm bends at the elbow |
| joint_4 | Wrist section twists around its own length |
| joint_5 | Wrist bends |
| joint_6 | Tool flange twists |

## Run — Gazebo Fortress

```
ros2 launch robot_arm_description gazebo.launch.py
```

The Gazebo window opens with `robot_arm` in the Entity Tree, listing all seven links and all six joints. The simulation starts paused.

Pressing play makes the arm collapse. This is expected — no controllers are attached yet, so nothing holds the joints against gravity, exactly as a real arm behaves with its motors switched off. Fixed once ros2_control is added.

Load a different world:

```
ros2 launch robot_arm_description gazebo.launch.py world:=shapes.sdf
```

## Testing

Validate the model before launching anything:

```
xacro src/robot_arm_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

Prints the full parent-child link tree from `world` down to `end_effector`.

Inspect the running system:

```
ros2 node list
ros2 topic list
ros2 topic echo /joint_states
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link end_effector
```

Note: `view_frames` only reports the two fixed joints unless something is publishing to `/joint_states`. Run `display.launch.py` — the slider GUI provides those angles, and the full nine-frame chain then appears.

`tf2_echo base_link end_effector` prints the live transform from base to tool tip. This is forward kinematics computed by TF, and is what the manual FK implementation will be checked against later.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package 'robot_arm_description' not found` | `colcon build` then `source install/setup.bash` |
| `ign: command not found` | `sudo apt install ignition-fortress` |
| `gz` prints an unfamiliar menu | Gazebo Classic installed — `sudo apt remove --purge gazebo* libgazebo*` |
| `can't find '.../urdf'` during build | Empty dir lost on clone — `mkdir -p urdf && touch urdf/.gitkeep` |
| Gazebo opens with no robot | Ctrl-C and relaunch (timing race) |
| Robot vibrates or explodes | Bad inertia values — check the cylinder_inertia macro |
| Arm collapses when unpaused | Expected, no controllers yet |
| RViz blank, "Fixed Frame does not exist" | Fixed Frame must be `world` |
| Sliders exist but nothing moves | `robot_state_publisher` not running — check `ros2 node list` |
| TF shows only 2 frames | Nothing publishing `/joint_states` — run `display.launch.py` |
| Black window, low FPS | `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` |

## Project Status

* [x] Workspace created
* [x] robot_arm_description package created
* [x] URDF/Xacro model — 6 revolute joints
* [x] Inertia, collision and visual geometry on every link
* [x] Joint limits, axes and frames defined
* [x] URDF validated with check_urdf
* [x] Gazebo Fortress launch file
* [x] Robot spawned in Gazebo — all links and joints in Entity Tree
* [x] RViz display launch with joint sliders
* [x] Full TF tree verified (9 frames)
* [x] Documentation
* [x] GitHub repository
* [ ] ros2_control
* [ ] joint_state_broadcaster
* [ ] joint_trajectory_controller
* [ ] Joint testing
* [ ] Trajectory control
* [ ] Forward kinematics
* [ ] Inverse kinematics
* [ ] Workspace analysis

## Future Development

* ros2_control with the ign_ros2_control hardware interface
* Per-joint movement test scripts
* Multi-joint trajectory execution
* Forward kinematics node using homogeneous transforms
* Inverse kinematics
* Workspace reach analysis and visualization
* Gripper / end effector
* Replace primitive cylinders with meshes

## License

Apache-2.0
