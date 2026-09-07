# 6-DOF Robotic Arm — ROS 2 Humble + Gazebo Fortress

A six-degree-of-freedom serial robotic arm, built from scratch and simulated in Gazebo Fortress. Every component is written manually and documented as it is built, so the whole system can be read, modified and debugged by hand.

Simulation only. No physical hardware is required.

---

## Overview

The arm follows the standard industrial layout used by machines like the UR5 and AR4: six revolute joints in a single open chain. The first three joints position the tool in space, the last three orient it once it arrives.

The project is built in phases. Each phase must work before the next begins, and every phase has its own test procedure.

| | |
|---|---|
| Degrees of freedom | 6, all revolute |
| Approximate reach | 0.85 m |
| Total mass | ~5.9 kg |
| Link geometry | Cylinders — simple primitives, no CAD meshes |
| Control | `ros2_control` with `joint_trajectory_controller` |
| Simulator | Gazebo Fortress (Ignition) |

All dimensions and masses are engineering assumptions, chosen to be physically plausible rather than copied from a real product. They live as Xacro properties at the top of one file and can be changed in one place.

---

## Software Stack

| Layer | Component |
|---|---|
| OS | Ubuntu 22.04 LTS (Jammy) |
| Middleware | ROS 2 Humble Hawksbill |
| Simulator | Gazebo Fortress (Ignition) |
| Sim bridge | `ros_gz_sim`, `ros_gz_bridge` |
| Control | `ros2_control`, `ros2_controllers`, `ign_ros2_control` |
| Description | URDF / Xacro |
| Visualization | RViz2 |
| Language | Python 3 |
| Build | colcon, ament_cmake, ament_python |

**This project uses Gazebo Fortress, launched with `ign gazebo`.** It does not use Gazebo Classic. Installing `gazebo_ros_pkgs` or `gazebo_ros2_control` alongside it produces broken bridges that are difficult to diagnose.

---

## Repository Structure

```
ros2_robot_arm_ws/
├── README.md
├── LICENSE
├── .gitignore
│
├── docs/
│   ├── simulation_and_testing.md    toolchain, setup, model tests
│   └── ros2_control.md              control layer, controller tests
│
└── src/
    ├── robot_arm_description/       WHAT the robot is
    │   ├── package.xml
    │   ├── CMakeLists.txt
    │   ├── urdf/
    │   │   ├── robot.urdf.xacro     geometry, links, joints, inertia
    │   │   └── ros2_control.xacro   hardware interface declaration
    │   ├── launch/
    │   │   ├── display.launch.py    RViz + joint sliders
    │   │   └── gazebo.launch.py     Fortress spawn, no control
    │   └── rviz/
    │       └── display.rviz
    │
    └── robot_arm_control/           HOW the robot is driven
        ├── package.xml
        ├── setup.py
        ├── config/
        │   └── controllers.yaml     controller definitions
        ├── launch/
        │   └── arm_control.launch.py   Fortress + ros2_control
        └── robot_arm_control/
            ├── joint_monitor.py        read joint states
            ├── test_single_joint.py    move one joint
            ├── test_all_joints.py      sweep every joint
            ├── test_trajectory.py      multi-joint motion
            └── go_home.py              return to zero
```

Generated directories (`build/`, `install/`, `log/`) are not tracked by git.

### Why two packages

| Package | Build type | Contains | Changes when |
|---|---|---|---|
| `robot_arm_description` | `ament_cmake` | Data files only — URDF, RViz config | The robot's physical design changes |
| `robot_arm_control` | `ament_python` | Python nodes with entry points | The way the robot is driven changes |

The split matters because a description package can be reused with a completely different control stack, and because `ament_cmake` installs directories while `ament_python` registers executables. Mixing them into one package means neither job is done cleanly.

---

## Robot Design

### Kinematic chain

```
                              ┌─────────────┐
                              │end_effector │  massless TF frame
                              └──────┬──────┘
                                     │ fixed
                              ┌──────┴──────┐
                              │   link_6    │  0.07 m
                              └──────┬──────┘
                            joint_6  ○ roll   (Z, ±180°)   ┐
                              ┌──────┴──────┐               │
                              │   link_5    │  0.10 m       │
                              └──────┬──────┘               │ ORIENTATION
                            joint_5  ○ pitch  (Y, ±120°)    │ (wrist)
                              ┌──────┴──────┐               │
                              │   link_4    │  0.12 m       │
                              └──────┬──────┘               │
                            joint_4  ○ roll   (Z, ±180°)   ┘
                              ┌──────┴──────┐
                              │   link_3    │  0.25 m  forearm
                              └──────┬──────┘
                            joint_3  ○ elbow  (Y, ±150°)   ┐
                              ┌──────┴──────┐               │
                              │   link_2    │  0.30 m       │ POSITIONING
                              └──────┬──────┘               │
                            joint_2  ○ shoulder (Y, ±120°)  │
                              ┌──────┴──────┐               │
                              │   link_1    │  0.18 m       │
                              └──────┬──────┘               │
                            joint_1  ○ base yaw (Z, ±180°) ┘
                              ┌──────┴──────┐
                              │  base_link  │  0.06 m
                              └──────┬──────┘
                                     │ fixed
                                  ┌──┴──┐
                                  │world│
                                  └─────┘
```

Nine frames: two fixed joints and six revolute joints. REP 103 convention — X forward, Y left, Z up, metres and radians throughout. Every link extends along its own +Z axis, so each joint origin sits at the parent link's length.

### Joints

| Joint | Function | Axis | Limits | Effort | Velocity |
|---|---|---|---|---|---|
| joint_1 | Base yaw | Z | ±180° | 60 N·m | 2.0 rad/s |
| joint_2 | Shoulder pitch | Y | ±120° | 60 N·m | 2.0 rad/s |
| joint_3 | Elbow pitch | Y | ±150° | 40 N·m | 2.5 rad/s |
| joint_4 | Wrist roll | Z | ±180° | 20 N·m | 3.0 rad/s |
| joint_5 | Wrist pitch | Y | ±120° | 15 N·m | 3.0 rad/s |
| joint_6 | Tool roll | Z | ±180° | 10 N·m | 3.0 rad/s |

### Links

| Link | Radius | Length | Mass |
|---|---|---|---|
| base_link | 0.100 m | 0.06 m | 2.0 kg |
| link_1 | 0.060 m | 0.18 m | 1.2 kg |
| link_2 | 0.050 m | 0.30 m | 1.0 kg |
| link_3 | 0.045 m | 0.25 m | 0.8 kg |
| link_4 | 0.035 m | 0.12 m | 0.4 kg |
| link_5 | 0.032 m | 0.10 m | 0.3 kg |
| link_6 | 0.030 m | 0.07 m | 0.2 kg |

Mass and radius both decrease toward the tool. Distal mass loads every joint below it, so real arms are always front-light.

---

## System Flow

### Shared front end

Every mode starts the same way:

```
robot.urdf.xacro
      │  xacro expands macros, properties and arithmetic
      ▼
plain URDF (XML text)
      │  stored as the ROS 2 parameter "robot_description"
      ▼
robot_description
```

This happens at launch time, in memory. No intermediate file is written.

### Mode A — RViz with sliders (no physics)

```
joint_state_publisher_gui  ──/joint_states──▶  robot_state_publisher
                                                       │
                                                     /tf
                                                       ▼
                                                     RViz2
```

The slider *asserts* an angle and everything downstream believes it. No gravity, no mass, no collisions. RViz will happily draw a physically impossible pose.

Used to verify the **kinematic model**: joint axes, limits, reach, TF tree.

### Mode B — Gazebo, no control

```
robot_state_publisher ──/robot_description──▶ ros_gz_sim create ──▶ ign gazebo
```

The arm spawns and stands upright while paused. Pressing play makes it collapse — nothing holds the joints.

Used to verify the **physical model**: mass, inertia, collision geometry, solver stability.

### Mode C — Gazebo with ros2_control

```
   test script
       │  FollowJointTrajectory goal
       ▼
┌──────────────────────────┐
│  joint_trajectory_ctrl   │  interpolates between waypoints
│     ("arm_controller")   │
└────────────┬─────────────┘
             │ command interfaces
             ▼
┌──────────────────────────┐
│    controller_manager    │  100 Hz update loop
│  (runs INSIDE Gazebo)    │
└────────────┬─────────────┘
             │ ign_ros2_control/IgnitionSystem
             ▼
┌──────────────────────────┐
│   Gazebo Fortress joints │
└────────────┬─────────────┘
             │ state interfaces
             ▼
┌──────────────────────────┐
│ joint_state_broadcaster  │ ──/joint_states──▶
└──────────────────────────┘
```

The arm now holds position against gravity and executes commanded trajectories.

**The `controller_manager` runs inside the Gazebo process**, started by a plugin declared in the URDF. It is not a node you launch separately. Killing Gazebo kills control, and controllers cannot spawn until the robot exists in the simulator.

### Mode A and Mode C cannot run together

Both `joint_state_publisher_gui` and `joint_state_broadcaster` publish `/joint_states`. Running both means `robot_state_publisher` receives contradictory angles. The `use_ros2_control` xacro argument switches between them.

---

## Installation

### 1. Verify the base system

```bash
lsb_release -a          # Codename must read "jammy"
ros2 --help             # must print the ros2 command list
```

### 2. Install Gazebo Fortress

```bash
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install -y ignition-fortress
```

Verify — this must open a 3D window:

```bash
ign gazebo shapes.sdf
```

### 3. Install ROS 2 packages

```bash
sudo apt install \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros-gz \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ign-ros2-control \
  liburdfdom-tools
```

### 4. Clone and build

```bash
git clone https://github.com/AhmedRayan254/sixdof_arm.git ~/ros2_robot_arm_ws
cd ~/ros2_robot_arm_ws
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

Verify:

```bash
ros2 pkg list | grep robot_arm
ros2 pkg executables robot_arm_control
ls install/robot_arm_description/share/robot_arm_description/
```

Expected: both packages listed, five executables, and `launch  package.xml  rviz  urdf`.

**Source the workspace in every terminal, every time.** Sourcing is per-terminal, not per-machine.

---

## Running

### Validate the model first

```bash
xacro src/robot_arm_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

Prints the nine-link tree from `world` to `end_effector`. If this fails, nothing downstream will work.

### RViz with joint sliders

```bash
ros2 launch robot_arm_description display.launch.py
```

Two windows: RViz with the arm and axis triads, and a slider window. Dragging a slider rotates that joint immediately.

### Gazebo, model only

```bash
ros2 launch robot_arm_description gazebo.launch.py
```

Opens Fortress with `robot_arm` in the Entity Tree. Starts paused. Pressing play makes the arm collapse — expected, since no controllers are attached.

### Gazebo with control

```bash
ros2 launch robot_arm_control arm_control.launch.py
```

Starts Fortress, bridges `/clock`, spawns the arm, then activates `joint_state_broadcaster` and `arm_controller`.

**The arm now stands upright and holds position.** That visual change is the clearest proof the control chain works.

---

## Controlling the Arm

```bash
# Watch joint state in a dedicated terminal
ros2 run robot_arm_control joint_monitor

# Move one joint (degrees, converted internally)
ros2 run robot_arm_control test_single_joint --ros-args \
  -p joint:=joint_2 -p angle_deg:=45.0 -p duration:=2.0

# Sweep every joint in turn
ros2 run robot_arm_control test_all_joints

# Multi-waypoint coordinated motion
ros2 run robot_arm_control test_trajectory

# Return to zero
ros2 run robot_arm_control go_home
```

Raw trajectory without a script — note positions are **radians**:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [{positions: [0.5, -0.3, 0.8, 0.0, 0.4, 0.0],
              time_from_start: {sec: 3}}]}"
```

---

## Testing

Each mode verifies something the others cannot.

| Mode | Verifies | Catches |
|---|---|---|
| RViz + sliders | Kinematic model | Wrong axis, wrong limits, broken TF, bad reach |
| Gazebo, no control | Physical model | Bad inertia, wrong mass, missing collision |
| Gazebo + control | Control chain | Interface claiming, trajectory execution, holding torque |

### Quick checks

```bash
# Model
check_urdf /tmp/robot.urdf
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link end_effector   # 1.020 m at zero pose

# Control
ros2 control list_controllers                       # both active
ros2 control list_hardware_interfaces               # 6 command, 18 state
ros2 topic hz /joint_states                         # ~50 Hz
ros2 action list                                    # action server present
```

Full procedures, with expected output for every command:

- [docs/simulation_and_testing.md](docs/simulation_and_testing.md) — 16 model and simulator tests
- [docs/ros2_control.md](docs/ros2_control.md) — 15 control tests

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| `Package 'robot_arm_*' not found` | Read the search path in the error. Workspace absent → source it. Present → build it. |
| `ign: command not found` | `sudo apt install ignition-fortress` |
| `gz` prints an unfamiliar menu | Gazebo Classic installed — `sudo apt remove --purge gazebo* libgazebo*` |
| `unbound prefix` in xacro | A `xacro:` tag sits above the `<robot>` tag |
| `can't find '.../urdf'` during build | Empty dir lost on clone — `mkdir -p` it and add a `.gitkeep` |
| `Failed to load system plugin` | `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` not set to `/opt/ros/humble/lib` |
| `controller manager not available` (loops) | The plugin never loaded — fix that error first |
| Arm collapses with controllers active | Interfaces `[unclaimed]` — `arm_controller` failed to start |
| Nothing moves, no error | `/clock` not bridged |
| TF shows only 2 frames | Nothing publishing `/joint_states` |
| Black window, low FPS | `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` |

Full error reference, including real failures from this build with their diagnosis, is in the docs above.

---

## Development Phases

```
PHASE 1   Planning — architecture, joints, frames          ✓
PHASE 2   Workspace and packages                           ✓
PHASE 3   Robot design on paper                            ✓
PHASE 4   URDF/Xacro model                                 ✓
PHASE 5   Physics — mass and inertia                       ✓
PHASE 6   Gazebo Fortress spawn                            ✓
PHASE 7   ros2_control                                     ✓
PHASE 8   Per-joint testing                                ✓
PHASE 9   Trajectory control                               ✓
PHASE 10  Forward kinematics                               □
PHASE 11  Inverse kinematics                               □
PHASE 12  Workspace analysis                               □
```

---

## Project Status

* [x] Workspace and two packages created
* [x] URDF/Xacro model — 6 revolute joints
* [x] Inertia, collision and visual geometry on every link
* [x] Joint limits, axes and frames defined
* [x] URDF validated with `check_urdf`
* [x] RViz display with joint sliders
* [x] Full TF tree verified — 9 frames
* [x] Robot spawned in Gazebo Fortress
* [x] `ros2_control` hardware interface (IgnitionSystem)
* [x] `joint_state_broadcaster`
* [x] `joint_trajectory_controller`
* [x] Arm holds position against gravity
* [x] Joint state monitor
* [x] Single-joint test with limit checking
* [x] All-joints sweep test
* [x] Multi-waypoint trajectory test
* [x] Documentation
* [ ] Forward kinematics
* [ ] Inverse kinematics
* [ ] Workspace analysis

---

## Future Development

* Forward kinematics node using homogeneous transforms, checked against TF
* Inverse kinematics — analytical solution for this wrist configuration
* Workspace reach analysis and visualization
* Gripper / end effector with its own control group
* Replace primitive cylinders with meshes
* Velocity and effort control modes alongside position
* CI — `colcon build` and lint on every push

---

## License

Apache-2.0. See [LICENSE](LICENSE).
