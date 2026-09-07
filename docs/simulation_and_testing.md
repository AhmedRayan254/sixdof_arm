# Simulation Tools and Testing

Complete reference for the simulation toolchain: what each tool is, how to install it, how to keep the environment clean, the full source of every file, every command in order, and the complete test procedure.

## Contents

1. [The Toolchain](#1-the-toolchain)
2. [Architecture and Data Flow](#2-architecture-and-data-flow)
3. [Which Gazebo, and Why It Is Confusing](#3-which-gazebo-and-why-it-is-confusing)
4. [Installation](#4-installation)
5. [Environment Setup and the Two-Workspace Trap](#5-environment-setup-and-the-two-workspace-trap)
6. [File Structure](#6-file-structure)
7. [Complete Source Files](#7-complete-source-files)
8. [Startup Sequence, Command by Command](#8-startup-sequence-command-by-command)
9. [Test Procedure A — RViz with Joint Sliders](#9-test-procedure-a--rviz-with-joint-sliders)
10. [Test Procedure B — Gazebo Fortress](#10-test-procedure-b--gazebo-fortress)
11. [Test Results Summary](#11-test-results-summary)
12. [Command Reference](#12-command-reference)
13. [Troubleshooting](#13-troubleshooting)

---

## 1. The Toolchain

Six pieces of software cooperate to simulate this arm. Each does exactly one job.

| Tool | Package | Role |
|---|---|---|
| **Gazebo Fortress** | `ignition-fortress` | Physics engine and 3D simulator. Gravity, collisions, forces. |
| **RViz2** | `ros-humble-rviz2` | Visualizer. Draws what ROS 2 believes the robot looks like. No physics. |
| **xacro** | `ros-humble-xacro` | Macro processor. Expands `robot.urdf.xacro` into plain URDF. |
| **robot_state_publisher** | `ros-humble-robot-state-publisher` | Reads URDF + joint angles, publishes the TF tree. |
| **joint_state_publisher_gui** | `ros-humble-joint-state-publisher-gui` | Slider window. Publishes `/joint_states` manually. |
| **ros_gz_sim / ros_gz_bridge** | `ros-humble-ros-gz` | Spawns robots into Fortress, translates messages. |

### Why each one is necessary

**Gazebo Fortress** answers *"what would this robot physically do?"* It is the only component that knows about mass, gravity and contact forces.

**RViz2** answers *"where does ROS 2 think each link is?"* No physics engine at all — it reads TF and draws shapes. Fast and honest about the model, but it will happily draw a physically impossible pose.

**xacro** exists because raw URDF has no variables, macros or arithmetic. Six near-identical links in raw URDF means six copies of the same 20 lines, and changing one dimension means editing it in three places (visual, collision, inertia).

**robot_state_publisher** bridges "joint angles" to "link positions". Given the URDF geometry and the current angle of every joint, it computes where every link actually is. Nothing draws the robot without it.

**joint_state_publisher_gui** exists only for testing. In a real system, controllers or hardware publish `/joint_states` instead.

**ros_gz_sim** provides the `create` node. Fortress cannot read ROS 2 topics by itself, so something must fetch the URDF and hand it to the simulator.

---

## 2. Architecture and Data Flow

### Shared front end

Both modes start the same way:

```
robot.urdf.xacro
      │
      │  xacro expands macros, properties and arithmetic
      ▼
plain URDF (XML text)
      │
      │  stored as the ROS 2 parameter "robot_description"
      ▼
robot_description
```

This happens at launch time, in memory. No intermediate file is written.

### Mode A — RViz with sliders

```
    ┌──────────────────────────────┐
    │  joint_state_publisher_gui   │
    │  (six sliders, one per joint)│
    └──────────────┬───────────────┘
                   │ /joint_states
                   │ (names + angles in radians)
                   ▼
    ┌──────────────────────────────┐
    │    robot_state_publisher     │◄──── robot_description
    │  combines geometry + angles  │
    └──────────────┬───────────────┘
                   │ /tf, /tf_static
                   ▼
    ┌──────────────────────────────┐
    │            RViz2             │
    └──────────────────────────────┘
```

No physics anywhere. The slider *asserts* an angle and everything downstream believes it.

### Mode B — Gazebo Fortress

```
    ┌──────────────────────────────┐
    │        ign gazebo            │
    │  physics engine + 3D window  │
    └──────────────▲───────────────┘
                   │ spawns the model
    ┌──────────────┴───────────────┐
    │      ros_gz_sim create       │
    │  reads /robot_description,   │
    │  builds the entity, exits    │
    └──────────────▲───────────────┘
                   │ /robot_description
    ┌──────────────┴───────────────┐
    │    robot_state_publisher     │
    └──────────────────────────────┘
```

`create` is a one-shot node. It runs, spawns the robot, and terminates. Seeing it exit in the launch output is normal.

### Why the two modes cannot run together

Both `joint_state_publisher_gui` and Gazebo want to be the authority on `/joint_states`. Run them together and `robot_state_publisher` receives contradictory angles — the slider says one thing, physics says another. RViz flickers between two poses.

To drive the arm in Gazebo you need `ros2_control`, not sliders.

---

## 3. Which Gazebo, and Why It Is Confusing

Two entirely separate simulators share a name.

| Name | Command | Status | Pairs with |
|---|---|---|---|
| Gazebo Classic 11 | `gazebo` | End of life | ROS 1, legacy ROS 2 |
| **Fortress (Ignition)** | **`ign gazebo`** | **LTS** | **ROS 2 Humble** |
| Harmonic | `gz sim` | LTS | ROS 2 Jazzy |

The rewrite was called *Ignition Gazebo*, then renamed to just *Gazebo*. Versions are alphabetical: Citadel, Edifice, Fortress, Garden, Harmonic.

### The most common mistake

```bash
sudo apt install gazebo               # WRONG - installs Gazebo Classic 11
sudo apt install ignition-fortress    # correct for Humble
```

**Symptom of Classic being installed:** the `gz` command prints a help menu listing camera, joint, log, marker, model, physics, sdf, stats, topic and world. That is Classic's utility tool, not the simulator.

### Do not mix versions

Do not install Harmonic (`gz-harmonic`) alongside `ros-humble-ros-gz`. Do not install `gazebo_ros_pkgs` or `gazebo_ros2_control` — those are Classic packages. Mixing produces broken bridges that are extremely difficult to diagnose.

---

## 4. Installation

### 4.1 Verify the base system

```bash
lsb_release -a
```

The Codename line must read `jammy`.

```bash
ros2 --help
```

Must print the ros2 command list.

### 4.2 Install Gazebo Fortress

```bash
# Prerequisites
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg

# Add the OSRF repository and signing key
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install
sudo apt-get update
sudo apt-get install -y ignition-fortress
```

### 4.3 Verify Fortress

```bash
ign gazebo --version
ign gazebo shapes.sdf
```

A 3D window opens with three shapes on a ground plane. **This must work before continuing.**

### 4.4 Install the ROS 2 packages

```bash
sudo apt install \
  ros-humble-xacro \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros-gz \
  liburdfdom-tools
```

| Package | Provides |
|---|---|
| `ros-humble-xacro` | The `xacro` command |
| `ros-humble-joint-state-publisher-gui` | The slider window |
| `ros-humble-ros-gz` | `ros_gz_sim` and `ros_gz_bridge` |
| `liburdfdom-tools` | `check_urdf`, for validating the model |

### 4.5 If Gazebo Classic was installed by mistake

```bash
sudo apt remove --purge gazebo* libgazebo*
sudo apt autoremove
```

Then follow 4.2 from the beginning.

---

## 5. Environment Setup and the Two-Workspace Trap

### 5.1 Sourcing is per-terminal

ROS 2 is not on your PATH until it is sourced, and **every new terminal starts fresh**. This is the single most common cause of "package not found".

```bash
source /opt/ros/humble/setup.bash          # the underlay (ROS 2 itself)
source ~/ros2_robot_arm_ws/install/setup.bash   # the overlay (your workspace)
```

Order matters: underlay first, overlay second. Where a package name exists in both, the overlay wins.

### 5.2 Reading a "package not found" error correctly

When `ros2 launch` fails, it prints every path it searched. **Read that list.** If your workspace path is absent, the workspace simply is not sourced in this terminal.

```
Package 'robot_arm_description' not found: "package 'robot_arm_description'
not found, searching: ['/opt/ros/humble', ...]"
```

Look for `/home/<you>/ros2_robot_arm_ws/install`. If it is missing, the fix is sourcing, not rebuilding.

### 5.3 The two-workspace trap

A ROS 2 **source build** (typically `~/ros2_humble/`) is a complete second copy of ROS 2 compiled from source. If it is sourced in `.bashrc` alongside the apt install, both are active simultaneously.

**Symptom:** the search path in error messages contains dozens of entries like:

```
'/home/ahmed/ros2_humble/install/rclcpp',
'/home/ahmed/ros2_humble/install/rviz2',
'/home/ahmed/ros2_humble/install/tf2_ros', ...
```

**Why it is a problem:** packages start resolving from the source build instead of apt. The code running is not the code you think is running, and the resulting errors make no sense.

**Diagnose:**

```bash
grep -n ros ~/.bashrc
echo $AMENT_PREFIX_PATH
```

**Fix:** edit `.bashrc` and remove the source-build line, keeping only the apt one.

```bash
nano ~/.bashrc
```

Keep this:

```bash
source /opt/ros/humble/setup.bash
```

Delete any line like this:

```bash
source ~/ros2_humble/install/setup.bash
```

Then:

```bash
source ~/.bashrc
echo $AMENT_PREFIX_PATH
```

The path should now show `/opt/ros/humble` without the long source-build chain.

**Is the source build needed?** Only if you intend to patch core ROS 2 packages. It takes one to three hours to build and roughly 16 GB of RAM. The binary apt install is the identical software. If you are not patching anything, `~/ros2_humble` only consumes disk space.

### 5.4 A safer alternative to automatic sourcing

Sourcing in `.bashrc` affects every terminal, which becomes inconvenient with more than one distribution installed. An alias gives you control:

```bash
echo "alias humble='source /opt/ros/humble/setup.bash'" >> ~/.bashrc
echo "alias armws='source ~/ros2_robot_arm_ws/install/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```

Then type `humble` and `armws` when you want them.

**Warning:** always use `>>` (append), never `>` (overwrite). A single `>` erases the entire contents of `.bashrc`. Never run these with `sudo` — `~` then refers to `/root`.

---

## 6. File Structure

```
ros2_robot_arm_ws/
├── build/                          generated by colcon, not tracked
├── install/                        generated by colcon, this is what you source
├── log/                            generated by colcon, build and run logs
└── src/
    └── robot_arm_description/
        ├── package.xml             dependencies and metadata
        ├── CMakeLists.txt          installs urdf/, launch/, rviz/
        ├── urdf/
        │   └── robot.urdf.xacro    the robot model
        ├── launch/
        │   ├── display.launch.py   RViz + sliders
        │   └── gazebo.launch.py    Fortress + spawn
        └── rviz/
            └── display.rviz        saved RViz configuration
```

### Why `install/` matters

ROS 2 tools do not read from `src/`. They read from `install/`. The `install(DIRECTORY ...)` block in `CMakeLists.txt` copies your files there.

Verify after every build:

```bash
ls install/robot_arm_description/share/robot_arm_description/
```

Expected: `launch  package.xml  rviz  urdf`

If a directory is missing, `ros2 launch` reports that the file does not exist even though you can see it in `src/`.

### Why empty directories need `.gitkeep`

Git does not track empty directories. If `urdf/`, `launch/` or `rviz/` is empty and you clone the repo, the directory vanishes and the build fails:

```
CMake Error: ament_cmake_symlink_install_directory() can't find '.../urdf'
```

Fix by placing an empty marker file in each:

```bash
touch urdf/.gitkeep launch/.gitkeep rviz/.gitkeep
```

---

## 7. Complete Source Files

### 7.1 `package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_arm_description</name>
  <version>0.1.0</version>
  <description>URDF/Xacro description of a 6-DOF serial robotic arm.</description>
  <maintainer email="ahmed.rafat.rayan@gmail.com">ahmed</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### 7.2 `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(robot_arm_description)

# No compiled code here, but ament_cmake is still required to make this
# a valid ROS 2 package.
find_package(ament_cmake REQUIRED)

# Copy these directories into install/robot_arm_description/share/
# Without this, ros2 launch and xacro cannot find our files, even though
# you can plainly see them sitting in src/.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### 7.3 `launch/display.launch.py`

```python
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
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    # Slider window. Publishes /joint_states for every non-fixed joint.
    # Without this, /joint_states stays empty and TF shows only the two
    # fixed joints.
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
```

### 7.4 `launch/gazebo.launch.py`

```python
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

    # Locate the INSTALLED package, not the src/ folder. This is why the
    # install(DIRECTORY ...) line in CMakeLists.txt matters.
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
    # ign command. Add '-r' to the cmd list to start unpaused.
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', LaunchConfiguration('world')],
        output='screen',
    )

    # Read the URDF off the topic and build the model inside the simulator.
    # This node spawns the robot and then exits - that is normal.
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
```

### 7.5 `rviz/display.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.25
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Topic:
        Value: /robot_description
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Marker Scale: 0.2
      Show Axes: true
      Show Names: true
  Global Options:
    Fixed Frame: world
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Interact
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 2.0
      Pitch: 0.4
      Focal Point:
        X: 0
        Y: 0
        Z: 0.4
Window Geometry:
  Height: 900
  Width: 1400
```

`Fixed Frame: world` matters. RViz draws everything relative to it. Point it at a frame that does not exist and you get "Fixed Frame does not exist" and a blank screen.

---

## 8. Startup Sequence, Command by Command

Run these in order. Each must succeed before the next.

### Step 1 — Open a terminal and source ROS 2

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

Expected: the ros2 command list prints.

### Step 2 — Go to the workspace

```bash
cd ~/ros2_robot_arm_ws
pwd
```

Expected: `/home/<you>/ros2_robot_arm_ws`

### Step 3 — Build

```bash
colcon build --symlink-install
```

Expected:

```
Starting >>> robot_arm_description
Finished <<< robot_arm_description [0.5s]

Summary: 1 package finished [0.7s]
```

`--symlink-install` symlinks files rather than copying, so edits to a `.xacro`, launch file or config take effect without rebuilding. Use it always.

### Step 4 — Source the workspace

```bash
source install/setup.bash
```

No output. This is the step people forget.

### Step 5 — Confirm the package is visible

```bash
ros2 pkg list | grep robot_arm_description
ros2 pkg prefix robot_arm_description
```

Expected:

```
robot_arm_description
/home/<you>/ros2_robot_arm_ws/install/robot_arm_description
```

### Step 6 — Confirm the data directories installed

```bash
ls install/robot_arm_description/share/robot_arm_description/
```

Expected:

```
launch  package.xml  rviz  urdf
```

If any of `launch`, `rviz` or `urdf` is missing, the `install(DIRECTORY ...)` line in CMakeLists.txt is wrong or the directory was empty.

### Step 7 — Validate the model

```bash
xacro src/robot_arm_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

Expected:

```
robot name is: robot_arm
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  link_1
            child(1):  link_2
                child(1):  link_3
                    child(1):  link_4
                        child(1):  link_5
                            child(1):  link_6
                                child(1):  end_effector
```

**If this fails, stop.** Nothing downstream will work.

### Step 8 — Launch

```bash
# Mode A
ros2 launch robot_arm_description display.launch.py

# Mode B (in a fresh terminal, with steps 1-4 repeated)
ros2 launch robot_arm_description gazebo.launch.py
```

---

## 9. Test Procedure A — RViz with Joint Sliders

**Goal:** verify every joint rotates around the axis the design specifies, within the limits specified.

**Why first:** this test isolates kinematics. Nothing moves on its own, so anything wrong is a modelling error, not a physics or control error.

### Launch

```bash
cd ~/ros2_robot_arm_ws
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

### A.1 — Node check

```bash
# second terminal
source ~/ros2_robot_arm_ws/install/setup.bash
ros2 node list
```

Expected — exactly three:

```
/joint_state_publisher_gui
/robot_state_publisher
/rviz2
```

### A.2 — Topic check

```bash
ros2 topic list
```

Expected:

```
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

### A.3 — Joint state check

```bash
ros2 topic echo /joint_states --once
```

Expected — all six named, all at zero:

```yaml
name:
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

If `name` is empty, the GUI is not publishing. If a joint is missing, it is not `revolute` in the URDF.

### A.4 — TF tree check

```bash
ros2 run tf2_tools view_frames
```

Expected: `frames.pdf` written to the current directory, containing **9 frames** in one unbroken chain.

**If only 2 frames appear** (`base_link` and `end_effector`), nothing is publishing `/joint_states`. Those two are the fixed joints, which `robot_state_publisher` can publish alone. The other six need joint angles.

### A.5 — Axis verification

Move **one slider at a time**, returning it to zero before the next.

| Slider | Expected motion | Fails if |
|---|---|---|
| joint_1 | Whole arm spins around vertical axis, stays upright | Arm tips sideways → axis should be `0 0 1` |
| joint_2 | Arm tilts forward/back from the shoulder | Arm spins instead → axis should be `0 1 0` |
| joint_3 | Arm bends at the elbow, upper arm stays put | Whole arm moves → wrong parent link |
| joint_4 | Wrist twists around its own length | Wrist bends → axis should be `0 0 1` |
| joint_5 | Wrist bends | Wrist twists → axis should be `0 1 0` |
| joint_6 | Tool flange twists at the very tip | Nothing visible → check TF instead |

To confirm joint_6 when the motion is too small to see:

```bash
ros2 run tf2_ros tf2_echo link_5 link_6
```

Move the joint_6 slider and watch the rotation values change.

### A.6 — Limit verification

Push each slider to both extremes and read the value in the GUI.

| Joint | Min (rad) | Max (rad) |
|---|---|---|
| joint_1 | −3.14159 | +3.14159 |
| joint_2 | −2.0944 | +2.0944 |
| joint_3 | −2.6180 | +2.6180 |
| joint_4 | −3.14159 | +3.14159 |
| joint_5 | −2.0944 | +2.0944 |
| joint_6 | −3.14159 | +3.14159 |

If a slider stops elsewhere, the `<limit>` in the URDF does not match the design table.

### A.7 — Reach verification

Set every slider to 0, then:

```bash
ros2 run tf2_ros tf2_echo base_link end_effector
```

Expected translation with the arm fully vertical:

```
- Translation: [0.000, 0.000, 1.020]
```

That is 0.06 + 0.18 + 0.30 + 0.25 + 0.12 + 0.10 + 0.07 = 1.02 m.

**This single number catches the most common modelling bug in the whole URDF.** If it is wrong, links are overlapping or gapped — almost always the `${length/2}` visual offset.

### A.8 — Self-collision check

In RViz's left panel, expand **RobotModel** and tick **Collision Enabled**. Sweep joint_3 to its extremes and watch whether link_3 passes through link_2.

Some overlap at extreme angles is normal for cylinders. Large intersection means the geometry is oversized.

### A.9 — Node graph

```bash
rqt_graph
```

Expected: `joint_state_publisher_gui` → `/joint_states` → `robot_state_publisher` → `/tf` → `rviz2`.

---

## 10. Test Procedure B — Gazebo Fortress

**Goal:** verify the model is physically valid — correct mass, sane inertia, no solver instability.

### Launch

```bash
cd ~/ros2_robot_arm_ws
source install/setup.bash
ros2 launch robot_arm_description gazebo.launch.py
```

### B.1 — Spawn check

In the **Entity Tree** panel, expand `robot_arm`. Expected — 7 links and 7 joints:

```
robot_arm
├── base_link
├── link_1 ... link_6
├── world_to_base
└── joint_1 ... joint_6
```

If a link is missing, it failed to parse. Check the terminal for SDF conversion errors.

### B.2 — Static check (paused)

While paused, orbit the camera and confirm:

- The arm stands perfectly vertical
- No links floating detached
- No links buried in the ground plane
- Sections stack end-to-end without gaps or overlap

Gaps or overlaps here mean the same `${length/2}` bug as A.7.

### B.3 — Mass check

Click each link in the Entity Tree and read its **Inertial** properties.

| Link | Expected mass (kg) |
|---|---|
| base_link | 2.0 |
| link_1 | 1.2 |
| link_2 | 1.0 |
| link_3 | 0.8 |
| link_4 | 0.4 |
| link_5 | 0.3 |
| link_6 | 0.2 |

Total 5.9 kg. A mass of 0 or 1e-6 means the inertial block did not parse.

### B.4 — Joint listing from Ignition's side

```bash
ign model -m robot_arm --list-joints
```

Lists all seven joints as Ignition sees them, confirming the URDF converted to SDF correctly.

### B.5 — Gravity test

Press the ▶ button in the bottom-left. **The arm collapses.** That is the correct result.

What matters is *how* it collapses:

| Behaviour | Meaning |
|---|---|
| Folds smoothly, settles, comes to rest | **Correct.** Inertia is sane. |
| Vibrates, jitters, shakes violently | Inertia too small relative to mass |
| Explodes, links fly apart | Inertia near zero or NaN |
| Sinks through the ground plane | Collision geometry missing |
| Moves in slow motion | Inertia far too large |

Smooth collapse to rest is a **pass**. It proves the physics engine can integrate your model without blowing up — exactly what controllers will need in the next phase.

The arm has no motors yet, so nothing resists gravity. A real arm does the same thing with its power switched off.

### B.6 — Real-time factor

Read the percentage in the bottom-right of the Gazebo window.

| Value | Meaning |
|---|---|
| ~100% | Simulation runs at real speed. Correct. |
| 30-70% | Heavy model or slow machine. Usable. |
| <30% | Software rendering, or the model is too complex. |

### B.7 — Reset test

Right-click the world → **Reset**. The arm should snap back to vertical. Press play again and it should collapse the same way. Non-deterministic behaviour between runs points at unstable inertia values.

---

## 11. Test Results Summary

| # | Test | Tool | Pass condition |
|---|---|---|---|
| A.1 | Nodes | RViz | 3 nodes running |
| A.2 | Topics | RViz | 6 topics present |
| A.3 | Joint states | RViz | 6 joints, all at 0.0 |
| A.4 | TF tree | RViz | 9 frames, one chain |
| A.5 | Joint axes | RViz | Each matches the table |
| A.6 | Joint limits | RViz | Sliders stop at spec |
| A.7 | Reach | RViz | 1.020 m at zero pose |
| A.8 | Self-collision | RViz | No gross intersection |
| A.9 | Node graph | RViz | Correct connections |
| B.1 | Spawn | Gazebo | 7 links, 7 joints |
| B.2 | Static pose | Gazebo | Vertical, no gaps |
| B.3 | Masses | Gazebo | Match the table |
| B.4 | SDF joints | Gazebo | 7 joints listed |
| B.5 | Gravity | Gazebo | Smooth collapse |
| B.6 | Real-time factor | Gazebo | >30% |
| B.7 | Reset | Gazebo | Deterministic |

### What each tool can and cannot catch

| Error | RViz | Gazebo |
|---|---|---|
| Wrong joint axis | ✓ obvious | ✗ hidden by falling |
| Wrong joint limits | ✓ sliders stop | ✗ not enforced visually |
| Links overlapping | ✓ tf2_echo | ✓ visible |
| Broken TF tree | ✓ view_frames | ✗ |
| Bad inertia | ✗ ignored | ✓ vibrates/explodes |
| Wrong mass | ✗ ignored | ✓ inspector |
| Missing collision | ✗ | ✓ sinks through floor |
| Physically impossible pose | ✗ drawn happily | ✓ rejected |

Neither tool is sufficient alone. That is the entire reason for running both.

---

## 12. Command Reference

### Environment

| Command | Effect |
|---|---|
| `source /opt/ros/humble/setup.bash` | Source ROS 2 itself (underlay) |
| `source install/setup.bash` | Source your workspace (overlay) |
| `echo $AMENT_PREFIX_PATH` | Which workspaces are active |
| `grep -n ros ~/.bashrc` | What gets sourced automatically |
| `ros2 doctor --report` | Environment diagnostics |

### Build

| Command | Effect |
|---|---|
| `colcon build --symlink-install` | Build everything, symlink data files |
| `colcon build --packages-select pkg` | Build one package only |
| `colcon build --event-handlers console_direct+` | Stream compiler output live |
| `rm -rf build install log` | Nuke stale artefacts before rebuilding |

### Model validation

| Command | Effect |
|---|---|
| `xacro file.xacro > /tmp/robot.urdf` | Expand xacro to plain URDF |
| `check_urdf /tmp/robot.urdf` | Validate and print the link tree |
| `urdf_to_graphiz /tmp/robot.urdf` | Generate a PDF diagram of the model |

### Introspection

| Command | Effect |
|---|---|
| `ros2 pkg list \| grep name` | Is the package visible? |
| `ros2 pkg prefix name` | Where is it installed? |
| `ros2 node list` | Which nodes are running |
| `ros2 node info /node` | That node's topics and services |
| `ros2 topic list` | Which topics exist |
| `ros2 topic list -t` | With message types |
| `ros2 topic echo /joint_states` | Live joint angles |
| `ros2 topic echo /joint_states --once` | One message then exit |
| `ros2 topic hz /joint_states` | Publish rate |
| `ros2 topic info /topic --verbose` | Publisher/subscriber counts and QoS |
| `ros2 param list` | All parameters |
| `rqt_graph` | Visual node/topic graph |

### TF

| Command | Effect |
|---|---|
| `ros2 run tf2_tools view_frames` | Write frames.pdf of the whole tree |
| `ros2 run tf2_ros tf2_echo A B` | Live transform from frame A to frame B |

### Ignition-side

| Command | Effect |
|---|---|
| `ign gazebo --version` | Confirm Fortress is installed |
| `ign gazebo shapes.sdf` | Open the demo world |
| `ign gazebo -r empty.sdf` | Open unpaused |
| `ign topic -l` | List Ignition topics |
| `ign model --list` | List models in the running world |
| `ign model -m robot_arm --list-joints` | List that model's joints |
| `ign model -m robot_arm --list-links` | List that model's links |

### Recording

| Command | Effect |
|---|---|
| `ros2 bag record -a -o session1` | Record all topics |
| `ros2 bag record /joint_states /tf -o session1` | Record specific topics |
| `ros2 bag info session1` | Inspect a recording |
| `ros2 bag play session1` | Replay it |

### The bridge

Fortress and ROS 2 use separate message systems. `ros_gz_bridge` translates one topic at a time:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
```

| Symbol | Direction |
|---|---|
| `@` | Bidirectional |
| `[` | Gazebo to ROS 2 only |
| `]` | ROS 2 to Gazebo only |

No bridges are required while the arm is a passive model. The clock bridge becomes necessary once controllers run on simulated time.

---

## 13. Troubleshooting

### Debugging order

Work down this list. The answer is almost always in the first four steps.

```
1. Read the actual error message, all of it
2. ros2 node list                       is the node even running?
3. ros2 topic list                      does the topic exist, spelled correctly?
4. ros2 topic info /topic --verbose     do publisher/subscriber counts match?
5. ros2 topic echo /topic               is the data sane?
6. check_urdf /tmp/robot.urdf           is the model still valid?
7. ros2 run tf2_tools view_frames       is the TF tree connected?
8. env | grep -E "ROS_DOMAIN|AMENT_PREFIX"
```

Fix only the component that is actually broken, then test again.

### Environment

| Symptom | Cause | Fix |
|---|---|---|
| `ros2: command not found` | Not sourced in this terminal | `source /opt/ros/humble/setup.bash` |
| Works in one terminal, not another | Sourcing is per-terminal | Source the workspace again |
| `colcon: command not found` | `ros-dev-tools` missing | `sudo apt install ros-dev-tools` |
| Search path lists `~/ros2_humble/install/...` | Source build sourced alongside apt | Remove that line from `.bashrc` — see section 5.3 |
| Nodes appear from an unexpected build | Two workspaces sourced at once | Clean terminal, check `$AMENT_PREFIX_PATH` |

### Build

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'robot_arm_description' not found` | Not built, or not sourced | `colcon build` then `source install/setup.bash` |
| Same error, workspace absent from search path | Not sourced in this terminal | `source ~/ros2_robot_arm_ws/install/setup.bash` |
| `can't find '.../urdf'` | Empty directory lost on clone | `mkdir -p urdf && touch urdf/.gitkeep` |
| Launch file not found | `launch/` missing from `install(DIRECTORY ...)` | Fix CMakeLists.txt, rebuild, re-source |
| Errors that stop making sense | Stale artefacts | `rm -rf build install log` then rebuild |

### RViz

| Symptom | Cause | Fix |
|---|---|---|
| Blank, "Fixed Frame does not exist" | Fixed Frame not set to `world` | Check the rviz config |
| Robot invisible or outline only | RobotModel not subscribed | Check Description Topic is `/robot_description` |
| Sliders exist, nothing moves | `robot_state_publisher` not running | `ros2 node list` |
| No slider window | GUI package not installed | `sudo apt install ros-humble-joint-state-publisher-gui` |
| TF shows only 2 frames | Nothing publishing `/joint_states` | Run `display.launch.py` |
| Crashes immediately | VM without 3D acceleration | `LIBGL_ALWAYS_SOFTWARE=1 rviz2` |

### Gazebo

| Symptom | Cause | Fix |
|---|---|---|
| `ign: command not found` | Fortress not installed | `sudo apt install ignition-fortress` |
| `gz` prints camera/joint/log/model menu | Gazebo Classic installed | `sudo apt remove --purge gazebo* libgazebo*` |
| `Package 'ros_gz_sim' not found` | Bridge not installed | `sudo apt install ros-humble-ros-gz` |
| Window opens, robot absent | `create` ran before the simulator was ready | Ctrl-C and relaunch — timing race |
| Robot vibrates or jitters | Inertia too small relative to mass | Check the `cylinder_inertia` macro |
| Robot explodes, links fly apart | Inertia near zero or NaN | Same |
| Robot sinks through the ground | Collision geometry missing | Check every link has `<collision>` |
| Moves in slow motion | Inertia far too large | Recheck the inertia formulas |
| Arm collapses when unpaused | **Expected.** No controllers attached | Nothing to fix |
| Simulation opens but does not run | Fortress starts paused | Press play, or launch with `-r` |
| Black window, very low FPS | Software rendering | `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...` |

### Model

| Symptom | Cause | Fix |
|---|---|---|
| `check_urdf` fails to parse | XML error, or a link has no parent | Read the line number in the error |
| Links overlap by half their length | Visual origin missing `${length/2}` offset | Check the link macro |
| Joint rotates around the wrong axis | Wrong `<axis xyz>` | Verify against the design table |
| Joint moves further than expected | Degrees used where radians expected | URDF is always radians |
| Reach wrong at zero pose | Links overlapping or gapped | `tf2_echo base_link end_effector` should read 1.020 |

### Silent failures

Everything appears to run but nothing happens. Nearly always one of three things:

- A topic name typo or a missing namespace
- A QoS mismatch between publisher and subscriber
- Mismatched `ROS_DOMAIN_ID` between terminals

Diagnose with `ros2 topic info /topic --verbose`.
