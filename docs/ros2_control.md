# ros2_control with Gazebo Fortress

Complete reference for the control layer of the 6-DOF arm: what `ros2_control` is, how it connects to Gazebo Fortress, every file in full, the staged verification procedure, the test suite, and every error encountered while building it.

## Contents

1. [Concepts](#1-concepts)
2. [Architecture](#2-architecture)
3. [Installation](#3-installation)
4. [Package Layout](#4-package-layout)
5. [Complete Source Files](#5-complete-source-files)
6. [Build Procedure](#6-build-procedure)
7. [Staged Verification](#7-staged-verification)
8. [Test Suite](#8-test-suite)
9. [Test Summary](#9-test-summary)
10. [Error Reference](#10-error-reference)
11. [Errors Encountered During This Build](#11-errors-encountered-during-this-build)

---

## 1. Concepts

### The problem ros2_control solves

Before controllers, the arm collapses the moment physics runs. Nothing applies torque to the joints. A real arm does exactly the same thing with its motors unpowered.

`ros2_control` is the layer between "I want joint_2 at 0.5 rad" and the actuator that makes it happen. It provides:

- A standard way to describe what a joint can do
- A plugin system so the same controller works against simulation, mock hardware, or real drivers
- A fixed-rate update loop that reads state and writes commands deterministically

### Command interfaces and state interfaces

This distinction underlies everything else.

| Interface type | Direction | Example |
|---|---|---|
| **command_interface** | You → hardware | `position` — "go to 0.5 rad" |
| **state_interface** | Hardware → you | `position`, `velocity`, `effort` — "I am at 0.48 rad" |

Each joint declares which it supports. Our arm has one command interface (`position`) and three state interfaces per joint. Six joints gives 6 command interfaces and 18 state interfaces.

A command interface can be **claimed** by at most one controller at a time. Two controllers cannot both command the same joint — that would be two drivers fighting over one steering wheel.

### The three pieces

| Piece | What it is | Why it exists |
|---|---|---|
| **Hardware interface** | `ign_ros2_control/IgnitionSystem` | Translates between ros2_control and Gazebo's joints. Declared in the URDF. |
| **joint_state_broadcaster** | A read-only controller | Publishes `/joint_states` from state interfaces. Replaces the slider GUI. |
| **joint_trajectory_controller** | A command controller | Accepts waypoints, interpolates between them, writes position commands. |

### Why the broadcaster comes first

`joint_state_broadcaster` only reads. If it works, the hardware interface is talking to Gazebo correctly.

Starting it alone isolates *"is the plumbing connected?"* from *"does my controller work?"* — two entirely different failures that look identical if you start both at once.

### Positions, velocities, effort

| Quantity | Unit | Meaning |
|---|---|---|
| position | radians | Joint angle |
| velocity | rad/s | Rate of change |
| effort | N·m | Torque being applied |

**ROS 2 is always radians.** Degrees appear only in human-facing output. The test nodes in this project accept degrees as a convenience parameter and convert internally — the conversion is `math.radians()`, never a hardcoded factor.

Reference conversions:

| Degrees | Radians |
|---|---|
| 30 | 0.5236 |
| 45 | 0.7854 |
| 90 | 1.5708 |
| 120 | 2.0944 |
| 150 | 2.6180 |
| 180 | 3.14159 |

---

## 2. Architecture

### Control loop

```
   your test script
         │  sends a FollowJointTrajectory goal
         ▼
┌─────────────────────────────────┐
│   joint_trajectory_controller   │  interpolates between waypoints
│        ("arm_controller")       │  outputs a position command per joint
└────────────────┬────────────────┘
                 │ command interfaces (write)
                 ▼
┌─────────────────────────────────┐
│       controller_manager        │  loads/starts/stops controllers
│   owns the hardware interface   │  runs the update loop at 100 Hz
└────────────────┬────────────────┘
                 │ ign_ros2_control/IgnitionSystem
                 ▼
┌─────────────────────────────────┐
│      Gazebo Fortress joints     │  applies the command, reports actual
└────────────────┬────────────────┘
                 │ state interfaces (read)
                 ▼
┌─────────────────────────────────┐
│    joint_state_broadcaster      │  publishes /joint_states
└─────────────────────────────────┘
```

### Where the controller_manager actually runs

This is the part that surprises people. With `ign_ros2_control`, the `controller_manager` does **not** run as a separate node you launch. It runs **inside the Gazebo process**, started by the plugin declared in the URDF's `<gazebo>` block.

Consequences:

- Killing Gazebo kills the controller_manager
- Controllers cannot be spawned until the robot has been spawned into the simulator
- The plugin reads `controllers.yaml` at spawn time, not at launch time
- If the plugin fails to load, there is no controller_manager at all — and spawners wait forever for a service that will never appear

### Startup ordering

```
ign gazebo starts
     │
robot_state_publisher publishes /robot_description
     │
ros_gz_sim create spawns the robot
     │   ← the Gazebo plugin starts controller_manager HERE
     ▼
joint_state_broadcaster spawner
     │
     ▼
arm_controller spawner
```

Spawning a controller before the entity exists produces "controller manager not available". Spawning `arm_controller` before the broadcaster is active produces a controller with no state to work from. The launch file enforces this with `OnProcessExit` event handlers.

### Why /clock must be bridged

Ignition and ROS 2 keep separate clocks. With `use_sim_time: true`, every ROS 2 node waits for `/clock` before it will act on anything. If nothing bridges Ignition's clock into ROS 2, the clock never ticks, and every node sits waiting — silently, with no error message.

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
```

The `[` means Gazebo → ROS 2 only.

---

## 3. Installation

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ign-ros2-control
```

| Package | Provides | Needed because |
|---|---|---|
| `ros2-control` | `controller_manager`, `spawner`, `ros2 control` CLI | Core framework |
| `ros2-controllers` | `joint_state_broadcaster`, `joint_trajectory_controller` | The controllers themselves |
| `ign-ros2-control` | `IgnitionSystem` hardware plugin | Bridges ros2_control to Fortress |

### Verify

```bash
ros2 control --help
dpkg -l | grep ign-ros2-control
find /opt/ros/humble -name "libign_ros2_control-system.so"
```

The last command must return a path. If it returns nothing, the package is not installed.

### Do not install the wrong variant

Both of these exist in the Humble repositories:

| Package | For | Plugin filename |
|---|---|---|
| `ros-humble-ign-ros2-control` | **Fortress** — use this | `libign_ros2_control-system.so` |
| `ros-humble-gz-ros2-control` | Harmonic | `libgz_ros2_control-system.so` |
| `ros-humble-gazebo-ros2-control` | Gazebo Classic | `libgazebo_ros2_control.so` |

Installing more than one produces plugin-resolution failures that are extremely difficult to diagnose. Humble pairs with Fortress, so `ign` is correct.

---

## 4. Package Layout

Controller configuration and test scripts live in their own package, separate from the robot description.

```
ros2_robot_arm_ws/src/
├── robot_arm_description/           the robot model
│   ├── urdf/
│   │   ├── robot.urdf.xacro         geometry, links, joints
│   │   └── ros2_control.xacro       hardware interface  ← new
│   └── launch/
│       └── display.launch.py        RViz + sliders (control disabled)
│
└── robot_arm_control/               the control layer  ← new package
    ├── package.xml
    ├── setup.py
    ├── config/
    │   └── controllers.yaml         controller definitions
    ├── launch/
    │   └── arm_control.launch.py    Fortress + controllers
    └── robot_arm_control/
        ├── joint_monitor.py
        ├── test_single_joint.py
        ├── test_all_joints.py
        ├── test_trajectory.py
        └── go_home.py
```

### Why `ament_python` for the control package

It contains Python nodes with entry points. The description package only installs data files, so it is `ament_cmake`.

### Create it

```bash
cd ~/ros2_robot_arm_ws/src
ros2 pkg create --build-type ament_python robot_arm_control
cd robot_arm_control
rm -rf robot_arm_control/robot_arm_control 2>/dev/null
mkdir -p config launch
touch config/.gitkeep launch/.gitkeep
```

---

## 5. Complete Source Files

### 5.1 `robot_arm_description/urdf/ros2_control.xacro`

The hardware interface. Declares what ros2_control may do with each joint.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ==================================================================
       ros2_control HARDWARE INTERFACE

       Declares what ros2_control is allowed to do with each joint:

         command_interface  what we can WRITE to  (position commands)
         state_interface    what we can READ     (position, velocity, effort)

       The plugin ign_ros2_control/IgnitionSystem connects these
       interfaces to the actual joints inside Gazebo Fortress.
       ================================================================== -->

  <!-- One joint's interface block. Written once, used six times. -->
  <xacro:macro name="arm_joint_interface" params="name lower upper">
    <joint name="${name}">

      <!-- We command position. The min/max here are a safety clamp
           inside ros2_control, separate from the URDF joint limits. -->
      <command_interface name="position">
        <param name="min">${lower}</param>
        <param name="max">${upper}</param>
      </command_interface>

      <!-- What we can read back from the simulated joint. -->
      <state_interface name="position">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>

    </joint>
  </xacro:macro>

  <xacro:macro name="arm_ros2_control" params="controllers_file">

    <ros2_control name="RobotArmSystem" type="system">
      <hardware>
        <!-- Fortress. NOT gazebo_ros2_control, that is Gazebo Classic. -->
        <plugin>ign_ros2_control/IgnitionSystem</plugin>
      </hardware>

      <xacro:arm_joint_interface name="joint_1" lower="-3.14159" upper="3.14159"/>
      <xacro:arm_joint_interface name="joint_2" lower="-2.0944"  upper="2.0944"/>
      <xacro:arm_joint_interface name="joint_3" lower="-2.6180"  upper="2.6180"/>
      <xacro:arm_joint_interface name="joint_4" lower="-3.14159" upper="3.14159"/>
      <xacro:arm_joint_interface name="joint_5" lower="-2.0944"  upper="2.0944"/>
      <xacro:arm_joint_interface name="joint_6" lower="-3.14159" upper="3.14159"/>
    </ros2_control>

    <!-- The Gazebo-side plugin that actually runs the controller_manager
         inside the simulator process. It reads controllers.yaml to know
         which controllers exist. -->
    <gazebo>
      <plugin filename="libign_ros2_control-system.so"
              name="ign_ros2_control::IgnitionROS2ControlPlugin">
        <parameters>${controllers_file}</parameters>
        <controller_manager_name>controller_manager</controller_manager_name>
      </plugin>
    </gazebo>

  </xacro:macro>

</robot>
```

**`initial_value` matters.** Without it the arm may start at a random pose or drop before the controller activates.

### 5.2 Wiring it into `robot.urdf.xacro`

Two edits. Position is critical.

**Near the top, immediately inside the `<robot>` tag:**

```xml
<?xml version="1.0"?>
<robot name="robot_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- ============== ARGUMENTS ========================================
       use_ros2_control:
         true   for Gazebo - loads the IgnitionSystem hardware interface
         false  for RViz with sliders - the slider GUI publishes
                /joint_states instead, and the two must never both run
       ================================================================== -->
  <xacro:arg name="use_ros2_control" default="true"/>
```

**At the very bottom, immediately before `</robot>`:**

```xml
  <!-- ============== ros2_control =====================================
       Must come AFTER every joint is defined, because the hardware
       interface refers to those joints by name.
       ================================================================== -->
  <xacro:include filename="$(find robot_arm_description)/urdf/ros2_control.xacro"/>

  <xacro:if value="$(arg use_ros2_control)">
    <xacro:arm_ros2_control
      controllers_file="$(find robot_arm_control)/config/controllers.yaml"/>
  </xacro:if>

</robot>
```

**Two rules that cause real failures if broken:**

1. Nothing with a `xacro:` prefix may appear **above** the `<robot>` tag. The namespace is declared on that tag, so anything before it is an unbound prefix.
2. The ros2_control block must come **after** all joints, because it references them by name.

**Why the argument exists:** `display.launch.py` passes `use_ros2_control:=false`. The slider GUI and `joint_state_broadcaster` both publish `/joint_states`. Running both means `robot_state_publisher` receives contradictory angles and RViz flickers between two poses.

### 5.3 `robot_arm_control/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>robot_arm_control</name>
  <version>0.1.0</version>
  <description>Controller configuration and test nodes for the 6-DOF arm.</description>
  <maintainer email="ahmed.rafat.rayan@gmail.com">ahmed</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>trajectory_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>control_msgs</exec_depend>
  <exec_depend>builtin_interfaces</exec_depend>

  <exec_depend>robot_arm_description</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <exec_depend>ign_ros2_control</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 5.4 `robot_arm_control/setup.py`

```python
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
```

The two `data_files` glob entries are mandatory. Without them, `ros2 launch` reports the launch file does not exist, and the Gazebo plugin cannot find `controllers.yaml`.

The `entry_points` block is what makes `ros2 run` work. Read each line as: the command NAME runs the function `main` in the module MODULE.

### 5.5 `robot_arm_control/config/controllers.yaml`

```yaml
# ============================================================
# ros2_control configuration for the 6-DOF arm
#
# Read by the Gazebo plugin at spawn time. Two controllers:
#   joint_state_broadcaster  reads joint states, publishes them
#   arm_controller           accepts trajectories, commands joints
# ============================================================

controller_manager:
  ros__parameters:
    # How many times per second the control loop runs.
    # Must be >= the simulation step rate to avoid missed updates.
    update_rate: 100  # Hz

    use_sim_time: true

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController


# ============================================================
# arm_controller
# Joint order here defines the order used in every trajectory
# message. Get it wrong and joints move to each other's targets.
# ============================================================
arm_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

    # What the controller writes to the hardware
    command_interfaces:
      - position

    # What the controller reads back
    state_interfaces:
      - position
      - velocity

    # Allows commanding a subset of joints. Essential for
    # single-joint testing.
    allow_partial_joints_goal: true

    # Publish the controller's own state for debugging
    state_publish_rate: 50.0
    action_monitor_rate: 20.0

    constraints:
      # How long after the trajectory ends the controller waits
      # for joints to settle before declaring failure
      goal_time: 0.6
      stopped_velocity_tolerance: 0.05
```

**Parameters that matter most:**

| Parameter | Effect if wrong |
|---|---|
| `joints` order | Joints move to each other's targets |
| `allow_partial_joints_goal` | `false` means every trajectory must name all six joints — single-joint testing impossible |
| `update_rate` | Too low and commands are missed between simulation steps |
| `goal_time` | Too tight and slow motions are reported as failures |
| `use_sim_time` | `false` with a simulator means time mismatch and erratic interpolation |

### 5.6 `robot_arm_control/launch/arm_control.launch.py`

```python
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
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        #
        # This MUST come before the gazebo action, or the ign process
        # inherits the old environment.
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
```

### 5.7 `joint_monitor.py`

```python
#!/usr/bin/env python3
"""Print the arm's joint positions in a readable table.

Subscribes to /joint_states and prints both radians and degrees.
Run this alongside any test to watch what the arm is actually doing.

    ros2 run robot_arm_control joint_monitor
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_ORDER = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']


class JointMonitor(Node):

    def __init__(self) -> None:
        super().__init__('joint_monitor')

        self.declare_parameter('rate_hz', 2.0)
        rate = float(self.get_parameter('rate_hz').value)

        self._latest = None

        # Must be stored on self, or Python garbage-collects the
        # subscription and the callback silently never fires.
        self._subscription = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)

        self._timer = self.create_timer(1.0 / rate, self._print_table)

        self.get_logger().info('Waiting for /joint_states...')

    def _on_joint_state(self, msg: JointState) -> None:
        self._latest = msg

    def _print_table(self) -> None:
        if self._latest is None:
            return

        # Message order is not guaranteed, so index by name.
        lookup = dict(zip(self._latest.name, self._latest.position))

        lines = ['', '  joint      radians    degrees',
                 '  ---------------------------------']
        for name in JOINT_ORDER:
            if name not in lookup:
                lines.append(f'  {name}    (not reported)')
                continue
            rad = lookup[name]
            lines.append(f'  {name}   {rad:+8.4f}   {math.degrees(rad):+8.2f}')

        print('\n'.join(lines))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 5.8 `test_single_joint.py`

```python
#!/usr/bin/env python3
"""Move one joint to one angle. The most basic control test.

Uses the FollowJointTrajectory action so we get a real result back
rather than firing a message into the void.

    ros2 run robot_arm_control test_single_joint --ros-args \\
      -p joint:=joint_2 -p angle_deg:=45.0 -p duration:=2.0

Angles are given in DEGREES for convenience and converted to
radians internally. ROS 2 itself always uses radians.
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

# Safety clamps, radians. Matches the URDF joint limits.
LIMITS = {
    'joint_1': (-3.14159, 3.14159),
    'joint_2': (-2.0944, 2.0944),
    'joint_3': (-2.6180, 2.6180),
    'joint_4': (-3.14159, 3.14159),
    'joint_5': (-2.0944, 2.0944),
    'joint_6': (-3.14159, 3.14159),
}


class SingleJointTest(Node):

    def __init__(self) -> None:
        super().__init__('test_single_joint')

        self.declare_parameter('joint', 'joint_2')
        self.declare_parameter('angle_deg', 45.0)
        self.declare_parameter('duration', 2.0)

        self._joint = str(self.get_parameter('joint').value)
        self._angle_deg = float(self.get_parameter('angle_deg').value)
        self._duration = float(self.get_parameter('duration').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def run(self) -> int:
        if self._joint not in LIMITS:
            self.get_logger().error(
                f'Unknown joint "{self._joint}". '
                f'Valid: {", ".join(LIMITS)}')
            return 1

        angle_rad = math.radians(self._angle_deg)
        low, high = LIMITS[self._joint]

        if not low <= angle_rad <= high:
            self.get_logger().error(
                f'{self._angle_deg:.1f} deg ({angle_rad:.4f} rad) is outside '
                f'the limits of {self._joint}: '
                f'{math.degrees(low):.1f} to {math.degrees(high):.1f} deg')
            return 1

        self.get_logger().info(f'Waiting for {ACTION_NAME}...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. Is arm_controller running? '
                'Check: ros2 control list_controllers')
            return 1

        point = JointTrajectoryPoint()
        point.positions = [angle_rad]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1.0) * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = [self._joint]
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f'Moving {self._joint} to {self._angle_deg:+.1f} deg '
            f'({angle_rad:+.4f} rad) over {self._duration:.1f} s')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the controller')
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._duration + 10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('No result returned (timed out)')
            return 1

        code = result.result.error_code
        if code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Motion completed successfully')
            return 0

        self.get_logger().error(
            f'Motion failed, error_code={code}: {result.result.error_string}')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SingleJointTest()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
```

**Why an action and not a topic:** publishing to `/arm_controller/joint_trajectory` fires a message with no feedback. The action returns a result, so the test can report success or failure and exit with a meaningful code.

### 5.9 `test_all_joints.py`

```python
#!/usr/bin/env python3
"""Exercise every joint in turn: move out, hold, return to zero.

Watch Gazebo while it runs and check each joint against the
design table.

    ros2 run robot_arm_control test_all_joints
    ros2 run robot_arm_control test_all_joints --ros-args -p angle_deg:=20.0
"""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

# joint name -> what you should see happen
EXPECTED = [
    ('joint_1', 'whole arm spins around the vertical axis'),
    ('joint_2', 'arm tilts forward/back from the shoulder'),
    ('joint_3', 'arm bends at the elbow'),
    ('joint_4', 'wrist twists around its own length'),
    ('joint_5', 'wrist bends'),
    ('joint_6', 'tool flange twists'),
]


class AllJointsTest(Node):

    def __init__(self) -> None:
        super().__init__('test_all_joints')

        self.declare_parameter('angle_deg', 30.0)
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('hold', 1.0)

        self._angle_deg = float(self.get_parameter('angle_deg').value)
        self._duration = float(self.get_parameter('duration').value)
        self._hold = float(self.get_parameter('hold').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _move(self, joint: str, angle_deg: float) -> bool:
        """Send one joint to one angle. Returns True on success."""
        point = JointTrajectoryPoint()
        point.positions = [math.radians(angle_deg)]
        point.velocities = [0.0]
        point.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1.0) * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = [joint]
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f'{joint}: goal rejected')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._duration + 10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error(f'{joint}: timed out')
            return False

        ok = result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        if not ok:
            self.get_logger().error(
                f'{joint}: failed, {result.result.error_string}')
        return ok

    def run(self) -> int:
        self.get_logger().info(f'Waiting for {ACTION_NAME}...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. '
                'Check: ros2 control list_controllers')
            return 1

        passed, failed = [], []

        for joint, description in EXPECTED:
            print()
            self.get_logger().info(f'=== {joint} ===')
            self.get_logger().info(f'Expect: {description}')

            ok = self._move(joint, self._angle_deg)
            time.sleep(self._hold)

            # Always return to zero, even if the outward move failed,
            # so the next joint starts from a known pose.
            ok = self._move(joint, 0.0) and ok
            time.sleep(0.3)

            (passed if ok else failed).append(joint)

        print()
        self.get_logger().info('================ SUMMARY ================')
        self.get_logger().info(f'Passed: {len(passed)}/6  {", ".join(passed)}')
        if failed:
            self.get_logger().error(f'Failed: {", ".join(failed)}')
            return 1

        self.get_logger().info('All joints moved successfully')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AllJointsTest()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
```

### 5.10 `test_trajectory.py`

```python
#!/usr/bin/env python3
"""Run a multi-waypoint trajectory with all six joints moving together.

Unlike test_all_joints, which moves one joint at a time, here every
joint is interpolated simultaneously between waypoints - which is
what a real motion looks like.

    ros2 run robot_arm_control test_trajectory
    ros2 run robot_arm_control test_trajectory --ros-args -p loops:=3
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']

# (label, [six angles in DEGREES], seconds from trajectory start)
WAYPOINTS = [
    ('home',        [  0,   0,   0,   0,   0,   0],  2.0),
    ('reach out',   [  0, -40,  70,   0,  35,   0],  5.0),
    ('swing left',  [ 60, -40,  70,   0,  35,   0],  8.0),
    ('twist wrist', [ 60, -40,  70,  90,  35,  45], 11.0),
    ('swing right', [-60, -30,  60, -90,  30, -45], 15.0),
    ('return home', [  0,   0,   0,   0,   0,   0], 19.0),
]


class TrajectoryTest(Node):

    def __init__(self) -> None:
        super().__init__('test_trajectory')

        self.declare_parameter('loops', 1)
        self._loops = int(self.get_parameter('loops').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def _build_trajectory(self) -> JointTrajectory:
        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        for label, degrees, t in WAYPOINTS:
            point = JointTrajectoryPoint()
            point.positions = [math.radians(d) for d in degrees]
            # Zero velocity at each waypoint means the arm settles
            # briefly. Omit for smoother continuous motion.
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            trajectory.points.append(point)

        return trajectory

    def run(self) -> int:
        self.get_logger().info(f'Waiting for {ACTION_NAME}...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Action server not available. '
                'Check: ros2 control list_controllers')
            return 1

        print()
        self.get_logger().info('Trajectory waypoints:')
        for label, degrees, t in WAYPOINTS:
            angles = ' '.join(f'{d:+4.0f}' for d in degrees)
            self.get_logger().info(f'  t={t:5.1f}s  [{angles}]  {label}')
        print()

        total = WAYPOINTS[-1][2]

        for loop in range(1, self._loops + 1):
            self.get_logger().info(f'--- Loop {loop}/{self._loops} ---')

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = self._build_trajectory()

            send_future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(
                self, send_future, timeout_sec=10.0)

            handle = send_future.result()
            if handle is None or not handle.accepted:
                self.get_logger().error('Goal rejected')
                return 1

            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=total + 15.0)

            result = result_future.result()
            if result is None:
                self.get_logger().error('Timed out waiting for result')
                return 1

            if result.result.error_code != \
                    FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().error(
                    f'Trajectory failed: {result.result.error_string}')
                return 1

            self.get_logger().info(f'Loop {loop} completed')

        self.get_logger().info('All trajectories completed successfully')
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryTest()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
```

### 5.11 `go_home.py`

```python
#!/usr/bin/env python3
"""Return all six joints to zero. Run this after any failed test.

    ros2 run robot_arm_control go_home
    ros2 run robot_arm_control go_home --ros-args -p duration:=5.0
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ACTION_NAME = '/arm_controller/follow_joint_trajectory'

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
               'joint_4', 'joint_5', 'joint_6']


class GoHome(Node):

    def __init__(self) -> None:
        super().__init__('go_home')

        self.declare_parameter('duration', 3.0)
        self._duration = float(self.get_parameter('duration').value)

        self._client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)

    def run(self) -> int:
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return 1

        point = JointTrajectoryPoint()
        point.positions = [0.0] * len(JOINT_NAMES)
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.time_from_start = Duration(
            sec=int(self._duration),
            nanosec=int((self._duration % 1.0) * 1e9),
        )

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES
        trajectory.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(f'Returning home over {self._duration:.1f} s')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Goal rejected')
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self._duration + 10.0)

        result = result_future.result()
        if result is None:
            self.get_logger().error('Timed out')
            return 1

        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Home position reached')
            return 0

        self.get_logger().error(f'Failed: {result.result.error_string}')
        return 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GoHome()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
```

### 5.12 `display.launch.py` — disable control

The slider GUI must not run with ros2_control enabled. Change one line:

```python
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' use_ros2_control:=false']),
        value_type=str,
    )
```

---

## 6. Build Procedure

```bash
cd ~/ros2_robot_arm_ws
colcon build --symlink-install
source install/setup.bash
```

### Verify the model parses both ways

```bash
# With control - grep count must be NON-ZERO
xacro src/robot_arm_description/urdf/robot.urdf.xacro use_ros2_control:=true \
  > /tmp/robot_control.urdf
check_urdf /tmp/robot_control.urdf
grep -c "ros2_control" /tmp/robot_control.urdf

# Without control - grep count must be ZERO
xacro src/robot_arm_description/urdf/robot.urdf.xacro use_ros2_control:=false \
  > /tmp/robot_plain.urdf
grep -c "ros2_control" /tmp/robot_plain.urdf
```

If the second returns a non-zero number, the `<xacro:if>` is not working and the slider GUI will fight with `joint_state_broadcaster`.

### Verify the executables registered

```bash
ros2 pkg list | grep robot_arm_control
ros2 pkg executables robot_arm_control
```

Expected — all five:

```
robot_arm_control go_home
robot_arm_control joint_monitor
robot_arm_control test_all_joints
robot_arm_control test_single_joint
robot_arm_control test_trajectory
```

If the package appears but the executables do not, the `entry_points` block in `setup.py` is missing them, or the package was not rebuilt after editing `setup.py`.

---

## 7. Staged Verification

Verify each stage before proceeding to the next.

### Launch

```bash
cd ~/ros2_robot_arm_ws
source install/setup.bash
ros2 launch robot_arm_control arm_control.launch.py
```

**Read the launch output.** The critical line to watch for is its *absence*:

```
[ign-1] [Err] [SystemLoader.cc:94] Failed to load system plugin
        [libign_ros2_control-system.so] : couldn't find shared library.
```

If that appears, stop. Nothing downstream will work. See error E.2.

Success looks like the spawners completing within a second or two rather than looping on "waiting for service".

### Stage 1 — Hardware interfaces

```bash
# second terminal
source ~/ros2_robot_arm_ws/install/setup.bash
ros2 control list_hardware_interfaces
```

Expected — 6 command, 18 state:

```
command interfaces
        joint_1/position [available] [claimed]
        joint_2/position [available] [claimed]
        joint_3/position [available] [claimed]
        joint_4/position [available] [claimed]
        joint_5/position [available] [claimed]
        joint_6/position [available] [claimed]
state interfaces
        joint_1/effort
        joint_1/position
        joint_1/velocity
        ... through joint_6
```

| Status | Meaning |
|---|---|
| `[claimed]` | A controller owns this interface. Correct. |
| `[unclaimed]` | No controller took it — `arm_controller` did not start |
| `[unavailable]` | The hardware interface itself failed to initialise |

### Stage 2 — Controllers

```bash
ros2 control list_controllers
```

Expected:

```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
arm_controller        [joint_trajectory_controller/JointTrajectoryController] active
```

Both must say **active**. `inactive` or missing means the spawner failed — scroll back through the launch output for the reason.

### Stage 3 — Joint states flowing

```bash
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
```

Expected: all six joints listed, publishing at roughly 50 Hz.

**This is the moment the arm stops collapsing.** Look at the Gazebo window — it should now stand upright and hold position instead of folding under gravity. That visual change is the single clearest proof the whole chain works.

### Stage 4 — Action server

```bash
ros2 action list
ros2 action info /arm_controller/follow_joint_trajectory
```

Expected: the action appears with exactly one action server.

---

## 8. Test Suite

Keep the monitor running in its own terminal throughout:

```bash
ros2 run robot_arm_control joint_monitor
```

### Test 1 — Single joint, valid angle

```bash
ros2 run robot_arm_control test_single_joint --ros-args \
  -p joint:=joint_2 -p angle_deg:=45.0 -p duration:=2.0
```

Expected: the shoulder tilts, the monitor settles at `+0.7854` rad / `+45.00` deg, and the node prints "Motion completed successfully" with exit code 0.

If it lands short — say 43° — the controller is hitting `goal_time` before the joint arrives. Either increase `duration` or raise `goal_time` in `controllers.yaml`.

### Test 2 — Limit rejection

```bash
ros2 run robot_arm_control test_single_joint --ros-args \
  -p joint:=joint_2 -p angle_deg:=200.0
```

Expected — rejected before anything is sent:

```
[ERROR] 200.0 deg (3.4907 rad) is outside the limits of joint_2:
        -120.0 to 120.0 deg
[ros2run]: Process exited with failure 1
```

Exit code 1 is **correct here**. A rejected command should fail loudly rather than silently doing nothing. This proves the `LIMITS` table matches the URDF and the check runs before the action call.

### Test 3 — Unknown joint

```bash
ros2 run robot_arm_control test_single_joint --ros-args -p joint:=joint_9
```

Expected: rejected with the list of valid joint names.

### Test 4 — Every joint in turn

```bash
ros2 run robot_arm_control test_all_joints
ros2 run robot_arm_control test_all_joints --ros-args -p angle_deg:=20.0
```

Watch Gazebo and check each joint against the printed expectation:

| Joint | Expected motion | Fails if |
|---|---|---|
| joint_1 | Whole arm spins around vertical axis | Arm tips sideways → axis should be `0 0 1` |
| joint_2 | Arm tilts forward/back from shoulder | Arm spins → axis should be `0 1 0` |
| joint_3 | Arm bends at elbow, upper arm stays put | Whole arm moves → wrong parent link |
| joint_4 | Wrist twists around its own length | Wrist bends → axis should be `0 0 1` |
| joint_5 | Wrist bends | Wrist twists → axis should be `0 1 0` |
| joint_6 | Tool flange twists at the tip | Nothing visible → check TF instead |

Ends with a pass/fail summary. Exit code 0 only if 6/6 passed.

### Test 5 — Coordinated trajectory

```bash
ros2 run robot_arm_control test_trajectory
ros2 run robot_arm_control test_trajectory --ros-args -p loops:=3
```

Expected: all six joints move **simultaneously** through five waypoints, smoothly, no jerking or pausing between them.

Jerking between waypoints means they are too close in time — increase the `time_from_start` spacing in `WAYPOINTS`.

### Test 6 — Return home

```bash
ros2 run robot_arm_control go_home
ros2 run robot_arm_control go_home --ros-args -p duration:=5.0
```

Run this after any failed test to get back to a known pose.

### Test 7 — Raw topic, no script

```bash
ros2 topic pub --once /arm_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
    points: [{positions: [0.5, -0.3, 0.8, 0.0, 0.4, 0.0],
              time_from_start: {sec: 3}}]}"
```

Proves the controller responds to raw messages, not just the test scripts. Note that positions here are **radians**, not degrees — the scripts do the conversion, raw messages do not.

### Test 8 — Controller lifecycle

```bash
ros2 control set_controller_state arm_controller inactive
ros2 control list_controllers
```

The arm should collapse — deactivating the controller releases the joints. Then:

```bash
ros2 control set_controller_state arm_controller active
ros2 run robot_arm_control go_home
```

Confirms controllers can be stopped and restarted without relaunching everything.

### Test 9 — Effort inspection

```bash
ros2 topic echo /joint_states --once
```

Look at the `effort` array. Joints holding against gravity should report non-zero torque. `joint_2` carries the most load and should show the largest magnitude.

### Test 10 — Controller state

```bash
ros2 topic echo /arm_controller/controller_state --once
```

Shows the controller's internal view: desired position, actual position, and the error between them. Persistent large error means the joint cannot keep up with the commanded trajectory.

---

## 9. Test Summary

| # | Test | Command | Pass condition |
|---|---|---|---|
| C.1 | Hardware interfaces | `ros2 control list_hardware_interfaces` | 6 command, 18 state, all claimed |
| C.2 | Controllers | `ros2 control list_controllers` | Both active |
| C.3 | Joint states | `ros2 topic hz /joint_states` | ~50 Hz |
| C.4 | Arm holds position | Watch Gazebo | Stands upright, no collapse |
| C.5 | Action server | `ros2 action list` | Action present, one server |
| C.6 | Single joint | `test_single_joint` | Reaches target, exit 0 |
| C.7 | Limit rejection | `test_single_joint -p angle_deg:=200` | Rejected before sending, exit 1 |
| C.8 | Unknown joint | `test_single_joint -p joint:=joint_9` | Rejected with valid list |
| C.9 | All joints | `test_all_joints` | 6/6 passed |
| C.10 | Trajectory | `test_trajectory` | Smooth simultaneous motion |
| C.11 | Return home | `go_home` | All joints reach 0.0 |
| C.12 | Raw topic | `ros2 topic pub ...` | Arm moves |
| C.13 | Lifecycle | `set_controller_state inactive` | Arm collapses, then recovers |
| C.14 | Effort | `echo /joint_states` | Non-zero effort under load |
| C.15 | Controller state | `echo /arm_controller/controller_state` | Small position error |

---

## 10. Error Reference

### Debugging order

```
1. Read the launch output for plugin load errors
2. ros2 control list_controllers      are controllers active?
3. ros2 control list_hardware_interfaces   are interfaces claimed?
4. ros2 topic hz /joint_states        is state flowing?
5. ros2 action list                   is the action server up?
6. ros2 topic echo /arm_controller/controller_state
```

### Plugin and startup

| Symptom | Cause | Fix |
|---|---|---|
| `Failed to load system plugin ... couldn't find shared library` | `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` not set | See error E.2 below |
| `controller manager not available` (loops forever) | Plugin never loaded, so no controller_manager exists | Fix the plugin load error first |
| `list_controllers` returns nothing | Same root cause | `grep ros2_control /tmp/robot_control.urdf` |
| Plugin package missing | `ign_ros2_control` not installed | `sudo apt install ros-humble-ign-ros2-control` |
| Spawner ran too early | Event handler missing or misordered | Ctrl-C, relaunch |

### Controllers

| Symptom | Cause | Fix |
|---|---|---|
| Controllers active, arm still falls | Interfaces `[unclaimed]` | `arm_controller` failed to start |
| `Action server not available` | `arm_controller` inactive | `ros2 control list_controllers` |
| Nothing moves, no error at all | `/clock` not bridged | Check `clock_bridge` node is running |
| Joints move to wrong targets | Joint order mismatch | Order in `controllers.yaml` must match trajectory `joint_names` |
| Only some joints respond | `allow_partial_joints_goal: false` | Set it to `true` |

### Trajectories

| Symptom | Cause | Fix |
|---|---|---|
| Arm jerks between waypoints | Waypoints too close in time | Increase `time_from_start` spacing |
| `goal_time` violation | Arm cannot reach in the allotted time | Increase duration, or raise `goal_time` |
| Motion overshoots then corrects | Zero velocity not set at waypoints | Add `point.velocities = [0.0] * n` |
| Joint stops short of target | Effort limit too low for the load | Raise `effort` in the URDF joint limit |
| Goal rejected immediately | Position outside command interface min/max | Check `ros2_control.xacro` limits |

### Model

| Symptom | Cause | Fix |
|---|---|---|
| `unbound prefix` at line 3 | A `xacro:` tag above the `<robot>` tag | Move it inside |
| ros2_control tags absent from output | `<xacro:if>` evaluated false | Pass `use_ros2_control:=true` |
| ros2_control tags present when unwanted | `display.launch.py` not passing false | Fix the launch file |
| Macro not found | `<xacro:include>` missing or wrong path | Check the include line |

### Environment

| Symptom | Cause | Fix |
|---|---|---|
| `Package 'robot_arm_control' not found` | Workspace not sourced in this terminal | `source install/setup.bash` |
| Executables missing from `ros2 pkg executables` | `entry_points` incomplete, or not rebuilt | Fix `setup.py`, rebuild, re-source |
| Search path shows `~/ros2_humble/install/...` | A ROS 2 source build is also sourced | Remove that line from `.bashrc` |

---

## 11. Errors Encountered During This Build

Real failures from this project, with the diagnosis that resolved each.

### E.1 — Unbound prefix in the xacro

**Symptom:**

```
XML parsing error: unbound prefix: line 3, column 2
Error: Error document empty.
ERROR: Model Parsing the xml failed
```

**Cause:** the ros2_control block was pasted at the **top** of `robot.urdf.xacro`, above the `<robot>` tag. The `xmlns:xacro` namespace is declared on that tag, so any `xacro:` element appearing before it has no namespace in scope.

**Diagnosis:**

```bash
head -5 src/robot_arm_description/urdf/robot.urdf.xacro
grep -n "<robot" src/robot_arm_description/urdf/robot.urdf.xacro
```

If `<robot` appears at a line number greater than 2, something was inserted above it.

**Fix:** move the block to just before the closing `</robot>`. The `<robot>` tag must be the second line of the file, immediately after `<?xml version="1.0"?>`.

**Lesson:** in xacro, position within the file is not cosmetic. Two hard rules: nothing with a `xacro:` prefix above `<robot>`, and the ros2_control block after all joints are defined.

### E.2 — Plugin not found despite being installed

**Symptom:**

```
[ign-1] [Err] [SystemLoader.cc:94] Failed to load system plugin
        [libign_ros2_control-system.so] : couldn't find shared library.
[spawner-5] waiting for service /controller_manager/list_controllers
        to become available...
```

followed by an endless loop of "Could not contact service".

**Cause:** Gazebo does not search ROS 2's library path. It looks in `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`, which was unset. The library existed the whole time.

**Diagnosis:**

```bash
dpkg -l | grep ign-ros2-control
find /opt/ros/humble -name "libign_ros2_control-system.so"
echo $IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

The package is installed and the file exists, but the variable is empty. That combination points at a search-path problem, not a missing package.

**Fix:**

```bash
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
```

Permanent fix — add to the launch file, **before** the gazebo action:

```python
SetEnvironmentVariable(
    name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
    value='/opt/ros/humble/lib',
),
```

Order matters. Set after `gazebo` and the `ign` process inherits the old environment.

**Lesson:** "not found" can mean "not installed" or "installed but not on the search path". Check for the file before reinstalling the package — the two failures look identical in the log but have completely different fixes.

### E.3 — Package not found in a second terminal

**Symptom:**

```
Package 'robot_arm_control' not found
```

in a terminal where the same command had worked minutes earlier in another window.

**Cause:** sourcing is per-terminal. A new terminal knows nothing about the workspace.

**Diagnosis:** read the search path the error prints. If your workspace path is absent from that list, the workspace is not sourced. This distinguishes it from a genuine build failure, where the path is present but the package is not.

**Fix:**

```bash
cd ~/ros2_robot_arm_ws
source install/setup.bash
```

**Lesson:** the search path in the error message is the diagnostic. Reading it tells you in one glance whether to source or to build.

### E.4 — Missing `meshes` directory after clone

**Symptom:**

```
CMake Error: ament_cmake_symlink_install_directory() can't find
'.../robot_arm_description/meshes'
```

**Cause:** git does not track empty directories. `CMakeLists.txt` listed `meshes` in `install(DIRECTORY ...)`, but the directory vanished on clone because it contained nothing.

**Fix:**

```bash
mkdir -p src/robot_arm_description/meshes
touch src/robot_arm_description/meshes/.gitkeep
```

**Lesson:** every directory named in `install(DIRECTORY ...)` needs at least one tracked file. A `.gitkeep` placeholder costs nothing and prevents a build failure on every fresh clone.

### E.5 — Workflow push rejected

**Symptom:**

```
! [remote rejected] main -> main (refusing to allow a Personal Access
Token to create or update workflow `.github/workflows/ci.yml`
without `workflow` scope)
```

**Cause:** GitHub treats files under `.github/workflows/` as privileged. A token with only `contents` permission cannot write them.

**Fix:** add **Workflows: Read and write** to the token's repository permissions.

**Lesson:** authentication succeeding does not mean authorisation for everything. The push uploaded all objects successfully and was rejected only at the final ref update.

---

## Commit

```bash
cd ~/ros2_robot_arm_ws
git status
git add .
git commit -m "Add ros2_control with Gazebo Fortress integration and joint test suite"
git push
```

```
PROJECT STATUS

✓ 6-DOF model, RViz, Gazebo spawn
✓ ros2_control hardware interface (IgnitionSystem)
✓ joint_state_broadcaster
✓ joint_trajectory_controller
✓ Arm holds position against gravity
✓ Joint state monitor
✓ Single-joint test with limit checking
✓ All-joints sweep test
✓ Multi-waypoint trajectory test
✓ Return-home recovery
□ Forward kinematics
□ Inverse kinematics
□ Workspace analysis
```
