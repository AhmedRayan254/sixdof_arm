# Industrial Design and Sensor Suite

Reference for the visual/mechanical redesign of the 6-DOF arm and the six sensor types layered on top of it. Covers what changed, what deliberately did not, how each component is built, and how to verify none of it broke the control stack.

## Contents

1. [Design Brief](#1-design-brief)
2. [The Core Insight](#2-the-core-insight)
3. [Industrial Body](#3-industrial-body)
4. [Materials in Gazebo Fortress](#4-materials-in-gazebo-fortress)
5. [The Custom World](#5-the-custom-world)
6. [Sensor Suite](#6-sensor-suite)
7. [Files Changed](#7-files-changed)
8. [Verification](#8-verification)
9. [Errors Encountered](#9-errors-encountered)

---

## 1. Design Brief

Make the arm look like a modern professional industrial robot while keeping the ROS 2 software architecture completely unchanged.

### What changed

- Every link's `<visual>` geometry, from one cylinder to 6–11 composed primitives
- Colours, from flat RViz materials to a Fortress-native two-tone industrial scheme
- A custom world with sensor system plugins and proper lighting
- Six new sensor types

### What did not change

| Preserved | Value |
|---|---|
| Degrees of freedom | 6, all revolute |
| Link names | `base_link`, `link_1` … `link_6`, `end_effector` |
| Joint names | `joint_1` … `joint_6`, `world_to_base`, `link_6_to_end_effector` |
| Joint hierarchy | Unchanged parent/child chain |
| Joint axes | Z, Y, Y, Z, Y, Z |
| Joint limits | ±180°, ±120°, ±150°, ±180°, ±120°, ±180° |
| Effort limits | 60, 60, 40, 20, 15, 10 N·m |
| Velocity limits | 2.0, 2.0, 2.5, 3.0, 3.0, 3.0 rad/s |
| Collision geometry | One cylinder per link, identical dimensions |
| Mass and inertia | Identical tensors |
| Reach | 1.080 m, `base_link` to `end_effector` |
| ros2_control interfaces | 6 command, 18 state |
| Controllers | `joint_state_broadcaster`, `arm_controller` |
| Test nodes | All five untouched |

The arm is the same robot from the software's point of view. Every existing test passes without modification.

---

## 2. The Core Insight

**Visual geometry has no effect on physics in Gazebo.** Only `<collision>` and `<inertial>` do.

This is what makes the redesign low-risk. A link can carry eleven visual primitives and still have exactly one collision cylinder and one inertia tensor. The physics engine never sees the extra detail, so:

- Stability is unchanged — no new vibration, no solver instability
- Real-time factor is unchanged — visuals are render-only
- Forward kinematics is unchanged — joint origins were never touched
- Collision behaviour is unchanged — the same single cylinder per link

The rule followed throughout: **detailed visual, simple collision.** Complex collision geometry makes contact solving slow and unstable. Simple convex primitives keep physics fast.

### Multiple visuals per link

URDF permits multiple `<visual>` elements in one link. On conversion to SDF each becomes a separately named visual (`link_2_visual`, `link_2_visual_1`, …) and Fortress renders them all.

```xml
<link name="link_2">
  <!-- PHYSICS: one collision, one inertial -->
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <geometry><cylinder radius="0.050" length="0.30"/></geometry>
  </collision>
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.00827" ixy="0" ixz="0"
             iyy="0.00827" iyz="0" izz="0.00125"/>
  </inertial>

  <!-- VISUAL: as many primitives as the look needs -->
  <visual name="link_2_tube">    ... </visual>
  <visual name="link_2_housing"> ... </visual>
  <visual name="link_2_motor">   ... </visual>
  <visual name="link_2_encoder"> ... </visual>
  <visual name="link_2_bearing"> ... </visual>
  <visual name="link_2_cable">   ... </visual>
  <visual name="link_2_plate">   ... </visual>
</link>
```

Primitives were chosen over imported meshes so the repository stays free of binary files, every dimension remains parametric, and resizing the arm still takes one number change.

---

## 3. Industrial Body

### Component macros

All in `urdf/industrial_parts.xacro`. Every macro emits `<visual>` elements only.

| Macro | Represents | Geometry |
|---|---|---|
| `structural_tube` | Load-bearing link body | Cylinder at 82% of collision radius |
| `joint_housing` | Casting wrapping the joint | Cylinder at 152% radius, aligned to joint axis |
| `bearing_ring` | Bearing cover / parting line | Thin ring at 156% radius |
| `motor_can` | Servo + harmonic gearbox | Offset cylinder, coaxial with joint |
| `encoder_puck` | Absolute encoder | Small disc on motor rear face |
| `mounting_plate` | Interface plate | Thin cylinder at link's distal end |
| `cable_run` | External conduit | Thin offset cylinder along the link |
| `bolt_ring` | Six-bolt pattern | Six small cylinders at 60° spacing |

### Housing alignment

A joint housing must align with the **joint axis**, not the link axis. Two orientations cover all six joints:

```xml
<xacro:property name="axis_z" value="0 0 0"/>       <!-- roll:  joints 1, 4, 6 -->
<xacro:property name="axis_y" value="1.5708 0 0"/>  <!-- pitch: joints 2, 3, 5 -->
```

Getting this wrong is visually obvious — the housing sits perpendicular to the joint it wraps.

### Proportions

| Ratio | Value | Reason |
|---|---|---|
| Housing / link radius | 1.52 | Housings read as separate castings |
| Motor / link radius | 1.16 | Visible bulge without dominating |
| Bearing / link radius | 1.56 | Slightly proud of the housing |
| Tube / collision radius | 0.82 | Tube recedes behind the housings |

### Base and tool flange

**Base** (11 visuals): bolted floor plate, cast pedestal, graphite collar, blue accent ring, rear connector block, six-bolt circle.

**Tool flange** (on `link_6`, 11 visuals): wrist housing, bearing ring, barrel, ISO 9409-1 style flat plate, central pilot boss, six-bolt circle.

The flange is modelled on `link_6` rather than `end_effector` so the massless frame stays massless and the SDF conversion has nothing to warn about.

---

## 4. Materials in Gazebo Fortress

### Gazebo Classic syntax does not work

```xml
<!-- SILENTLY DOES NOTHING in Fortress -->
<gazebo reference="link_2">
  <material>Gazebo/Orange</material>
</gazebo>
```

The `//gazebo/material` script-name form applies only to Gazebo Classic. Fortress uses the Blinn-Phong model with `<ambient>`, `<diffuse>`, `<specular>`, `<emissive>`. The symptom of using the old syntax is a robot that renders grey or white in Gazebo while looking correct in RViz.

### What does work

**1. URDF `<color rgba>`** — works in both RViz and Fortress. The `sdformat_urdf` converter renders solid colour as `0.4 × ambient + 0.8 × diffuse`. Limited to solid colours; no `<script>`, `<shader>` or `<pbr>` allowed here.

```xml
<material name="link_2_tube_mat">
  <color rgba="0.529 0.522 0.506 1.0"/>
</material>
```

**2. Fortress-native `<gazebo>` block** — the only path for specular and PBR.

```xml
<gazebo reference="link_2">
  <visual>
    <material>
      <specular>0.90 0.90 0.92 1.0</specular>
      <emissive>0 0 0 1</emissive>
      <pbr>
        <metal>
          <metalness>0.85</metalness>
          <roughness>0.38</roughness>
        </metal>
      </pbr>
    </material>
  </visual>
</gazebo>
```

This project uses both: per-visual URDF colours drive the two-tone scheme, and a `metal_finish` macro adds specular/PBR per link.

### Palette

| Name | RGBA | Used for |
|---|---|---|
| `c_shell` | `0.529 0.522 0.506 1.0` | Proximal link shells (UR silver) |
| `c_shell_lt` | `0.640 0.636 0.622 1.0` | Wrist shells, mounting plates |
| `c_housing` | `0.130 0.133 0.140 1.0` | Joint housings (graphite) |
| `c_motor` | `0.075 0.078 0.082 1.0` | Motor cans, connector block |
| `c_accent` | `0.043 0.541 0.780 1.0` | Bearing rings, base ring |
| `c_steel` | `0.760 0.770 0.780 1.0` | Tool flange, floor plate |
| `c_bolt` | `0.290 0.300 0.310 1.0` | Bolt heads |
| `c_cable` | `0.045 0.045 0.050 1.0` | Cable conduits |
| `c_base` | `0.220 0.225 0.232 1.0` | Base pedestal |
| `c_encoder` | `0.620 0.180 0.110 1.0` | Encoder pucks |

### Why colours are inlined per visual

Named materials declared once and referenced by name are not guaranteed to survive URDF → SDF conversion. Inlining `<color rgba>` into each `<visual>` resolves reliably in Fortress. The palette lives as xacro properties, so the values are still defined in one place.

### Reference industrial colours

For adapting the scheme to a different brand look. RAL → RGBA conversions are approximations; RAL is a physical paint standard.

| Brand | RAL | RGBA |
|---|---|---|
| Fanuc yellow | 1021 | `0.965 0.714 0.0 1.0` |
| ABB orange | 2003 | `0.965 0.471 0.161 1.0` |
| ABB graphite | 7035 | `0.773 0.780 0.769 1.0` |
| KUKA orange | 2003 | `0.965 0.471 0.161 1.0` |
| KUKA black | 9005 | `0.05 0.05 0.05 1.0` |
| UR silver | 9007 | `0.529 0.522 0.506 1.0` |
| Franka white | — | `0.902 0.922 0.929 1.0` |

---

## 5. The Custom World

`worlds/arm_world.sdf` exists for one required reason and two cosmetic ones.

**Required:** sensor system plugins must be declared at world level.

```xml
<plugin filename="ignition-gazebo-imu-system"
        name="ignition::gazebo::systems::Imu"/>
<plugin filename="ignition-gazebo-forcetorque-system"
        name="ignition::gazebo::systems::ForceTorque"/>
<plugin filename="ignition-gazebo-contact-system"
        name="ignition::gazebo::systems::Contact"/>
```

Fortress has a known defect where a force-torque sensor declared only inside a xacro-loaded model never publishes. Declaring the system plugin in the world is the reliable path.

**Cosmetic:** a directional sun plus a point fill light so the metallic surfaces actually read as metallic, and a grey ground plane instead of the default.

Both launch files default to this world. Override with `world:=empty.sdf` — but the IMU and force-torque sensors will not publish without the plugins.

---

## 6. Sensor Suite

Six sensor types, split by integration risk.

### Simulated in Gazebo

Need SDF `<sensor>` blocks, world-level system plugins, and `ros_gz_bridge` entries. Additive — they create no links or frames and do not alter ros2_control.

| Sensor | Topic | ROS type | Mount |
|---|---|---|---|
| IMU | `/imu` | `sensor_msgs/Imu` | `link_6` |
| Force-torque | `/wrist_ft` | `geometry_msgs/Wrench` | `joint_6` |

Bridge strings:

```
/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU
/wrist_ft@geometry_msgs/msg/Wrench[ignition.msgs.Wrench
```

`[` means Gazebo → ROS 2 only. The force-torque sensor reports the wrench in the joint's **child** frame, measured child-to-parent.

**Deliberately not registered as ros2_control sensor state interfaces.** Doing so would change `ros2 control list_hardware_interfaces` away from the 6 command / 18 state contract the test suite depends on.

### Derived in ROS 2

Pure subscribers on `/joint_states`. No Gazebo plugin, no bridge, no way to destabilise the control loop.

#### Encoders — `encoder_simulator.py`

In `ros2_control` the joint position and velocity state interfaces **are** the encoder. `joint_state_broadcaster` already publishes them, perfectly noiseless.

A real absolute encoder is quantised to a finite count and carries noise. This node applies both and republishes on `/encoder_states`.

| Joint | Counts/rev | Bits |
|---|---|---|
| joint_1, joint_2 | 262144 | 18 |
| joint_3 | 131072 | 17 |
| joint_4–6 | 65536 | 16 |

**It never publishes to `/joint_states`.** Two publishers on that topic means `robot_state_publisher` receives contradictory angles and the TF tree flickers.

#### Motor current and temperature — `motor_diagnostics.py`

Fortress has no motor current or temperature sensor and does not need one. Both follow from the joint torque already on `/joint_states`.

```
current      I = τ / Kt
heating      P = I² · R
temperature  C · dT/dt = P − (T − T_ambient) / R_th
```

A standard first-order lumped thermal model. Time constants of 20–30 minutes are typical for industrial servos, so temperature climbs slowly and realistically under sustained load.

| Joint | Kt (N·m/A) | R (Ω) | C_th (J/°C) | R_th (°C/W) |
|---|---|---|---|---|
| joint_1, joint_2 | 1.16 | 1.10 | 900 | 1.30 |
| joint_3 | 0.83 | 1.60 | 620 | 1.70 |
| joint_4 | 0.45 | 2.40 | 330 | 2.40 |
| joint_5 | 0.36 | 2.90 | 260 | 2.80 |
| joint_6 | 0.28 | 3.40 | 200 | 3.20 |

Scaled with the URDF effort limits, so proximal joints carry higher torque constants and larger thermal mass.

Published:

```
/motor_current/<joint>       std_msgs/Float64          amperes
/motor_temperature/<joint>   sensor_msgs/Temperature   degrees C
/motor_diagnostics           diagnostic_msgs/DiagnosticArray
```

Diagnostic levels: OK below 70 °C, WARN at 70 °C, ERROR at 90 °C.

#### Limit and home switches — `limit_switches.py`

Real arms carry a hard limit switch near each end of travel and a home switch used during homing. Both are position-triggered, so a node watching `/joint_states` reproduces them exactly.

A Gazebo contact sensor is the wrong tool — it detects physical touch, not joint travel.

Defaults: switches trip 0.035 rad (2°) before the mechanical stop; the home window is 0.010 rad.

Published:

```
/limit_switch/<joint>/lower   std_msgs/Bool
/limit_switch/<joint>/upper   std_msgs/Bool
/home_switch/<joint>          std_msgs/Bool
/limit_switch_diagnostics     diagnostic_msgs/DiagnosticArray
```

### Integration risk summary

| Sensor | New Gazebo plugin | New bridge | Touches control path |
|---|---|---|---|
| Encoders | No | No | No |
| Motor current | No | No | No |
| Motor temperature | No | No | No |
| Limit/home switches | No | No | No |
| IMU | Yes (world) | Yes | No |
| Force-torque | Yes (world) | Yes | No |

---

## 7. Files Changed

### Modified

| File | Change |
|---|---|
| `robot_arm_description/CMakeLists.txt` | Install `worlds/` |
| `robot_arm_description/urdf/robot.urdf.xacro` | Composed visuals, includes, `use_sensors` arg |
| `robot_arm_description/launch/gazebo.launch.py` | Custom world, sensor bridge, plugin path |
| `robot_arm_control/package.xml` | Added `std_msgs`, `geometry_msgs`, `diagnostic_msgs` |
| `robot_arm_control/setup.py` | Three new entry points |
| `robot_arm_control/launch/arm_control.launch.py` | Sensor bridge, sensor nodes, `use_sensors` arg |

### Created

| File | Purpose |
|---|---|
| `urdf/materials.xacro` | Colour palette + `metal_finish` macro |
| `urdf/industrial_parts.xacro` | Eight visual component macros |
| `urdf/sensors.xacro` | IMU and force-torque sensor blocks |
| `worlds/arm_world.sdf` | World + sensor system plugins + lighting |
| `robot_arm_control/encoder_simulator.py` | Quantised encoder view |
| `robot_arm_control/motor_diagnostics.py` | Current + thermal model |
| `robot_arm_control/limit_switches.py` | End-of-travel and home switches |

### Untouched

`urdf/ros2_control.xacro`, `launch/display.launch.py`, `rviz/display.rviz`, `config/controllers.yaml`, `robot_arm_description/package.xml`, and all five original test nodes.

---

## 8. Verification

The redesign is only valid if the robot is unchanged from the software's point of view.

### Before launching

```bash
cd ~/ros2_robot_arm_ws
colcon build --symlink-install
source install/setup.bash

# All five xacro files must be present in install/, not just src/
ls install/robot_arm_description/share/robot_arm_description/urdf/

xacro src/robot_arm_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
check_urdf /tmp/robot.urdf
```

`check_urdf` must print the same nine-link chain as before the redesign.

### Both xacro paths

```bash
# With control: non-zero
xacro src/robot_arm_description/urdf/robot.urdf.xacro \
  use_ros2_control:=true > /tmp/a.urdf
grep -c "ros2_control" /tmp/a.urdf

# Without: must be zero, or the slider GUI fights joint_state_broadcaster
xacro src/robot_arm_description/urdf/robot.urdf.xacro \
  use_ros2_control:=false > /tmp/b.urdf
grep -c "ros2_control" /tmp/b.urdf
```

### After launching

```bash
ros2 launch robot_arm_control arm_control.launch.py
```

```bash
# Reach — MUST read 1.080 m, not 1.020
ros2 run tf2_ros tf2_echo base_link end_effector

# TF tree — 9 frames, one unbroken chain
ros2 run tf2_tools view_frames

# Controllers — both active
ros2 control list_controllers

# Interfaces — 6 command, 18 state, all [claimed]
ros2 control list_hardware_interfaces

# Sensors present
ros2 topic list | grep -E "motor|limit|home|imu|wrist_ft|encoder"
```

### Regression suite

Every existing test must pass without modification.

```bash
ros2 run robot_arm_control joint_monitor &
ros2 run robot_arm_control test_single_joint --ros-args \
  -p joint:=joint_2 -p angle_deg:=45.0 -p duration:=2.0
ros2 run robot_arm_control test_all_joints
ros2 run robot_arm_control test_trajectory
ros2 run robot_arm_control go_home
```

### Checklist

| Check | Pass condition |
|---|---|
| `check_urdf` | Nine-link chain, same as before |
| Reach | `1.080 m` exactly |
| TF frames | 9, unbroken |
| Controllers | Both `active` |
| Interfaces | 6 command, 18 state, `[claimed]` |
| Collisions per link | Exactly 1 |
| Masses | 2.0 / 1.2 / 1.0 / 0.8 / 0.4 / 0.3 / 0.2 |
| Real-time factor | Unchanged from before the redesign |
| Arm holds position | Stands upright, no collapse |
| Existing tests | All pass unmodified |
| Sensor topics | All present and publishing |

A drop in real-time factor would indicate a collision or inertia regression. Visuals cannot cause it.

---

## 9. Errors Encountered

Real failures from this redesign, with the diagnosis that resolved each.

### E.1 — XML comment parse failure

**Symptom:** `not well-formed (invalid token)` when parsing `industrial_parts.xacro` and `robot.urdf.xacro`.

**Cause:** comment separators used `---`. XML forbids `--` anywhere inside a comment. `<!--` and `-->` are the only legal occurrences.

**Fix:** replaced all runs of three or more hyphens with `=`.

**Lesson:** decorative separators inside XML comments must not use hyphens.

### E.2 — Reach figure was wrong in the documentation

**Symptom:** `tf2_echo base_link end_effector` reported 1.080 m where the docs claimed 1.020 m.

**Cause:** a documentation error, not a model error. 1.020 m is the sum of the six link lengths. It excludes the 0.06 m base pedestal. `base_link → end_effector` includes it.

```
link_1 origin -> end_effector : 1.020 m
base_link     -> end_effector : 1.080 m
```

**Fix:** corrected the figure in the README, all docs and every verification checklist.

**Lesson:** a verification constant must itself be verified. This one was carried unchecked from Phase 1 and would have caused a false alarm at exactly the moment it mattered most.

### E.3 — Plugin installed but not found

**Symptom:**

```
[Err] [SystemLoader.cc:94] Failed to load system plugin
      [libign_ros2_control-system.so] : couldn't find shared library.
```

followed by the spawner waiting forever for a controller_manager that never started.

**Cause:** Gazebo does not search ROS 2's library path. It reads `IGN_GAZEBO_SYSTEM_PLUGIN_PATH`, which was unset. The library existed the whole time.

**Diagnosis:**

```bash
dpkg -l | grep ign-ros2-control          # installed
find /opt/ros/humble -name "libign_ros2_control-system.so"   # exists
echo $IGN_GAZEBO_SYSTEM_PLUGIN_PATH      # empty  <- the real problem
```

**Fix:** `SetEnvironmentVariable` in both launch files, placed **before** the gazebo action or the `ign` process inherits the old environment.

**Lesson:** "not found" can mean "not installed" or "installed but not on the search path". The two look identical in the log and need completely different fixes.

### E.4 — Backup folders broke the build

**Symptom:**

```
ERROR:colcon:colcon build: Duplicate package names not supported:
- robot_arm_control:
  - backup_20260924_172832/src/robot_arm_control
  - backup_20260924_173245/src/robot_arm_control
  - src/robot_arm_control
```

**Cause:** the apply script wrote its backups inside the workspace. colcon scans every directory under the workspace root and found three copies of each package.

**Fix:** move backups outside the workspace.

```bash
mv backup_* ~/
```

**Lesson:** anything placed inside a colcon workspace is a build candidate. Backups, archives and scratch copies belong outside it.

### E.5 — `$(find ...)` resolves to install, not src

**Symptom:** `No such file or directory: .../install/robot_arm_description/share/robot_arm_description/urdf/materials.xacro` — while the file plainly existed in `src/`.

**Cause:** `$(find package)` in xacro resolves to the **installed** share directory. New files do not reach `install/` until `colcon build` copies them.

**Fix:** build before validating.

**Lesson:** after adding any new file referenced by `$(find ...)`, rebuild before running `xacro` or `ros2 launch`. Even with `--symlink-install`, a file that has never been installed has no symlink to follow.
