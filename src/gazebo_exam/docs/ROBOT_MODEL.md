# Robot Model Documentation

This document provides detailed information about the FR3 robot model and its configuration.

## Robot Overview

**Model:** Franka Emika FR3 Research Robot
**Type:** 7-DOF Collaborative Robot Arm
**End Effector:** Franka Hand (Parallel Gripper)
**Total Mass:** ~18.4 kg
**Workspace:** ~850mm radius (approximate)

## URDF File Structure

### File Hierarchy

```
urdf/
├── robots/fr3/
│   ├── fr3.urdf.xacro              # Main URDF entry point
│   ├── fr3_macro.xacro             # Robot structure macro
│   ├── fr3_ros2_control.xacro      # Hardware interface definition
│   ├── fr3_transmission.xacro      # Joint transmission (if used)
│   └── config/
│       ├── joint_limits.yaml       # Position, velocity, effort limits
│       ├── kinematics.yaml         # DH parameters, joint origins
│       ├── inertials.yaml          # Mass and inertia tensors
│       ├── visual_parameters.yaml  # Mesh file references
│       ├── dynamics.yaml           # Damping and friction
│       └── initial_positions.yaml  # Default joint configuration
└── end_effectors/franka_hand/
    ├── franka_hand.urdf.xacro      # Gripper entry point
    ├── franka_hand_macro.xacro     # Gripper structure
    └── franka_hand_ros2_control.xacro  # Gripper control interface
```

### Xacro Parameters

The main URDF accepts these parameters:

```xml
<xacro:arg name="prefix" default=""/>
<xacro:arg name="name" default="fr3"/>
<xacro:arg name="sim_gazebo" default="false"/>
<xacro:arg name="use_fake_hardware" default="true"/>
```

**sim_gazebo:**
- `true` → Uses GazeboSystem plugin for physics simulation
- `false` → Uses GenericSystem (mock hardware for visualization)

## Kinematic Chain

### Link Structure

```
world (fixed frame)
  └─ global_link (fixed)
      └─ link0 (base)
          └─ joint0 (revolute)
              └─ link1
                  └─ joint1 (revolute)
                      └─ link2
                          └─ joint2 (revolute)
                              └─ link3
                                  └─ joint3 (revolute)
                                      └─ link4
                                          └─ joint4 (revolute)
                                              └─ link5
                                                  └─ joint5 (revolute)
                                                      └─ link6
                                                          └─ joint6 (revolute)
                                                              └─ link7 (tool flange)
                                                                  └─ hand (gripper)
                                                                      ├─ left_finger
                                                                      └─ right_finger
```

### Joint Specifications

| Joint | Type | Axis | Position Limits (rad) | Velocity Limit (rad/s) | Effort Limit (Nm) |
|-------|------|------|----------------------|------------------------|-------------------|
| joint0 | Revolute | Z | [-2.74, 2.74] | 2.62 | 87 |
| joint1 | Revolute | Z | [-1.78, 1.78] | 2.62 | 87 |
| joint2 | Revolute | Z | [-2.90, 2.90] | 2.62 | 87 |
| joint3 | Revolute | Z | [-3.04, -0.15] | 2.62 | 87 |
| joint4 | Revolute | Z | [-2.81, 2.81] | 5.26 | 12 |
| joint5 | Revolute | Z | [0.54, 4.52] | 4.18 | 12 |
| joint6 | Revolute | Z | [-3.02, 3.02] | 5.26 | 12 |

**Note:** Joint3 and Joint5 have restricted ranges compared to full circle rotation.

### Gripper Joint

| Joint | Type | Range (m) | Description |
|-------|------|-----------|-------------|
| finger_joint1 | Prismatic | [0.0, 0.07] | Parallel gripper opening |

- 0.0 m = Fully closed
- 0.07 m = Fully open (70mm width)
- Mimic mechanism for parallel gripper (both fingers move symmetrically)

## Kinematics

### DH Parameters (Modified Denavit-Hartenberg)

Extracted from `kinematics.yaml`:

| Joint | a (m) | d (m) | α (rad) | θ offset (rad) |
|-------|-------|-------|---------|----------------|
| joint0 | 0 | 0.333 | 0 | 0 |
| joint1 | 0 | 0 | -π/2 | 0 |
| joint2 | 0.0825 | 0 | π/2 | 0 |
| joint3 | -0.0825 | 0.384 | π/2 | 0 |
| joint4 | 0 | 0 | -π/2 | 0 |
| joint5 | 0.088 | 0 | π/2 | 0 |
| joint6 | 0 | 0.107 | π/2 | 0 |

### End-Effector Transform

From link7 to gripper origin:
- **Translation:** [0, 0, 0.1034] m
- **Rotation:** yaw = -π/4 rad (45° about Z-axis)

### TCP (Tool Center Point)

The end-effector frame (fr3_hand_tcp or similar) is located at the center of the gripper, between the fingers.

## Inertial Properties

### Link Masses

Extracted from `inertials.yaml`:

| Link | Mass (kg) | Center of Mass Offset (m) |
|------|-----------|---------------------------|
| link0 | 2.4 | [0, 0, 0.05] |
| link1 | 2.9 | [0, -0.03, 0.12] |
| link2 | 2.9 | [0, 0.03, 0.03] |
| link3 | 2.2 | [0.04, 0, 0.03] |
| link4 | 2.6 | [-0.04, 0.04, 0] |
| link5 | 2.3 | [0, 0.02, 0.08] |
| link6 | 1.8 | [0.06, 0, 0.01] |
| link7 | 0.2 | [0, 0, 0.02] |

**Total Arm Mass:** ~17.3 kg
**Gripper Mass:** ~1.1 kg
**Total Robot Mass:** ~18.4 kg

### Inertia Tensors

Each link has a 3x3 inertia tensor defined in `inertials.yaml`. These are CAD-derived values for accurate dynamics simulation.

Example for link0:
```yaml
inertia:
  ixx: 0.0573
  ixy: 0.0
  ixz: 0.0
  iyy: 0.0573
  iyz: 0.0
  izz: 0.0573
```

## Dynamics

### Joint Dynamics

From `dynamics.yaml`:

**All Joints:**
- **Damping:** 0.1 Ns/rad
- **Friction:** 10 N (Coulomb friction)

These values affect simulation behavior but don't reflect exact hardware values.

## Visual and Collision Meshes

### Visual Meshes (DAE Format)

Located in `meshes/robot_arms/fr3/visual/`:

- link0.dae through link7.dae
- Detailed CAD geometry
- Includes colors and materials
- Used in RViz and Gazebo visualization

### Collision Meshes (STL Format)

Located in `meshes/robot_arms/fr3/collision/`:

- link0.stl through link7.stl
- Simplified geometry
- Faster collision detection
- Used in motion planning and physics

### Gripper Meshes

Located in `meshes/robot_ee/franka_hand_white/` and `franka_hand_black/`:

- hand.dae / hand.stl
- finger.dae / finger.stl
- Color variants available (white/black)

## SRDF Configuration

### Planning Groups

**manipulator (arm):**
```xml
<group name="manipulator">
  <chain base_link="link0" tip_link="link7" />
</group>
```

**gripper:**
```xml
<group name="gripper">
  <joint name="finger_joint1" />
</group>
```

### Named States

**HomePos:**
```
joint0: 0.0
joint1: -1.5708 (−π/2 rad, vertical)
joint2: 0.0
joint3: 0.0
joint4: 1.5708 (π/2 rad)
joint5: 0.0
joint6: 0.0
```

**ManipulationPos:**
```
joint0: 0.3665
joint1: -0.1745
joint2: -0.4189
joint3: -1.3439
joint4: 0.0524
joint5: 1.5708
joint6: 0.7854
```

**ZeroPos:**
```
All joints: 0.0 (extended configuration)
```

### Disabled Collisions

Self-collision checking is disabled between adjacent links:
- link0 ↔ link1
- link1 ↔ link2
- link2 ↔ link3
- link3 ↔ link4
- link4 ↔ link5
- link5 ↔ link6
- link6 ↔ link7
- hand ↔ link6, link7

**Rationale:** Adjacent links in a serial chain don't collide due to kinematic constraints.

## ROS2 Control Configuration

### Hardware Interface

From `fr3_ros2_control.xacro`:

**Command Interfaces (per joint):**
- position
- velocity

**State Interfaces (per joint):**
- position
- velocity
- effort

**Initial Positions:**
Loaded from `initial_positions.yaml`:
```yaml
joint0: 0.0
joint1: -0.7854  # -45°
joint2: 0.0
joint3: -2.3562  # -135°
joint4: 0.0
joint5: 1.5708   # 90°
joint6: 0.7854   # 45°
```

### Gazebo Integration

When `sim_gazebo:=true`:
```xml
<plugin>gazebo_ros2_control/GazeboSystem</plugin>
```

This plugin:
- Simulates joint actuators
- Applies commanded positions/velocities
- Computes joint states from physics
- Handles effort (torque) computation

## Workspace Analysis

### Reachable Workspace

**Maximum Reach:**
- Sum of link lengths ≈ 0.333 + 0.316 + 0.384 + 0.088 + 0.107 + 0.103 ≈ 1.33 m (extended)
- Practical workspace radius: ~0.85 m

**Height Range:**
- Minimum: ~0.1 m above base
- Maximum: ~1.3 m above base

**Joint Restrictions:**
- Joint3: Limited to negative angles [-3.04, -0.15]
- Joint5: Cannot reach angles below 0.54 rad
- These restrictions create workspace voids

### Singular Configurations

The FR3 has singularities when:
1. **Shoulder singularity:** Joint2 = 0 (arm fully extended sideways)
2. **Elbow singularity:** Joint4 = 0 (elbow straight)
3. **Wrist singularity:** Joint5 = 0 or π (wrist axes aligned)

Avoid these configurations in motion planning.

## Coordinate Frames

### Base Frame
- **Origin:** Center of link0 base
- **X-axis:** Forward
- **Y-axis:** Left
- **Z-axis:** Up

### Tool Flange Frame (link7)
- **Position:** End of link6
- **Orientation:** Rotated by joint6

### Gripper Frame (hand)
- **Position:** [0, 0, 0.1034] from link7
- **Orientation:** -45° yaw from link7

### TCP Frame
- **Position:** Between gripper fingers
- **Typical offset:** ~0.1 m from hand origin

## Configuration Files Reference

### joint_limits.yaml
```yaml
joint0:
  has_position_limits: true
  min_position: -2.7437
  max_position: 2.7437
  has_velocity_limits: true
  max_velocity: 2.6180
  has_effort_limits: true
  max_effort: 87.0
```

### kinematics.yaml
```yaml
joint0:
  x: 0.0
  y: 0.0
  z: 0.333
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

### inertials.yaml
```yaml
link0:
  origin:
    xyz: [0.0, 0.0, 0.05]
    rpy: [0.0, 0.0, 0.0]
  mass: 2.4
  inertia:
    ixx: 0.0573
    ixy: 0.0
    # ... etc
```

### visual_parameters.yaml
```yaml
link0:
  visual:
    mesh: package://gazebo_exam/meshes/robot_arms/fr3/visual/link0.dae
  collision:
    mesh: package://gazebo_exam/meshes/robot_arms/fr3/collision/link0.stl
```

## Usage in Code

### Getting Current Pose

```cpp
#include <moveit/move_group_interface/move_group_interface.h>

auto move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");
geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
```

### Setting Joint Values

```cpp
std::vector<double> joint_values = {0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854};
move_group.setJointValueTarget(joint_values);
move_group.move();
```

### Gripper Control

```cpp
auto gripper_group = moveit::planning_interface::MoveGroupInterface(node, "gripper");
std::vector<double> gripper_open = {0.07};  // Open
std::vector<double> gripper_close = {0.0};  // Close
gripper_group.setJointValueTarget(gripper_open);
gripper_group.move();
```

## Validation Tools

### Check URDF Validity

```bash
check_urdf urdf/robots/fr3/fr3.urdf.xacro
```

### View URDF Tree

```bash
urdf_to_graphiz urdf/robots/fr3/fr3.urdf.xacro
```

### Visualize Transform Tree

```bash
ros2 launch gazebo_exam display_franka.launch.py
ros2 run tf2_tools view_frames
```

## Common Issues

### Issue: "IK solver failed"

**Causes:**
- Target pose unreachable
- Near singularity
- Joint limit violations

**Solutions:**
- Check workspace limits
- Adjust target orientation
- Use joint space goals instead

### Issue: "Self-collision detected"

**Causes:**
- Target configuration causes collision
- Collision meshes too conservative

**Solutions:**
- Update disabled_collisions in SRDF
- Simplify collision meshes
- Adjust target pose

### Issue: "Joint limits exceeded"

**Causes:**
- Target requires joint3 or joint5 beyond limits
- Inverse kinematics found invalid solution

**Solutions:**
- Check joint limits in URDF
- Try different approach angles
- Use constrained planning
