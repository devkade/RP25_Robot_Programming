# System Architecture

This document describes the high-level architecture of the gazebo_exam package.

## Overview

The gazebo_exam package is built on a multi-layered architecture that separates robot description, motion planning, control, and simulation concerns.

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    User Applications                         │
│  (moveit2_practice_sample, step1_practice, step2_hw)        │
└────────────────────────┬────────────────────────────────────┘
                         │ MoveGroupInterface API
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                    MoveIt2 Layer                             │
│  ┌──────────────┐  ┌───────────────┐  ┌─────────────────┐  │
│  │ move_group   │  │ Planning Scene│  │ Trajectory Exec │  │
│  │   (OMPL)     │  │  Interface    │  │   Manager       │  │
│  └──────────────┘  └───────────────┘  └─────────────────┘  │
└────────────────────────┬────────────────────────────────────┘
                         │ FollowJointTrajectory action
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                  ROS2 Control Layer                          │
│  ┌──────────────────────┐  ┌──────────────────────────┐    │
│  │ controller_manager   │  │ JointTrajectoryController│    │
│  │  (100 Hz update)     │  │  (arm + gripper)         │    │
│  └──────────────────────┘  └──────────────────────────┘    │
└────────────────────────┬────────────────────────────────────┘
                         │ Hardware interface
                         ▼
┌─────────────────────────────────────────────────────────────┐
│              Hardware Abstraction Layer                      │
│  ┌──────────────────┐         ┌────────────────────────┐   │
│  │ GazeboSystem     │   OR    │ GenericSystem (mock)   │   │
│  │  (simulation)    │         │  (visualization only)  │   │
│  └──────────────────┘         └────────────────────────┘   │
└────────────────────────┬────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│                   Gazebo Physics Engine                      │
│  (collision detection, dynamics, visualization)              │
└─────────────────────────────────────────────────────────────┘
```

## Component Overview

### 1. Robot Description Layer

**URDF (Unified Robot Description Format)**
- Defines physical robot structure (links, joints, inertias)
- Uses xacro macros for modularity and parameter loading
- Separate files for FR3 arm and Franka Hand gripper
- Configuration YAML files for kinematics, limits, dynamics

**SRDF (Semantic Robot Description Format)**
- Defines planning groups ("manipulator", "gripper")
- Specifies disabled collision pairs
- Pre-defined named states (HomePos, ManipulationPos)
- End-effector definitions

**Key Files:**
- `urdf/robots/fr3/fr3.urdf.xacro` - Main robot URDF
- `urdf/robots/fr3/fr3_macro.xacro` - Robot macro definitions
- `urdf/end_effectors/franka_hand/` - Gripper URDF
- `srdf/fr3.srdf.xacro` - Semantic description

### 2. Motion Planning Layer (MoveIt2)

**move_group Node**
- Central planning server
- Integrates multiple planning algorithms (OMPL)
- Collision checking
- Inverse kinematics (KDL solver)
- Trajectory execution management

**Planning Scene Interface**
- Manages collision objects
- Adds/removes obstacles dynamically
- Ground plane, boxes, cylinders for pick-and-place

**MoveGroupInterface (C++ API)**
- User-facing API for applications
- `setPoseTarget()` - Cartesian goal
- `setJointValueTarget()` - Joint space goal
- `plan()` - Compute trajectory
- `execute()` - Send to controller

**Key Configuration:**
- `moveit_config/ompl_planning.yaml` - 11 OMPL planners
- `moveit_config/kinematics.yaml` - KDL IK solver (3 attempts, 0.05s timeout)
- `moveit_config/joint_limits_6dof_gripper.yaml` - Acceleration limits

### 3. Control Layer (ros2_control)

**controller_manager**
- Manages controller lifecycle
- Updates at 100 Hz
- Routes commands to hardware interface

**Controllers:**
- `manipulator_controller` - JointTrajectoryController for 7-DOF arm
- `gripper_controller` - JointTrajectoryController for gripper
- `joint_state_broadcaster` - Publishes joint states

**Hardware Interfaces:**
- `GazeboSystem` - For simulation (when sim_gazebo:=true)
- `GenericSystem` - For mock hardware (visualization only)

**Key Configuration:**
- `controller/fr3_controller_gripper.yaml` - Controller definitions
- `urdf/robots/fr3/fr3_ros2_control.xacro` - Hardware interface spec

### 4. Simulation Layer (Gazebo)

**Gazebo Server**
- Physics engine (ODE or Bullet)
- Collision detection
- Dynamics simulation
- Gravity, friction, contact forces

**Gazebo Client**
- 3D visualization
- Camera views
- Interactive manipulation

**gazebo_ros2_control Plugin**
- Bridges Gazebo and ros2_control
- Simulates joint actuators
- Publishes sensor feedback

### 5. Visualization Layer (RViz2)

**RViz2 Displays:**
- Robot model (URDF visualization)
- Planning scene (obstacles, collision objects)
- Trajectory visualization (planned paths)
- Interactive markers (goal pose editing)

**MoveIt Motion Planning Plugin:**
- Interactive planning interface
- Goal pose manipulation
- Plan/execute buttons
- Configuration space visualization

**Key Configuration:**
- `rviz/moveit_franka.rviz` - MoveIt planning config
- `rviz/display_franka.rviz` - Basic visualization config

## Data Flow

### Motion Planning Request Flow

```
1. User Application
   └─> MoveGroupInterface.setPoseTarget(target_pose)
   └─> MoveGroupInterface.plan()

2. MoveIt2 move_group
   └─> Inverse Kinematics (KDL)
       └─> Converts Cartesian pose to joint angles
   └─> Motion Planning (OMPL RRTConnect)
       └─> Samples configuration space
       └─> Checks collisions
       └─> Builds trajectory
   └─> Time Parametrization
       └─> Adds velocity/acceleration profiles
   └─> Returns Plan

3. User Application
   └─> MoveGroupInterface.execute(plan)

4. MoveIt2 Trajectory Execution
   └─> Sends FollowJointTrajectory action to controller

5. ros2_control
   └─> JointTrajectoryController interpolates points
   └─> Sends position commands to hardware interface

6. Gazebo (if simulation)
   └─> Simulates joint actuators
   └─> Updates physics
   └─> Publishes joint states

7. Feedback Loop
   └─> joint_state_broadcaster publishes /joint_states
   └─> MoveIt2 monitors execution
   └─> RViz visualizes current state
```

## Configuration Cascade

The system uses a layered configuration approach:

### Level 1: URDF Robot Description
- Physical parameters (kinematics, inertias, visual/collision meshes)
- Joint limits (position, velocity, effort)
- Dynamics (damping, friction)

**Source:** `urdf/robots/fr3/config/*.yaml`

### Level 2: SRDF Semantic Description
- Planning groups and chains
- Disabled collisions
- Named states

**Source:** `srdf/*.srdf.xacro`

### Level 3: MoveIt2 Configuration
- Kinematics solver settings
- Motion planner algorithms
- Trajectory execution parameters
- Joint acceleration limits (for planning)

**Source:** `moveit_config/*.yaml`

### Level 4: Controller Configuration
- Controller update rates
- Command/state interfaces
- Trajectory tolerances
- Goal thresholds

**Source:** `controller/*.yaml`

## Integration Points

### URDF ↔ MoveIt2
- MoveIt2 reads robot_description parameter
- Builds kinematic model from URDF
- Uses SRDF for planning groups and collision matrix

### MoveIt2 ↔ ros2_control
- MoveIt sends FollowJointTrajectory actions
- Controllers defined in `controllers_6dof_gripper.yaml`
- Action interface: `/manipulator_controller/follow_joint_trajectory`

### ros2_control ↔ Gazebo
- `gazebo_ros2_control` plugin simulates hardware
- Joint commands → Gazebo joint controllers
- Gazebo joint states → ROS joint_states topic

### Application ↔ MoveIt2
- C++ API: `moveit::planning_interface::MoveGroupInterface`
- Planning requests via ROS services
- Execution via actions
- State monitoring via topics

## Module Dependencies

```
moveit2_practice_sample.cpp
├── Depends on: rclcpp
├── Depends on: moveit_ros_planning_interface
├── Depends on: tf2_geometry_msgs
└── Requires running: move_group node

step2_hw.cpp
├── Depends on: rclcpp
├── Depends on: moveit_ros_planning_interface
├── Depends on: moveit_planning_scene_interface
└── Creates: PickAndLiftNode class with threading

Launch Files
├── display_franka.launch.py
│   └── No MoveIt2, no Gazebo
├── gazebo_franka.launch.py
│   └── No MoveIt2, includes Gazebo + controllers
├── moveit2_franka.launch.py
│   └── Includes: gazebo_franka.launch.py + move_group
├── step1_practice_gazebo.launch.py
│   └── Custom: Gazebo + objects + executable
└── step2_hw_gazebo.launch.py
    └── Custom: Gazebo + 8 objects + executable
```

## Design Patterns

### 1. Macro-Based Configuration (Xacro)
- Reusable robot macros
- Parameter-driven instantiation
- Separation of structure and configuration

### 2. Layered Abstraction
- Hardware abstraction (ros2_control)
- Planning abstraction (MoveIt2)
- User-facing API (MoveGroupInterface)

### 3. Event-Driven Launch
- Sequential node spawning
- Event handlers (OnProcessStart, OnProcessExit)
- Ensures correct initialization order

### 4. Planning Scene Management
- Collision objects added dynamically
- Ground plane for safety
- Object removal after manipulation

### 5. Modular Launch Files
- Base launches (display, gazebo, moveit2)
- Composed launches (include base + add features)
- Parameter-driven behavior

## Extensibility Points

### Adding New Demonstrations
1. Add function to `moveit2_practice_sample.cpp`
2. Update `cmd` parameter handling
3. Rebuild and test

### Adding New Objects
1. Create xacro file in `models/`
2. Add spawn command in launch file
3. Configure position, orientation, dimensions

### Changing Robot Model
1. Update URDF xacro files
2. Regenerate SRDF (collision matrix)
3. Update kinematics configuration
4. Recalibrate MoveIt2 parameters

### Adding New Controllers
1. Define in `controller/*.yaml`
2. Add to ros2_control spawner in launch file
3. Update MoveIt controller mappings

## Thread and Process Model

### Nodes and Threads

**move_group Node:**
- Main thread: ROS spinning
- Planning thread: OMPL planning
- Execution thread: Trajectory monitoring

**Gazebo:**
- Physics update thread (100 Hz default)
- Rendering thread
- ROS communication thread

**User Applications:**
- Main thread: ROS node
- Optional worker thread (step2_hw uses threading)

### Synchronization
- Action-based execution (FollowJointTrajectory)
- Service-based planning requests
- Topic-based state feedback
- Launch event handlers for startup sequencing

## Performance Characteristics

### Planning Time
- Simple poses: 0.1-1 second
- Complex paths: 1-10 seconds
- Default timeout: 45 seconds (configurable)

### Control Loop Rates
- controller_manager: 100 Hz
- Gazebo physics: 100 Hz (default)
- joint_state_broadcaster: 50 Hz
- Action monitoring: 20 Hz

### Memory Usage
- Base system: ~500 MB
- With Gazebo: ~1.5 GB
- With RViz: +200-500 MB
- Per additional object: ~10-50 MB

## Security Considerations

- No authentication/authorization (educational package)
- Localhost communication only
- No remote control interfaces
- File permissions standard (rw-rw-rw-)

## Known Limitations

1. **Joint 3 and 5 have restricted ranges** (see URDF)
2. **IK solver may fail for unreachable poses** (3 attempts, 0.05s timeout)
3. **Gazebo simulation slower than real-time** with many objects
4. **No tactile sensing** - gripper force not sensed
5. **Self-collision disabled** for some adjacent links (performance optimization)
