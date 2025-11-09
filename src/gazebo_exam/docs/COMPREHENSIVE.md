# Gazebo Exam - Comprehensive Codebase Documentation

## Executive Summary

**gazebo_exam** is a ROS 2 robotics package implementing motion planning and simulation for the FR3 collaborative robot arm (7 DOF) with Franka Hand gripper end-effector. The package integrates MoveIt2 for motion planning, Gazebo for physics simulation, and includes educational exercise materials for learning robotics manipulation tasks.

**Key Statistics:**
- Total files: 71
- C++ source files: 3
- Python launch files: 8
- Xacro configuration files: 16
- YAML configuration files: 14
- Mesh files: 25 (DAE and STL formats)

---

## 1. Directory Structure and Organization

```
gazebo_exam/
├── CMakeLists.txt                 # Build configuration
├── package.xml                     # Package manifest
├── src/                           # C++ source code
│   ├── moveit2_practice_sample.cpp # Educational example with 6 practice scenarios
│   ├── step1_practice.cpp          # Homework assignment (empty template)
│   └── step2_hw.cpp                # Advanced assignment: pick-and-place task
├── launch/                        # ROS2 launch files (8 files)
│   ├── display_franka.launch.py          # RViz visualization only
│   ├── gazebo_franka.launch.py           # Gazebo simulator with controllers
│   ├── moveit2_franka.launch.py          # Gazebo + MoveIt2 (no execution)
│   ├── moveit2_practice.launch.py        # MoveIt2 + step1_practice executable
│   ├── step1_practice.launch.py          # MoveIt2 + step1_practice executable (alt)
│   ├── step1_practice_gazebo.launch.py   # Gazebo + step1_practice + multiple objects
│   ├── step2_hw.launch.py                # MoveIt2 + step2_hw executable
│   └── step2_hw_gazebo.launch.py         # Gazebo + step2_hw + 8 simulation objects
├── urdf/                          # Robot description files
│   ├── robots/fr3/
│   │   ├── fr3.urdf.xacro         # Main FR3 robot URDF
│   │   ├── fr3_macro.xacro        # FR3 robot macro definitions
│   │   ├── fr3_ros2_control.xacro # ROS2 control interface
│   │   ├── fr3_transmission.xacro # Joint transmission definitions
│   │   └── config/                # Configuration YAML files
│   │       ├── joint_limits.yaml       # Joint position/velocity limits
│   │       ├── kinematics.yaml         # DH parameters and joint origins
│   │       ├── inertials.yaml          # Link masses and inertia tensors
│   │       ├── visual_parameters.yaml  # Mesh file references
│   │       ├── dynamics.yaml           # Damping and friction parameters
│   │       └── initial_positions.yaml  # Default joint positions
│   └── end_effectors/franka_hand/
│       ├── franka_hand.urdf.xacro      # Gripper URDF
│       ├── franka_hand_macro.xacro     # Gripper macro with links/joints
│       └── franka_hand_ros2_control.xacro  # Gripper control interface
├── srdf/                          # Semantic robot description files
│   ├── fr3.srdf.xacro             # Main SRDF (includes macros)
│   ├── fr3_macro.srdf.xacro       # FR3 semantic information
│   └── franka_hand_macro.srdf.xacro   # Gripper semantic information
├── moveit_config/                 # MoveIt2 configuration
│   ├── kinematics.yaml            # IK solver configuration (KDL)
│   ├── controllers_6dof_gripper.yaml   # MoveIt controller mappings
│   ├── joint_limits_6dof_gripper.yaml  # MoveIt acceleration limits
│   └── ompl_planning.yaml          # OMPL planner configurations
├── controller/                    # ROS2 control configurations
│   ├── fr3_controller.yaml         # Arm-only controller config
│   └── fr3_controller_gripper.yaml # Arm + gripper controller config
├── meshes/                        # Visual and collision meshes
│   ├── robot_arms/fr3/
│   │   ├── visual/    (7 DAE files: link0-link7)
│   │   └── collision/ (7 STL files: link0-link7)
│   └── robot_ee/
│       ├── franka_hand_white/ (hand + finger meshes)
│       └── franka_hand_black/  (hand + finger meshes)
├── models/                        # Simulated object models for exercises
│   ├── box_red.xacro              # Red box with parametric dimensions
│   ├── box_blue.xacro             # Blue box (for differentiation)
│   ├── cylinder.xacro             # Cylindrical object
│   ├── triangle.xacro             # Triangular/wedge object
│   └── case.xacro                 # Container/case mesh model
├── rviz/                          # RViz configuration files
│   ├── moveit_franka.rviz         # MoveIt2 visualization config
│   └── display_franka.rviz        # Basic URDF visualization config
├── include/moveit2_exam/          # C++ header files (if any custom ones exist)
└── README.md (implied)            # Package documentation
```

---

## 2. C++ Executables and Source Code

### 2.1 moveit2_practice_sample.cpp (Educational Reference Implementation)
**Location:** `/home/kangdongkyu/colcon_ws/src/gazebo_exam/src/moveit2_practice_sample.cpp`
**Purpose:** Educational material demonstrating MoveIt2 APIs and robot control patterns
**Dependencies:** rclcpp, moveit_ros_planning_interface, tf2_geometry_msgs

**Key Components:**

1. **Utility Functions:**
   - `list_to_pose()` - Converts (x,y,z,roll,pitch,yaw) to geometry_msgs::Pose with quaternion
   - `go_to_pose_goal()` - Plans and executes end-effector pose target
   - `gripper_sample()` - Controls gripper finger joints

2. **Movement Demonstrations (6 scenarios via --cmd parameter):**
   - **cmd=1: move_sample()** - Moves end-effector along Y, X, and Z axes (incremental)
     - Demonstrates relative position adjustments
     - Starting pose: (0.4, 0.2, 0.5) with orientation -π/2 roll
   
   - **cmd=2: rotation_sample()** - Rotates end-effector around pitch axis
     - Three rotation steps: 0° → 0.3 rad → -0.3 rad → 0°
     - Fixed position: (0.34, 0.2, 0.3)
   
   - **cmd=3: gripper_sample()** - Opens and closes gripper
     - Open: finger_joint1 = 0.07 m
     - Close: finger_joint1 = 0.0 m
   
   - **cmd=4: waypoint_sample()** - Cartesian path planning
     - Computes continuous trajectory through waypoints
     - Uses computeCartesianPath() API
     - Path: down → horizontal → return
   
   - **cmd=5: rectangle_sample()** - HOMEWORK PLACEHOLDER
     - Students should implement rectangle trajectory
     - Reference: waypoint_sample()
   
   - **cmd=6: circle_sample()** - HOMEWORK PLACEHOLDER
     - Students should implement circular trajectory
     - Reference: waypoint_sample()

**Main Function Flow:**
1. Initializes ROS2 node with simulated time
2. Creates MoveGroupInterface for "manipulator" and "gripper"
3. Sets planning time to 45.0 seconds
4. Reads "cmd" parameter and executes corresponding scenario

---

### 2.2 step1_practice.cpp (Student Assignment Template)
**Location:** `/home/kangdongkyu/colcon_ws/src/gazebo_exam/src/step1_practice.cpp`
**Purpose:** Empty starter template for Step 1 assignment
**Status:** INCOMPLETE - Students must implement motion planning logic
**Expected Implementation:** 
- Likely duplicates moveit2_practice_sample.cpp logic
- May include variations for learning purposes

---

### 2.3 step2_hw.cpp (Advanced Pick-and-Place Assignment)
**Location:** `/home/kangdongkyu/colcon_ws/src/gazebo_exam/src/step2_hw.cpp`
**Purpose:** Complex manipulation task: pick block from one location, place at another
**Architecture:** Object-oriented design with PickAndLiftNode class
**Key Dependencies:** rclcpp, moveit_planning_interface, geometric_msgs, tf2_geometry_msgs

**Custom Data Classes:**

1. **Location Class**
   ```cpp
   struct Location {
     double x, y;  // 2D grid coordinates for object placement
   };
   ```

2. **Block Class**
   ```cpp
   struct Block {
     double width, length, height, radius;  // Dimensions (radius unused for boxes)
     Location location;                      // Position in workspace
   };
   ```

**Core Operations:**

1. **initial_pose()** - Move arm to safe configuration
   - Joint values: {0, -2.03, 1.58, -1.19, -1.58, 0.78}

2. **lift_block()** - Pick operation
   - Approach pose above block at z=0.4m
   - Descend to contact height (block.height + gripper_offset)
   - Close gripper with specified closing force
   - Lift back to approach height

3. **place_block()** - Placement operation
   - Move to target location at working height
   - Descend to block stack height
   - Open gripper to release
   - Return to approach height

4. **open_gripper() / close_gripper()** - Gripper control
   - Open: finger_joint1 = 0.07m
   - Close: finger_joint1 = closing_value (default 0.0)

**Main Execution:**
- Spawns two 5cm³ blocks at (0.4, 0.1) and (0.4, -0.1)
- Task: Lift first block and place at second block location
- Uses PickAndLiftNode::run() in separate thread

**Planning Scene:**
- Adds ground plane collision object at z = -0.005m
- Collision checking enabled for safety

---

## 3. Launch Files and Their Purposes

### 3.1 Visualization-Only Launches

**display_franka.launch.py**
- **Purpose:** Static URDF visualization without simulation
- **Components:**
  - robot_state_publisher: Publishes transform tree from URDF
  - joint_state_publisher_gui: Interactive joint slider interface
  - rviz2: 3D visualization with display_franka.rviz config
- **Use Case:** Exploring robot structure, testing URDF validity
- **No Simulation:** Gazebo not included

---

### 3.2 Gazebo Physics Simulation (without MoveIt2)

**gazebo_franka.launch.py**
- **Purpose:** Pure Gazebo simulation with controller interfaces
- **Architecture:**
  1. Launches Gazebo physics engine
  2. Spawns FR3 robot into simulation
  3. Starts ROS2 control components:
     - ros2_control_node: Interface to joint controllers
     - joint_state_broadcaster: Publishes joint states
     - manipulator_controller: JointTrajectoryController for arm
     - gripper_controller: JointTrajectoryController for gripper
  4. Optional RViz2 visualization
- **Controllers:** Uses fr3_controller_gripper.yaml (7 arm joints + gripper)
- **Use Case:** Testing low-level control, physics interactions
- **No MoveIt2:** Motion planning not available

---

### 3.3 Gazebo + MoveIt2 (No Execution Demo)

**moveit2_franka.launch.py**
- **Purpose:** Full MoveIt2 stack with physics simulation
- **Components:**
  1. Includes gazebo_franka.launch.py for simulation base
  2. Launches move_group node with:
     - URDF robot description (sim_gazebo=true)
     - SRDF semantic information (fr3.srdf.xacro)
     - Kinematics solver configuration (KDL)
     - Joint planning limits
     - OMPL motion planner (RRTConnect, PRM, etc.)
     - MoveItSimpleControllerManager for trajectory execution
  3. RViz2 with MoveIt planning interface (moveit_franka.rviz)
- **MoveIt Configuration:**
  - Planning group: "manipulator" (7 DOF arm) + "gripper" (1 DOF)
  - IK solver: kdl_kinematics_plugin
  - Collision detection: Disabled for gripper internal collisions
  - Joint acceleration limits: 204 rad/s² (arm), 230 rad/s² (gripper)
- **No Auto-Execution:** RViz used for manual planning
- **Use Case:** Interactive motion planning visualization

---

### 3.4 Gazebo + MoveIt2 + Automation Demos

**moveit2_practice.launch.py**
- **Purpose:** Run step1_practice executable with MoveIt2
- **Components:**
  1. Launches full MoveIt2 stack
  2. Spawns step1_practice C++ node after move_group ready
  3. No Gazebo physics (planning-only)
- **Configuration:** Identical MoveIt2 setup to moveit2_franka.launch.py
- **Executable:** step1_practice binary
- **Use Case:** Testing student homework with planning simulation

**step1_practice.launch.py**
- **Purpose:** Alternative launch for step1_practice with MoveIt2
- **Differences:** Identical to moveit2_practice.launch.py
- **Use Case:** Redundant alternative (both launch step1_practice binary)

---

### 3.5 Gazebo + Object Models + Demonstrations

**step1_practice_gazebo.launch.py**
- **Purpose:** Step 1 assignment with simulated manipulation objects
- **Simulation Objects Spawned:**
  1. "box1": Red box at (0.4, 0.1, 0.03) - 5cm cube
  2. "box2": Blue box at (0.4, -0.1, 0.03) - 5cm cube
- **Execution:** Runs step1_practice executable
- **Physics:** Full Gazebo physics simulation
- **Use Case:** Learning manipulation with visual feedback

**step2_hw_gazebo.launch.py**
- **Purpose:** Advanced assignment with rich simulation environment
- **Simulation Objects (8 total):**
  1. box1-box6: Red boxes at various positions with different heights
     - box1: (0.4, 0.1, 0.03) - 5cm standard
     - box2: (0.4, 0.0, 0.03) - 5cm standard
     - box3: (0.4, -0.1, 0.03) - 2.5cm narrower
     - box4: (0.5, -0.1, 0.035) - 7cm tall
     - box5: (0.5, 0.1, 0.04) - 8cm tall
     - box6: (0.6, 0.0, 0.045) - 9cm tall
  2. cylinder: (0.5, 0.0, 0.03) - radius 2.5cm, height 6cm
  3. triangle: (0.6, 0.1, 0.03) - wedge shape
  4. case: (0.0, 0.2, 0.0) - Container mesh
- **Execution:** Runs step2_hw executable
- **Complexity:** Heavy simulation for pick-and-place learning

**step2_hw.launch.py**
- **Purpose:** Run step2_hw without Gazebo (MoveIt2 only)
- **Components:**
  1. Full MoveIt2 stack
  2. step2_hw executable spawned after move_group ready
- **No Physics:** Planning simulation only
- **Use Case:** Development/testing before full simulation

---

## 4. Launch File Dependency Graph

```
display_franka.launch.py
  - robot_state_publisher (URDF)
  - joint_state_publisher_gui
  - rviz2

gazebo_franka.launch.py
  - gazebo (physics)
  - robot_state_publisher
  - ros2_control_node
  - joint_state_broadcaster → [OnProcessExit(gazebo_spawn_robot)]
  - manipulator_controller → [OnProcessExit(joint_state_broadcaster)]
  - gripper_controller → [OnProcessExit(joint_state_broadcaster)]
  - [optional] rviz2

moveit2_franka.launch.py
  - gazebo_franka.launch.py (includes all above)
  - move_group (MoveIt2)
  - rviz2 (with MoveIt interface)

moveit2_practice.launch.py
  - move_group (MoveIt2, no Gazebo)
  - step1_practice → [OnProcessStart(move_group)]

step1_practice_gazebo.launch.py
  - gazebo + controllers + objects
  - step1_practice → [OnProcessExit(gazebo_spawn_robot)]

step2_hw_gazebo.launch.py
  - gazebo + controllers + 8 objects
  - step2_hw → [OnProcessExit(gazebo_spawn_robot)]

step2_hw.launch.py
  - move_group (MoveIt2, no Gazebo)
  - step2_hw → [OnProcessStart(move_group)]
```

---

## 5. Robot Model Structure (URDF/Xacro Hierarchy)

### 5.1 FR3 Arm Structure

**File Hierarchy:**
```
fr3.urdf.xacro (root)
├── Includes: fr3_macro.xacro
├── Parameters:
│   ├── joint_limits.yaml (7 joint constraints)
│   ├── kinematics.yaml (DH parameters)
│   ├── inertials.yaml (link properties)
│   ├── visual_parameters.yaml (mesh refs)
│   └── initial_positions.yaml (default config)
├── Instantiates: xacro:fr3_robot
│   └── Creates: link0 - link7
│   └── Creates: joint0 - joint6
│   └── Includes: fr3_ros2_control.xacro
└── Attaches gripper via franka_hand_macro.xacro

fr3_macro.xacro (macro definitions)
├── Macro: fr3_robot params={prefix, name, parent, ...}
│   ├── Nested Macro: define_link (creates links with meshes/inertia)
│   │   ├── Loads YAML files
│   │   ├── Creates visual geometry (DAE meshes)
│   │   ├── Creates collision geometry (STL meshes)
│   │   └── Sets inertial parameters
│   ├── Nested Macro: define_joint (creates revolute joints)
│   │   ├── Reads kinematics YAML
│   │   ├── Sets joint limits
│   │   ├── Sets dynamics (damping, friction)
│   │   └── Defines axis (0, 0, 1)
│   ├── Calls define_link 8 times (link0-link7)
│   ├── Calls define_joint 7 times (joint0-joint6)
│   └── Includes: fr3_ros2_control.xacro

fr3_ros2_control.xacro
├── Defines: ros2_control system interface
├── Hardware:
│   ├── If sim_gazebo: gazebo_ros2_control/GazeboSystem plugin
│   └── Else: mock_components/GenericSystem (for testing)
├── Joint definitions for joint0-joint6:
│   ├── Command interfaces: position, velocity
│   ├── State interfaces: position, velocity, effort
│   ├── Limits: position [-6.28, 6.28], velocity [-3.15, 3.15]
│   └── Initial positions from YAML

franka_hand.urdf.xacro (gripper)
├── Includes: franka_hand_ros2_control.xacro
├── Links: hand, left_finger, right_finger
├── Meshes: DAE files (visual), STL files (collision)
├── Inertial properties per link
└── Gripper ros2_control interface (finger_joint1)
```

### 5.2 FR3 Kinematics Chain

**7-DOF Serial Chain:**
```
world → global[fixed] → link0 → joint0 → link1 → ... → link7 → [gripper attached]

Joint Parameters (from kinematics.yaml):
- joint0: pitch=0, yaw=0, shift=(0, 0, 0.333)
- joint1: pitch=-π/2, rotation to shift=(0, -0.316, 0)
- joint2: pitch=π/2, rotation to shift=(0.0825, 0, 0)
- joint3: pitch=π/2, rotation to shift=(-0.0825, 0.384, 0)
- joint4: pitch=-π/2, rotation to shift=(0, 0, 0)
- joint5: pitch=π/2, rotation to shift=(0.088, 0, 0)
- joint6: pitch=π/2, rotation to link7 frame

End-Effector Offset (to gripper origin):
- Position: (0, 0, 0.1034) relative to link7
- Orientation: yaw=-π/4 (45° rotation)
```

### 5.3 Joint Specifications

**Arm Joints (joint0-joint6):**
- Type: Revolute (continuous rotation)
- Axis: Z-axis in local frame
- Dynamics: damping=0.1 Ns/rad, friction=10 N
- Limits (from joint_limits.yaml):
  ```
  joint0: [-2.74, 2.74] rad, v_max=2.62, τ_max=87 Nm
  joint1: [-1.78, 1.78] rad, v_max=2.62, τ_max=87 Nm
  joint2: [-2.90, 2.90] rad, v_max=2.62, τ_max=87 Nm
  joint3: [-3.04, -0.15] rad, v_max=2.62, τ_max=87 Nm (restricted!)
  joint4: [-2.81, 2.81] rad, v_max=5.26, τ_max=12 Nm
  joint5: [0.54, 4.52] rad, v_max=4.18, τ_max=12 Nm (restricted!)
  joint6: [-3.02, 3.02] rad, v_max=5.26, τ_max=12 Nm
  ```

**Gripper Joints:**
- finger_joint1: Mimic control (parallel gripper)
- Range: [0, 0.07] m (fully closed to open)

### 5.4 Link Masses and Inertias

**Total Robot Mass:** ~18.4 kg

**Inertial Properties (from inertials.yaml):**
- link0: 2.4 kg (base) - highest mass
- link1: 2.9 kg
- link2: 2.9 kg
- link3: 2.2 kg
- link4: 2.6 kg
- link5: 2.3 kg
- link6: 1.8 kg
- link7: 0.2 kg (lightest - tool flange)

Each link has 3x3 inertia tensor computed from CAD models

---

## 6. SRDF (Semantic Robot Description) Organization

### 6.1 Group Definitions

**Group: manipulator**
- Kinematic chain from link0 (base) to link7 (tool flange)
- 7 DOF configuration space

**Group: gripper**
- Single joint: finger_joint1
- Parallel gripper mechanism

### 6.2 Named States (Predefined Configurations)

**HomePos:**
```
joint0: 0 rad
joint1: -π/2 rad (vertical)
joint2: 0 rad
joint3: 0 rad
joint4: π/2 rad
joint5: 0 rad
joint6: 0 rad
```

**ManipulationPos:**
```
joint0: 0.3665 rad
joint1: -0.1745 rad (bent)
joint2: -0.4189 rad
joint3: -1.3439 rad
joint4: 0.0524 rad
joint5: 1.5708 rad
joint6: 0.7854 rad
```

**ZeroPos:**
```
All joints: 0 rad (extended)
```

### 6.3 Collision Specifications

**Disabled Collisions (Adjacent Links):**
- link0 ↔ link1
- link1 ↔ link2
- link2 ↔ link3
- link3 ↔ link4
- link4 ↔ link5
- link5 ↔ link6
- link6 ↔ link7
- hand ↔ link6, link7

**Rationale:** Adjacent links in kinematic chain don't self-collide

---

## 7. MoveIt2 Configuration Details

### 7.1 Kinematics Solver

**Configuration: kinematics.yaml**
```yaml
robot_description_kinematics:
  manipulator:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.05 rad
    kinematics_solver_timeout: 0.05 seconds
    kinematics_solver_attempts: 3
```

**KDL (Kinematics and Dynamics Library):**
- Numerical IK solver
- Search resolution: checks 0.05 rad steps in configuration space
- Attempts: tries up to 3 different solutions
- Used for: cartesian path planning, pose-goal planning

### 7.2 Motion Planning (OMPL)

**Configuration: ompl_planning.yaml**

**Available Planners:**
1. **SBLkConfigDefault** (Sampling-Based LazyPRM)
   - Good for cluttered environments
   - Low computational cost

2. **RRTkConfigDefault** (Rapidly-exploring Random Tree)
   - Goal bias: 5% chance to sample goal directly
   - Fast convergence for simple tasks

3. **RRTConnectkConfigDefault** (RRT-Connect)
   - Bidirectional search from start and goal
   - Fast for point-to-point planning
   - **Default planner** (if not specified)

4. **RRTstarkConfigDefault** (RRT*)
   - Asymptotically optimal version of RRT
   - Delay collision checking until valid parent found

5. **PRMkConfigDefault** (Probabilistic Roadmap)
   - Precomputes configuration space connectivity
   - Good for multiple queries

6. Additional planners: EST, LBKPIECE, BKPIECE, KPIECE, TRRT, PRMstar

**Planning Parameters:**
- max_nearest_neighbors: 10 (for PRM variants)
- Default planner: Not explicitly set (system chooses RRTConnect)

### 7.3 Trajectory Execution

**Configuration: Controllers_6dof_gripper.yaml**

**Controller Mapping (MoveIt → ROS2 Control):**
```
manipulator_controller: (JointTrajectoryController)
  - Joints: joint0-joint6 (7 arm joints)
  - Interface: follow_joint_trajectory action
  - Default: true (primary controller)

gripper_controller: (JointTrajectoryController)
  - Joints: finger_joint1
  - Interface: follow_joint_trajectory action
  - Default: true
```

**Execution Settings:**
```
moveit_manage_controllers: False
  - Controllers pre-spawned, MoveIt doesn't manage lifecycle

allowed_execution_duration_scaling: 1.2
  - Allow 20% longer than planned duration

allowed_goal_duration_margin: 1.0 seconds
  - Add 1 second tolerance to goal time

allowed_start_tolerance: 0.5 radians
  - Accept starting position up to 0.5 rad away from current

trajectory_duration_monitoring: False
  - Don't monitor execution duration
```

### 7.4 Joint Acceleration Limits

**Configuration: joint_limits_6dof_gripper.yaml**

```yaml
Arm Joints:
  joint0-2: max_acceleration = 204 rad/s²
  joint3-6: max_acceleration = 36 rad/s²

Gripper:
  finger_joint1: max_acceleration = 230 rad/s²
```

**Purpose:** Time parametrization of trajectories - ensures smooth acceleration profiles

---

## 8. Controller Configurations

### 8.1 fr3_controller_gripper.yaml (Full Robot)

**ROS2 Control Structure:**
```
controller_manager:
  update_rate: 100 Hz
  
  Controllers:
    manipulator_controller: JointTrajectoryController
      - Joints: joint0-6
      - Interfaces: position, velocity (command), position, velocity (state)
      - State publish rate: 50 Hz
      - Action monitor rate: 20 Hz
      - Allow partial goals: false
      
    gripper_controller: JointTrajectoryController
      - Joints: finger_joint1 only
      - Interfaces: position, velocity (command), position, velocity (state)
      
    joint_state_broadcaster: JointStateBroadcaster
      - Publishes /joint_states topic
      
    forward_velocity_controller: JointGroupVelocityController
      - All 7 arm + gripper joints
      - Low-level velocity control interface
      
    forward_position_controller: JointGroupPositionController
      - All 7 arm + gripper joints
      - Low-level position control interface
```

**Trajectory Constraints (for trajectory tracking):**
```
For each joint:
  trajectory_tolerance: 0.2 rad
  goal_tolerance: 0.1 rad
  
Overall:
  goal_time: 0.0 (no time limit)
  stopped_velocity_tolerance: 0.1 rad/s
```

### 8.2 fr3_controller.yaml (Arm Only)

**Reduced Configuration:**
- Only 6 arm joints (joint0-5)
- No gripper controller
- Otherwise identical structure to gripper version

---

## 9. Configuration Files Summary

### 9.1 URDF Configuration Files (urdf/robots/fr3/config/)

**joint_limits.yaml** (7 joints × 4 parameters)
- lower, upper (rad)
- velocity (rad/s)
- effort (Nm)

**kinematics.yaml** (DH parameters for 7 joints)
- x, y, z (m) - link offset
- roll, pitch, yaw (rad) - joint frame rotation

**inertials.yaml** (7 links)
- origin: xyz (m), rpy (rad)
- mass (kg)
- inertia: 6 unique tensor elements (kg·m²)

**visual_parameters.yaml** (mesh file references)
- References to DAE files for visual rendering
- References to STL files for collision geometry

**initial_positions.yaml**
- Default joint angles for each configuration
- Used by ROS2 control for initial state

### 9.2 MoveIt2 Configuration Files (moveit_config/)

**kinematics.yaml**
- IK solver plugin and parameters

**controllers_6dof_gripper.yaml**
- MoveIt → ROS2 control action mappings

**joint_limits_6dof_gripper.yaml**
- Acceleration limits for trajectory planning

**ompl_planning.yaml**
- OMPL planner algorithm configurations

---

## 10. Mesh Files and Assets

### 10.1 FR3 Arm Meshes

**Visual Meshes (DAE - Digital Asset Exchange format):**
- link0.dae through link7.dae
- Located: meshes/robot_arms/fr3/visual/
- Purpose: Rendering in RViz and Gazebo
- Format: Collada with color/material information

**Collision Meshes (STL - Stereolithography format):**
- link0.stl through link7.stl
- Located: meshes/robot_arms/fr3/collision/
- Purpose: Collision detection and physics simulation
- Typically simplified geometry for computational efficiency

### 10.2 Franka Hand Gripper Meshes

**Hand Assembly (main body):**
- hand.dae (visual, white material)
- hand.stl (collision)

**Finger Assembly (parallel gripper fingers):**
- finger.dae (visual, used for both left and right)
- finger.stl (collision)

**Color Variants:**
- franka_hand_white/ (default, white colored)
- franka_hand_black/ (alternative, black colored)

### 10.3 Simulation Object Models

**Procedural Primitives:**
- box_red.xacro - Red colored box with parametric dimensions
- box_blue.xacro - Blue colored box
- cylinder.xacro - Cylindrical object
- triangle.xacro - Triangular/wedge shape

**Mesh-Based Models:**
- case.xacro - External mesh (case.dae) for container/platform

**Total Mesh Count:** 25 files (14 DAE + 11 STL)

---

## 11. RViz Configuration Files

### 11.1 moveit_franka.rviz
- **Purpose:** MoveIt2 motion planning visualization
- **Includes:**
  - Interactive markers for goal pose editing
  - Trajectory visualization (planned path)
  - Planning scene display
  - Collision object visualization
  - Interactive planning controls

### 11.2 display_franka.rviz
- **Purpose:** Basic URDF visualization
- **Includes:**
  - Robot model display
  - TF frame visualization
  - Minimal UI (no planning tools)

---

## 12. Practice Exercises and Homework Structure

### 12.1 Learning Progression

**Level 1: moveit2_practice_sample.cpp (Reference Implementation)**
- Shows MoveIt2 API usage
- 6 completed demonstrations
- Students learn from code examples

**Level 2: step1_practice.cpp (Guided Exercise)**
- Empty template with function signatures
- Task: Implement motion functions (likely copies of moveit2_practice_sample)
- Learning: API application and debugging
- Launch files: step1_practice.launch.py, step1_practice_gazebo.launch.py
- Difficulty: Beginner

**Level 3: step2_hw.cpp (Advanced Task)**
- Pick-and-place object manipulation
- Uses Object-Oriented design patterns
- Complex task with multiple stages:
  1. Approach object
  2. Grip object
  3. Lift and move
  4. Place at destination
  5. Release and retreat
- Launch files: step2_hw.launch.py, step2_hw_gazebo.launch.py
- Difficulty: Intermediate/Advanced
- Rich simulation environment (8 objects)

### 12.2 Development Workflow

1. **Understand Theory:** Read moveit2_practice_sample.cpp
2. **Practice Basics:** Complete step1_practice.cpp
3. **Apply Skills:** Implement step2_hw.cpp logic
4. **Test in Simulation:**
   - First: MoveIt2 planning-only (no Gazebo)
   - Then: Full physics simulation with Gazebo

### 12.3 Recommended Exercise Progression

**Week 1: Visualization**
```bash
# Just look at the robot structure
ros2 launch gazebo_exam display_franka.launch.py
```

**Week 2: Simulation Basics**
```bash
# See robot in Gazebo, test controllers
ros2 launch gazebo_exam gazebo_franka.launch.py launch_rviz:=true
```

**Week 3: Motion Planning**
```bash
# Plan trajectories with MoveIt2
ros2 launch gazebo_exam moveit2_franka.launch.py
```

**Week 4: Reference Examples**
```bash
# Run different demonstrations (cmd=1-6)
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=1
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=4  # waypoint example
```

**Week 5: Step 1 Assignment**
```bash
# Implement and test step1_practice
ros2 launch gazebo_exam step1_practice.launch.py
ros2 launch gazebo_exam step1_practice_gazebo.launch.py  # with physics
```

**Week 6-7: Step 2 Advanced Assignment**
```bash
# Implement pick-and-place
ros2 launch gazebo_exam step2_hw.launch.py
ros2 launch gazebo_exam step2_hw_gazebo.launch.py  # full simulation
```

---

## 13. Build and Dependency Information

### 13.1 CMakeLists.txt Structure

**Build Targets (3 executables):**
1. `moveit2_practice_sample` - Reference implementation
2. `step1_practice` - Student exercise
3. `step2_hw` - Advanced assignment

**Installation Rules:**
- Directories installed: launch, urdf, srdf, rviz, meshes, moveit_config, controller, models
- Share directory: `/usr/local/share/gazebo_exam/` (after colcon install)

### 13.2 Dependencies

**Core ROS2:**
- ament_cmake (build system)
- rclcpp (C++ client)
- std_msgs (basic message types)

**Motion Planning:**
- moveit_ros_planning_interface (C++ API)
- moveit_ros_move_group (planning server)
- moveit_ros_visualize (planning visualization)

**Geometry:**
- geometry_msgs (Pose, Transform, etc.)
- tf2 (transform library)
- tf2_ros (ROS2 transform integration)
- tf2_geometry_msgs (conversions)

**Simulation:**
- gazebo_ros (Gazebo ROS2 integration)
- gazebo_ros2_control (hardware simulation)
- ros2_control (unified control interface)
- controller_manager (spawns controllers)

**Utilities:**
- xacro (URDF macro processor)
- urdf (robot description parser)
- rviz2 (3D visualization)
- joint_state_publisher_gui (interactive joint control)

### 13.3 C++ Standards

- Minimum: C++14
- Compiler flags: -Wall -Wextra -Wpedantic (strict warnings enabled)

---

## 14. Package Manifest

**package.xml Summary:**
- Name: gazebo_exam
- Version: 0.0.0
- Build system: ament_cmake
- Maintainer: hnkim
- License: TODO (not yet specified)
- Build dependencies: ament_cmake, rclcpp, std_msgs, and all listed above

**Key Note:** Package is in development/education state (version 0.0.0, incomplete license)

---

## 15. File Statistics

| Category | Count | Files |
|----------|-------|-------|
| Total files | 71 | Mixed |
| C++ sources | 3 | .cpp |
| Python scripts | 8 | launch.py |
| Xacro (URDF/SRDF) | 16 | .xacro |
| YAML config | 14 | .yaml |
| Mesh files | 25 | .dae, .stl |
| RViz configs | 2 | .rviz |
| Main config files | 4 | CMakeLists.txt, package.xml, ... |

---

## 16. Key Integration Points

### 16.1 URDF/SRDF Pipeline

```
URDF (fr3.urdf.xacro)
  ↓ [xacro processor]
  ↓ Expanded URDF (XML with all parameters)
  
SRDF (fr3.srdf.xacro)
  ↓ [xacro processor]
  ↓ Expanded SRDF (semantic groups, states, collisions)
  
Both → MoveIt2
  ↓
  robot_description (ROS parameter)
  ↓
  MoveGroupInterface (C++ API)
  ↓
  Motion planning, collision checking, visualization
```

### 16.2 Control Flow in Simulation

```
Step 1: Launch
  └─ gazebo_franka.launch.py
     ├─ gazebo physics engine
     ├─ robot_state_publisher (reads URDF)
     └─ ros2_control_node + spawner.py (loads controllers)

Step 2: Motion Planning (if using MoveIt)
  └─ MoveIt2 move_group
     ├─ Reads robot_description (URDF)
     ├─ Reads SRDF
     ├─ Loads IK solver
     ├─ Initializes OMPL planner
     └─ Connects to JointTrajectoryController

Step 3: User Command
  └─ C++ Application (step1_practice, step2_hw)
     ├─ MoveGroupInterface.setPoseTarget()
     ├─ MoveGroupInterface.plan()
     ├─ MoveGroupInterface.execute()
     └─ Trajectory sent to ROS2 control

Step 4: Execution
  └─ ros2_control (JointTrajectoryController)
     ├─ Receives trajectory
     ├─ Sends commands to Gazebo
     └─ Gazebo simulates physics
         ├─ Updates joint states
         ├─ Detects collisions
         └─ Publishes /joint_states
         
Step 5: Feedback Loop
  └─ MoveIt2 monitors execution
     └─ Visualizes in RViz
```

### 16.3 Configuration Cascade

```
Generic Config → Specific Instantiation

moveit_config/
  ├─ kinematics.yaml → KDL solver with 3 attempts, 0.05s timeout
  ├─ joint_limits_6dof_gripper.yaml → MoveIt acceleration planning
  ├─ controllers_6dof_gripper.yaml → Action name mappings
  └─ ompl_planning.yaml → 11 algorithm options

controller/
  └─ fr3_controller_gripper.yaml → ROS2 control hardware interface
      ├─ Update rate: 100 Hz
      ├─ Joint trajectory controller constraints
      └─ Command/state interface definitions

urdf/robots/fr3/config/
  ├─ kinematics.yaml → DH parameters (joint origins)
  ├─ joint_limits.yaml → Hardware position/velocity/effort limits
  ├─ inertials.yaml → CAD-based mass/inertia properties
  ├─ visual_parameters.yaml → Mesh file paths
  └─ dynamics.yaml → Gazebo-specific damping/friction
```

---

## 17. Typical Use Cases and Workflows

### 17.1 Education Use Cases

**1. Robot Visualization (Day 1)**
```bash
ros2 launch gazebo_exam display_franka.launch.py
# Explores: Link structure, joint ranges, end-effector
```

**2. Simulation Physics (Day 2)**
```bash
ros2 launch gazebo_exam gazebo_franka.launch.py launch_rviz:=true
# Explores: Controller interfaces, real-time control
```

**3. Motion Planning (Day 3-4)**
```bash
# Example 1: Interactive planning
ros2 launch gazebo_exam moveit2_franka.launch.py
# Uses RViz to set goals, view plans

# Example 2: Reference program
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=1
# Runs move_sample() demonstration
```

**4. Homework Implementation (Days 5+)**
```bash
# Step 1: Basic motion planning
ros2 launch gazebo_exam step1_practice.launch.py
# Student implements equivalent of moveit2_practice_sample

# Step 2: Complex manipulation
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
# Student implements pick-and-place with multiple objects
```

### 17.2 Development Workflow

**Iterative Development:**
1. Edit C++ source → step1_practice.cpp
2. `colcon build` → compile to executable
3. `ros2 launch gazebo_exam step1_practice_gazebo.launch.py` → test
4. Monitor `/joint_states`, `/move_group/result` topics
5. Debug in RViz with trajectory visualization
6. Refine code → back to step 1

### 17.3 Problem-Solving Scenarios

**"Robot won't reach the target"**
- Check MoveIt collision settings (disabled_collisions in SRDF)
- Verify workspace reachability (IK solver timeout)
- Check joint limits vs target configuration

**"Gripper not closing"**
- Verify gripper_controller is spawned
- Check grasp force value in close_gripper()
- Monitor gripper position in `/joint_states`

**"Gazebo crashes during simulation"**
- Reduce object count or complexity
- Increase planning time
- Check for self-collisions in URDF

---

## 18. Summary and Key Takeaways

**Architecture Overview:**
- Modern ROS2 manipulation framework
- Educational progression from visualization → simulation → planning → implementation
- Industry-standard FR3 robot with Franka Hand
- Comprehensive xacro-based configuration system
- MoveIt2 for motion planning and control
- Gazebo for physics simulation

**Key Technologies:**
- ROS2 (humble distribution)
- MoveIt2 (motion planning)
- Gazebo 11+ (physics)
- ros2_control (unified control interface)
- Xacro (robot description language)
- OMPL (planning algorithms)
- KDL (inverse kinematics)

**Learning Objectives:**
1. Understand URDF/SRDF robot descriptions
2. Configure and use MoveIt2 for motion planning
3. Implement manipulation tasks in C++
4. Use Gazebo for physics-based validation
5. Debug complex robotic systems

**Package Completeness:**
- Well-structured with clear separation of concerns
- Extensive configuration examples
- Progressive exercise difficulty
- Rich visualization and debugging tools
- Production-ready hardware-compatible design

---

## Appendix: File Reference

### Core Files
- **CMakeLists.txt** - Build configuration for 3 executables
- **package.xml** - ROS2 package manifest
- **src/moveit2_practice_sample.cpp** - Reference (210 lines)
- **src/step1_practice.cpp** - Template (17 lines)
- **src/step2_hw.cpp** - Assignment (226 lines)

### Launch Files (8 total)
- display_franka.launch.py
- gazebo_franka.launch.py
- moveit2_franka.launch.py
- moveit2_practice.launch.py
- step1_practice.launch.py
- step1_practice_gazebo.launch.py
- step2_hw.launch.py
- step2_hw_gazebo.launch.py

### URDF Files (7 xacro)
- urdf/robots/fr3/fr3.urdf.xacro
- urdf/robots/fr3/fr3_macro.xacro
- urdf/robots/fr3/fr3_ros2_control.xacro
- urdf/robots/fr3/fr3_transmission.xacro
- urdf/end_effectors/franka_hand/franka_hand.urdf.xacro
- urdf/end_effectors/franka_hand/franka_hand_macro.xacro
- urdf/end_effectors/franka_hand/franka_hand_ros2_control.xacro

### SRDF Files (3 xacro)
- srdf/fr3.srdf.xacro
- srdf/fr3_macro.srdf.xacro
- srdf/franka_hand_macro.srdf.xacro

### Configuration YAML (14 total)
- moveit_config/ (4 files): kinematics.yaml, controllers_6dof_gripper.yaml, joint_limits_6dof_gripper.yaml, ompl_planning.yaml
- controller/ (2 files): fr3_controller.yaml, fr3_controller_gripper.yaml
- urdf/robots/fr3/config/ (6 files): joint_limits.yaml, kinematics.yaml, inertials.yaml, visual_parameters.yaml, dynamics.yaml, initial_positions.yaml

### Other Config (5 total)
- 2 RViz configs (moveit_franka.rviz, display_franka.rviz)
- 5 Xacro object models (box_red.xacro, box_blue.xacro, cylinder.xacro, triangle.xacro, case.xacro)

### Mesh Assets (25 total)
- 7 FR3 arm visual meshes (DAE)
- 7 FR3 arm collision meshes (STL)
- 2 Gripper hand meshes (DAE visual, STL collision)
- 2 Gripper finger meshes (DAE visual, STL collision)
- 4 color variants (white/black hands and fingers)

---

**End of Comprehensive Documentation**

Generated for: gazebo_exam ROS2 package
Thoroughness Level: Very Thorough
Documentation Date: 2025-11-09
