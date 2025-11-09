# Launch Files Documentation

This document provides detailed information about all launch files in the gazebo_exam package.

## Overview

The package provides 8 launch files organized into functional categories:

| Category | Files | Purpose |
|----------|-------|---------|
| Visualization | display_franka.launch.py | Static URDF visualization |
| Simulation Base | gazebo_franka.launch.py | Gazebo + controllers only |
| MoveIt Integration | moveit2_franka.launch.py | Gazebo + MoveIt2 |
| Practice | moveit2_practice.launch.py | Reference demos |
| Step 1 | step1_practice.launch.py, step1_practice_gazebo.launch.py | Basic assignment |
| Step 2 | step2_hw.launch.py, step2_hw_gazebo.launch.py | Advanced assignment |

## Launch File Dependency Graph

```
display_franka.launch.py (standalone)

gazebo_franka.launch.py (standalone)
    ↓
moveit2_franka.launch.py (includes gazebo_franka)

moveit2_practice.launch.py (standalone MoveIt2)

step1_practice.launch.py (standalone MoveIt2)
step1_practice_gazebo.launch.py (standalone Gazebo + MoveIt2)

step2_hw.launch.py (standalone MoveIt2)
step2_hw_gazebo.launch.py (standalone Gazebo + MoveIt2)
```

---

## 1. display_franka.launch.py

### Purpose
Static URDF visualization in RViz without simulation or motion planning.

### Components Launched

1. **robot_state_publisher**
   - Publishes transform tree from URDF
   - Reads `robot_description` parameter

2. **joint_state_publisher_gui**
   - Interactive slider interface for joints
   - Publishes to `/joint_states` topic

3. **rviz2**
   - Visualization with `display_franka.rviz` config
   - Shows robot model and TF frames

### Launch Command

```bash
ros2 launch gazebo_exam display_franka.launch.py
```

### Use Cases

- Exploring robot structure
- Validating URDF modifications
- Understanding kinematic chain
- Teaching robot anatomy

### Parameters

- `name` (default: "fr3") - Robot name
- `prefix` (default: "") - Joint/link prefix
- Uses `sim_gazebo:=false` - No simulation plugins

### Topics Published

- `/joint_states` - Joint positions from GUI sliders
- `/tf`, `/tf_static` - Transform tree

### Notes

- No physics simulation
- No motion planning
- Manual joint control only
- Fast startup (~2 seconds)

---

## 2. gazebo_franka.launch.py

### Purpose
Full Gazebo physics simulation with ROS2 control, without MoveIt2.

### Components Launched

1. **Gazebo Server**
   - Physics engine (ODE or Bullet)
   - Collision detection
   - Dynamics simulation

2. **Gazebo Client** (GUI)
   - 3D visualization
   - Camera controls
   - Scene manipulation

3. **robot_state_publisher**
   - URDF → TF tree conversion

4. **spawn_entity.py**
   - Spawns FR3 robot into Gazebo world
   - Uses `robot_description` parameter

5. **ros2_control_node**
   - Control interface manager
   - Loads `fr3_controller_gripper.yaml`
   - Connects to GazeboSystem hardware interface

6. **Controller Spawners** (sequential):
   - `joint_state_broadcaster` → after Gazebo spawn
   - `manipulator_controller` → after broadcaster
   - `gripper_controller` → after broadcaster

7. **RViz2** (optional)
   - Parameter: `launch_rviz:=true`

### Launch Command

```bash
# Without RViz
ros2 launch gazebo_exam gazebo_franka.launch.py

# With RViz
ros2 launch gazebo_exam gazebo_franka.launch.py launch_rviz:=true
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| name | "fr3" | Robot name |
| prefix | "" | Joint/link prefix |
| launch_rviz | false | Start RViz visualization |
| use_sim_time | true | Use Gazebo time |

### URDF Configuration

Uses `sim_gazebo:=true`:
- Enables `gazebo_ros2_control/GazeboSystem` plugin
- Activates Gazebo-specific tags (e.g., `<gazebo>`)

### Controller Configuration

From `controller/fr3_controller_gripper.yaml`:

**Active Controllers:**
- `manipulator_controller` (JointTrajectoryController)
  - 7 arm joints
  - Action: `/manipulator_controller/follow_joint_trajectory`

- `gripper_controller` (JointTrajectoryController)
  - 1 gripper joint
  - Action: `/gripper_controller/follow_joint_trajectory`

- `joint_state_broadcaster` (JointStateBroadcaster)
  - Publishes `/joint_states` at 50 Hz

### Startup Sequence

```
1. Launch Gazebo (physics + GUI)
2. Publish robot_description
3. Spawn robot entity → EVENT: gazebo_spawn_robot exits
4. Start joint_state_broadcaster → EVENT: broadcaster exits
5. Start manipulator_controller
6. Start gripper_controller
7. (Optional) Launch RViz
```

### Use Cases

- Testing low-level controllers
- Physics simulation without planning
- Direct trajectory execution
- Learning Gazebo/ROS2 control

### Topics

**Published:**
- `/joint_states` - Current joint positions/velocities
- `/tf`, `/tf_static` - Transform tree
- `/clock` - Simulation time

**Subscribed:**
- `/manipulator_controller/joint_trajectory` - Arm commands
- `/gripper_controller/joint_trajectory` - Gripper commands

### Services

- `/controller_manager/list_controllers` - List active controllers
- `/controller_manager/load_controller` - Load new controller

### Startup Time

~10-15 seconds (Gazebo initialization is slow)

---

## 3. moveit2_franka.launch.py

### Purpose
Complete system: Gazebo simulation + MoveIt2 motion planning + RViz interface.

### Components Launched

**Includes:** `gazebo_franka.launch.py`
- All Gazebo and controller components

**Additional:**

1. **move_group Node**
   - MoveIt2 planning server
   - OMPL motion planners
   - Collision detection
   - Trajectory execution manager

2. **RViz2 with MoveIt Plugin**
   - Config: `moveit_franka.rviz`
   - Interactive markers for goal setting
   - Trajectory visualization
   - Planning scene display

### Launch Command

```bash
ros2 launch gazebo_exam moveit2_franka.launch.py
```

### MoveIt Configuration Loaded

- **Robot Description:** URDF from xacro
- **Semantic Description:** SRDF from xacro
- **Kinematics:** KDL solver (kinematics.yaml)
- **Planning:** OMPL planners (ompl_planning.yaml)
- **Controllers:** MoveIt controller mappings (controllers_6dof_gripper.yaml)
- **Joint Limits:** Acceleration limits (joint_limits_6dof_gripper.yaml)

### Planning Pipeline

```
move_group Parameters:
  - planning_plugin: ompl_interface/OMPLPlanner
  - default_planner: RRTConnect (if not specified)
  - planning_time: 45 seconds (configurable in code)
  - planning_attempts: 10 (configurable in code)
```

### Execution Pipeline

```
MoveIt2 → FollowJointTrajectory action → JointTrajectoryController → Gazebo
```

### Use Cases

- Interactive motion planning
- Testing complex trajectories
- Visual planning validation
- Teaching MoveIt2 concepts

### RViz Interface

**Panels:**
- MotionPlanning - Plan/execute controls
- Planning Scene - Add/remove obstacles
- Trajectory - View planned paths
- Displays - Robot model, planning scene, etc.

**Interactive Markers:**
- Blue sphere at end-effector
- Drag to set goal pose
- Click "Plan" to compute trajectory
- Click "Execute" to run on robot

### Startup Time

~20-25 seconds (Gazebo + MoveIt2 initialization)

### Notes

- No automatic execution (user controls via RViz)
- Planning failures shown in terminal
- Can add collision objects via RViz

---

## 4. moveit2_practice.launch.py

### Purpose
Run reference implementation (moveit2_practice_sample) with MoveIt2, no Gazebo.

### Components Launched

1. **move_group Node** (MoveIt2)
2. **robot_state_publisher**
3. **RViz2** with MoveIt plugin
4. **moveit2_practice_sample executable**
   - Spawned after move_group starts
   - Reads `cmd` parameter

### Launch Command

```bash
# Default (no command)
ros2 launch gazebo_exam moveit2_practice.launch.py

# Run specific demonstration
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=1  # move_sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=2  # rotation_sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=3  # gripper_sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=4  # waypoint_sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=5  # rectangle_sample (homework)
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=6  # circle_sample (homework)
```

### Hardware Interface

Uses `GenericSystem` (mock hardware):
- No physics simulation
- Instant trajectory execution
- Visualized in RViz only

### Demonstration Scenarios

| cmd | Function | Description |
|-----|----------|-------------|
| 1 | move_sample | Move along X, Y, Z axes |
| 2 | rotation_sample | Rotate around pitch axis |
| 3 | gripper_sample | Open/close gripper |
| 4 | waypoint_sample | Cartesian path through waypoints |
| 5 | rectangle_sample | (Student homework) |
| 6 | circle_sample | (Student homework) |

### Use Cases

- Learning MoveIt2 API
- Studying reference code
- Fast testing (no Gazebo overhead)
- Understanding motion primitives

### Startup Time

~8-10 seconds (no Gazebo)

---

## 5. step1_practice.launch.py

### Purpose
Run student practice template (step1_practice) with MoveIt2, no Gazebo.

### Structure

Identical to `moveit2_practice.launch.py` but runs `step1_practice` executable instead.

### Launch Command

```bash
ros2 launch gazebo_exam step1_practice.launch.py
```

### Expected Implementation

Students should implement motion planning logic similar to moveit2_practice_sample.

### Use Cases

- Testing student code quickly
- Debugging without physics
- Iterative development

---

## 6. step1_practice_gazebo.launch.py

### Purpose
Step 1 assignment with full Gazebo simulation and 2 manipulation objects.

### Components Launched

**All components from gazebo_franka.launch.py, plus:**

1. **MoveIt2 move_group**
2. **RViz2** with MoveIt plugin
3. **Simulation Objects:**
   - box1 (red box) at (0.4, 0.1, 0.03)
   - box2 (blue box) at (0.4, -0.1, 0.03)
   - Both: 5cm × 5cm × 5cm cubes
4. **step1_practice executable**

### Object Spawn Details

```python
# Box 1 (red)
spawn_box1 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'box1',
        '-topic', 'box_red_description',
        '-x', '0.4', '-y', '0.1', '-z', '0.03'
    ]
)

# Box 2 (blue)
spawn_box2 = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
        '-entity', 'box2',
        '-topic', 'box_blue_description',
        '-x', '0.4', '-y', '-0.1', '-z', '0.03'
    ]
)
```

### Launch Command

```bash
ros2 launch gazebo_exam step1_practice_gazebo.launch.py
```

### Use Cases

- Testing pick-and-place logic
- Visual feedback with physics
- Final validation before submission

### Startup Time

~20-25 seconds

---

## 7. step2_hw.launch.py

### Purpose
Run advanced homework (step2_hw) with MoveIt2, no Gazebo.

### Structure

Similar to step1_practice.launch.py but runs `step2_hw` executable.

### Launch Command

```bash
ros2 launch gazebo_exam step2_hw.launch.py
```

### Expected Implementation

Pick-and-place task using PickAndLiftNode class.

---

## 8. step2_hw_gazebo.launch.py

### Purpose
Advanced assignment with rich simulation environment (8 objects).

### Components Launched

**All from gazebo_franka.launch.py, plus:**

1. **MoveIt2 move_group**
2. **RViz2**
3. **8 Simulation Objects:**

| Object | Type | Position (x, y, z) | Dimensions/Radius | Color |
|--------|------|-------------------|-------------------|-------|
| box1 | Box | (0.4, 0.1, 0.03) | 5cm cube | Red |
| box2 | Box | (0.4, 0.0, 0.03) | 5cm cube | Red |
| box3 | Box | (0.4, -0.1, 0.03) | 2.5cm cube | Red |
| box4 | Box | (0.5, -0.1, 0.035) | 7cm tall | Red |
| box5 | Box | (0.5, 0.1, 0.04) | 8cm tall | Red |
| box6 | Box | (0.6, 0.0, 0.045) | 9cm tall | Red |
| cylinder | Cylinder | (0.5, 0.0, 0.03) | r=2.5cm, h=6cm | - |
| triangle | Triangle/Wedge | (0.6, 0.1, 0.03) | Custom shape | - |

4. **case** - Container mesh at (0.0, 0.2, 0.0)
5. **step2_hw executable**

### Launch Command

```bash
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
```

### Complexity

- Most complex simulation environment
- Tests manipulation of various shapes
- Heavy computational load
- Realistic pick-and-place scenario

### Use Cases

- Final homework validation
- Complex manipulation testing
- Multi-object scenarios

### Startup Time

~25-30 seconds (many objects to spawn)

---

## Common Launch Patterns

### Launch File Structure

All launch files follow this pattern:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

def generate_launch_description():
    # 1. Declare arguments
    arg1 = DeclareLaunchArgument('param1', default_value='value')

    # 2. Load configurations (YAML, xacro)
    robot_description = Command(['xacro ', urdf_file])

    # 3. Define nodes
    node1 = Node(package='pkg', executable='exe')

    # 4. Add event handlers (OnProcessStart, OnProcessExit)
    handler = RegisterEventHandler(...)

    # 5. Return LaunchDescription
    return LaunchDescription([arg1, node1, handler])
```

### Event-Driven Sequencing

```python
# Wait for Gazebo spawn before starting controllers
spawn_controller = RegisterEventHandler(
    OnProcessExit(
        target_action=spawn_robot,
        on_exit=[joint_state_broadcaster_spawner]
    )
)
```

### Parameter Loading

```python
# Load YAML
yaml_config = load_yaml('package', 'path/to/file.yaml')

# Load xacro
robot_description = Command([
    'xacro ',
    PathJoinSubstitution([package, 'urdf', 'robot.urdf.xacro']),
    ' sim_gazebo:=true'
])
```

## Debugging Launch Files

### Check Loaded Parameters

```bash
ros2 param list /move_group
ros2 param get /move_group robot_description
```

### Monitor Node Status

```bash
ros2 node list
ros2 node info /move_group
```

### Check Controller Status

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

### View Topics

```bash
ros2 topic list
ros2 topic echo /joint_states
```

### Gazebo Status

```bash
ros2 service list | grep gazebo
ros2 service call /gazebo/get_entity_state ...
```

## Performance Comparison

| Launch File | Startup Time | CPU Usage | RAM Usage | Use Case |
|-------------|--------------|-----------|-----------|----------|
| display_franka | ~2s | Low | ~200 MB | Quick URDF check |
| gazebo_franka | ~15s | Medium | ~1.5 GB | Controller testing |
| moveit2_franka | ~25s | High | ~2 GB | Interactive planning |
| moveit2_practice | ~10s | Low | ~500 MB | Fast development |
| step1_practice_gazebo | ~25s | High | ~2 GB | Assignment testing |
| step2_hw_gazebo | ~30s | Very High | ~2.5 GB | Final validation |

## Troubleshooting

### Issue: "No node named move_group"

**Cause:** MoveIt2 not started or crashed
**Solution:**
```bash
ros2 node list  # Check if /move_group exists
ros2 topic echo /rosout  # Check for error messages
```

### Issue: Controllers not spawning

**Cause:** Gazebo not ready or event handler failed
**Solution:**
```bash
ros2 control list_controllers
# If empty, manually spawn:
ros2 run controller_manager spawner manipulator_controller
```

### Issue: RViz shows "No transform from..."

**Cause:** robot_state_publisher not running or URDF error
**Solution:**
```bash
ros2 run tf2_ros tf2_echo world link0
check_urdf urdf/robots/fr3/fr3.urdf.xacro
```

### Issue: Executable not found

**Cause:** Package not built or not sourced
**Solution:**
```bash
colcon build --packages-select gazebo_exam
source install/setup.bash
ros2 pkg executables gazebo_exam  # List available executables
```
