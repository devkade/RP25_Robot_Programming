# MoveIt2 Configuration

This document details the MoveIt2 configuration for the gazebo_exam package.

## Overview

MoveIt2 is configured to provide motion planning for the FR3 robot with the following capabilities:
- Inverse kinematics (IK) solving
- Motion planning (multiple OMPL algorithms)
- Collision avoidance
- Trajectory execution
- Planning scene management

## Configuration Files

Located in `moveit_config/`:

| File | Purpose |
|------|---------|
| kinematics.yaml | IK solver configuration |
| ompl_planning.yaml | OMPL planner algorithms |
| controllers_6dof_gripper.yaml | Controller interface mappings |
| joint_limits_6dof_gripper.yaml | Planning acceleration limits |

## Kinematics Configuration

### File: kinematics.yaml

```yaml
robot_description_kinematics:
  manipulator:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    kinematics_solver_search_resolution: 0.05
    kinematics_solver_timeout: 0.05
    kinematics_solver_attempts: 3
```

### KDL Kinematics Plugin

**Algorithm:** Numerical inverse kinematics using Newton-Raphson method

**Parameters:**

- **kinematics_solver:** Plugin type (KDL)
- **search_resolution:** 0.05 rad (2.86°)
  - Configuration space discretization
  - Smaller = more accurate, slower
  - Larger = faster, less accurate

- **timeout:** 0.05 seconds per IK attempt
  - Fast timeout for real-time performance
  - May fail for difficult poses

- **attempts:** 3 tries with different random seeds
  - Increases success rate
  - Total IK time: up to 0.15 seconds (3 × 0.05s)

### IK Solver Behavior

**Successful IK:**
```
Input: target_pose (x, y, z, qx, qy, qz, qw)
Output: joint_values [j0, j1, j2, j3, j4, j5, j6]
Success if: joint_values satisfy target_pose within tolerance
```

**Failed IK:**
- Returns false
- Common causes:
  - Pose outside workspace
  - Near singularity
  - Joint limit violations
  - Timeout

### Workspace Limits

Implicitly defined by:
- Joint limits (from URDF)
- Link lengths (from kinematics.yaml)
- Singularity avoidance

## Motion Planning Configuration

### File: ompl_planning.yaml

OMPL (Open Motion Planning Library) provides multiple planning algorithms.

### Available Planners

#### 1. RRTConnect (Default)
```yaml
RRTConnectkConfigDefault:
  type: geometric::RRTConnect
  range: 0.0  # Auto-compute
```

**Characteristics:**
- Bidirectional tree search (start ← → goal)
- Fast convergence for point-to-point
- Not asymptotically optimal
- **Use for:** Simple pick-and-place, fast planning

#### 2. RRT (Basic)
```yaml
RRTkConfigDefault:
  type: geometric::RRT
  range: 0.0
  goal_bias: 0.05  # 5% chance to sample goal directly
```

**Characteristics:**
- Single tree from start
- Goal biasing speeds convergence
- **Use for:** Simple obstacles, exploration

#### 3. RRT* (Optimal)
```yaml
RRTstarkConfigDefault:
  type: geometric::RRTstar
  range: 0.0
  goal_bias: 0.05
  delay_collision_checking: true
```

**Characteristics:**
- Asymptotically optimal
- Rewires tree for better paths
- Slower than RRT
- **Use for:** When path quality matters

#### 4. PRM (Probabilistic Roadmap)
```yaml
PRMkConfigDefault:
  type: geometric::PRM
  max_nearest_neighbors: 10
```

**Characteristics:**
- Builds roadmap of valid configurations
- Good for multiple queries
- Pre-computation phase
- **Use for:** Repeated planning in same environment

#### 5. PRM* (Optimal Roadmap)
```yaml
PRMstarkConfigDefault:
  type: geometric::PRMstar
```

**Characteristics:**
- Asymptotically optimal version of PRM
- Denser roadmap connections
- **Use for:** High-quality multi-query planning

#### 6. KPIECE
```yaml
KPIECEkConfigDefault:
  type: geometric::KPIECE
  range: 0.0
  goal_bias: 0.05
  border_fraction: 0.9
  failed_expansion_cell_score_factor: 0.5
  min_valid_path_fraction: 0.5
```

**Characteristics:**
- Discretizes workspace into cells
- Explores underexplored regions
- **Use for:** Complex obstacle environments

#### 7. BKPIECE (Bidirectional KPIECE)
```yaml
BKPIECEkConfigDefault:
  type: geometric::BKPIECE
  range: 0.0
  border_fraction: 0.9
  failed_expansion_cell_score_factor: 0.5
  min_valid_path_fraction: 0.5
```

#### 8. LBKPIECE
```yaml
LBKPIECEkConfigDefault:
  type: geometric::LBKPIECE
  range: 0.0
  border_fraction: 0.9
  min_valid_path_fraction: 0.5
```

#### 9. EST (Expansive Space Trees)
```yaml
ESTkConfigDefault:
  type: geometric::EST
  range: 0.0
  goal_bias: 0.05
```

#### 10. SBL (Single-query Bi-directional Lazy planner)
```yaml
SBLkConfigDefault:
  type: geometric::SBL
  range: 0.0
```

#### 11. TRRT (Transition-based RRT)
```yaml
TRRTkConfigDefault:
  type: geometric::TRRT
  range: 0.0
  goal_bias: 0.05
  max_states_failed: 10
  temp_change_factor: 2.0
  init_temperature: 10e-6
  frountier_threshold: 0.0
  frountierNodeRatio: 0.1
  k_constant: 0.0
```

**Characteristics:**
- Cost-aware planning
- Useful for path quality optimization

### Planner Selection

**In Code:**
```cpp
move_group.setPlannerId("RRTConnectkConfigDefault");
// Or any planner name from ompl_planning.yaml
```

**Default Planner:**
If not specified, MoveIt chooses RRTConnect.

### Planning Pipeline Configuration

```yaml
ompl_planning_pipeline_config:
  move_group:
    planning_plugin: ompl_interface/OMPLPlanner
    request_adapters: >
      default_planner_request_adapters/AddTimeOptimalParameterization
      default_planner_request_adapters/FixWorkspaceBounds
      default_planner_request_adapters/FixStartStateBounds
      default_planner_request_adapters/FixStartStateCollision
      default_planner_request_adapters/FixStartStatePathConstraints
    start_state_max_bounds_error: 0.5
```

**Request Adapters:**

1. **AddTimeOptimalParameterization**
   - Adds velocity/acceleration profiles to path
   - Respects joint acceleration limits

2. **FixWorkspaceBounds**
   - Ensures goal within workspace

3. **FixStartStateBounds**
   - Adjusts start state if slightly outside bounds

4. **FixStartStateCollision**
   - Moves start state out of collision

5. **FixStartStatePathConstraints**
   - Satisfies path constraints from start

## Controller Configuration

### File: controllers_6dof_gripper.yaml

Maps MoveIt planning groups to ROS2 controllers.

```yaml
controller_names:
  - manipulator_controller
  - gripper_controller

manipulator_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - joint0
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

gripper_controller:
  action_ns: follow_joint_trajectory
  type: FollowJointTrajectory
  default: true
  joints:
    - finger_joint1
```

**Action Interface:**
- `/manipulator_controller/follow_joint_trajectory`
- `/gripper_controller/follow_joint_trajectory`

**Execution Settings:**
```yaml
moveit_manage_controllers: false  # Controllers pre-spawned
trajectory_execution:
  allowed_execution_duration_scaling: 1.2  # 20% tolerance
  allowed_goal_duration_margin: 1.0  # +1 second
  allowed_start_tolerance: 0.5  # 0.5 rad start deviation
```

## Joint Acceleration Limits

### File: joint_limits_6dof_gripper.yaml

Defines acceleration limits for trajectory time parametrization.

```yaml
joint_limits:
  joint0:
    has_acceleration_limits: true
    max_acceleration: 204  # rad/s²
  joint1:
    has_acceleration_limits: true
    max_acceleration: 204
  joint2:
    has_acceleration_limits: true
    max_acceleration: 204
  joint3:
    has_acceleration_limits: true
    max_acceleration: 36  # Lower for distal joints
  joint4:
    has_acceleration_limits: true
    max_acceleration: 36
  joint5:
    has_acceleration_limits: true
    max_acceleration: 36
  joint6:
    has_acceleration_limits: true
    max_acceleration: 36
  finger_joint1:
    has_acceleration_limits: true
    max_acceleration: 230  # Gripper can accelerate faster
```

**Purpose:**
- Smooth trajectory generation
- Respects motor capabilities
- Ensures safe motion

**Note:** These differ from URDF velocity/effort limits (separate concerns).

## Planning Scene

### Collision Objects

Add obstacles dynamically:

```cpp
#include <moveit/planning_scene_interface/planning_scene_interface.h>

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

// Add box obstacle
moveit_msgs::msg::CollisionObject collision_object;
collision_object.header.frame_id = "world";
collision_object.id = "box1";

shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions = {0.05, 0.05, 0.05};  // 5cm cube

geometry_msgs::msg::Pose box_pose;
box_pose.position.x = 0.4;
box_pose.position.y = 0.1;
box_pose.position.z = 0.025;
box_pose.orientation.w = 1.0;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

planning_scene_interface.applyCollisionObject(collision_object);
```

### Ground Plane

Common practice (see step2_hw.cpp):

```cpp
moveit_msgs::msg::CollisionObject ground_plane;
ground_plane.header.frame_id = "world";
ground_plane.id = "ground";

shape_msgs::msg::SolidPrimitive plane;
plane.type = plane.BOX;
plane.dimensions = {2.0, 2.0, 0.01};  // Large thin box

geometry_msgs::msg::Pose plane_pose;
plane_pose.position.z = -0.005;  // Below robot base
plane_pose.orientation.w = 1.0;

ground_plane.primitives.push_back(plane);
ground_plane.primitive_poses.push_back(plane_pose);
ground_plane.operation = ground_plane.ADD;

planning_scene_interface.applyCollisionObject(ground_plane);
```

## MoveIt C++ API Usage

### Basic Motion Planning

```cpp
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Initialize
auto move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");

// Configure planning
move_group.setPlanningTime(10.0);  // 10 seconds timeout
move_group.setNumPlanningAttempts(10);
move_group.setPlannerId("RRTConnectkConfigDefault");

// Set goal
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.4;
target_pose.position.y = 0.2;
target_pose.position.z = 0.5;
target_pose.orientation.w = 1.0;

move_group.setPoseTarget(target_pose);

// Plan
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

// Execute
if (success) {
  move_group.execute(plan);
}
```

### Cartesian Path Planning

```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;

// Current pose
geometry_msgs::msg::Pose waypoint = move_group.getCurrentPose().pose;

// Add waypoints
waypoint.position.z -= 0.1;
waypoints.push_back(waypoint);

waypoint.position.y += 0.1;
waypoints.push_back(waypoint);

// Compute path
moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group.computeCartesianPath(
  waypoints,
  0.01,  // eef_step (1cm)
  0.0,   // jump_threshold (disabled)
  trajectory
);

// Execute if successful
if (fraction > 0.95) {  // 95% of path achieved
  move_group.execute(trajectory);
}
```

### Joint Space Planning

```cpp
std::vector<double> joint_group_positions = {0, -0.7854, 0, -2.3562, 0, 1.5708, 0.7854};

move_group.setJointValueTarget(joint_group_positions);
move_group.move();  // Plan and execute
```

### Workspace Constraints

```cpp
moveit_msgs::msg::WorkspaceParameters workspace;
workspace.header.frame_id = "world";
workspace.min_corner.x = 0.0;
workspace.min_corner.y = -1.0;
workspace.min_corner.z = 0.0;
workspace.max_corner.x = 1.0;
workspace.max_corner.y = 1.0;
workspace.max_corner.z = 2.0;

move_group.setWorkspace(
  workspace.min_corner.x, workspace.min_corner.y, workspace.min_corner.z,
  workspace.max_corner.x, workspace.max_corner.y, workspace.max_corner.z
);
```

## Performance Tuning

### Planning Time

**Faster Planning:**
```cpp
move_group.setPlanningTime(5.0);  // Reduce from default 45s
move_group.setNumPlanningAttempts(3);  // Reduce attempts
```

**Better Solutions:**
```cpp
move_group.setPlanningTime(30.0);  // More time
move_group.setNumPlanningAttempts(20);  // More attempts
move_group.setPlannerId("RRTstarkConfigDefault");  // Optimal planner
```

### Collision Checking

**Speed vs Safety Trade-off:**

Edit `ompl_planning.yaml`:
```yaml
projection_evaluator: joints(joint0,joint1)  # Fewer joints = faster
longest_valid_segment_fraction: 0.05  # Larger = faster, less safe
```

### IK Solver

**Faster IK:**
```yaml
kinematics_solver_timeout: 0.01  # Reduce from 0.05
kinematics_solver_attempts: 1  # Single attempt
```

**More Robust IK:**
```yaml
kinematics_solver_timeout: 0.1  # Increase timeout
kinematics_solver_attempts: 5  # More attempts
```

## Common Configuration Issues

### Issue: Planning Always Fails

**Check:**
1. Are planning group names correct? ("manipulator", not "arm")
2. Is move_group node running?
3. Are controllers spawned?
4. Is target reachable?

**Debug:**
```bash
ros2 topic echo /move_group/result
ros2 param get /move_group robot_description_semantic
```

### Issue: IK Solver Fails

**Solutions:**
1. Increase timeout/attempts in kinematics.yaml
2. Try different target orientations
3. Check joint limits
4. Verify workspace bounds

### Issue: Slow Planning

**Solutions:**
1. Use RRTConnect instead of RRT*
2. Reduce planning time
3. Simplify collision meshes
4. Disable unnecessary collision pairs in SRDF

### Issue: Jerky Trajectories

**Solutions:**
1. Check acceleration limits in joint_limits_6dof_gripper.yaml
2. Verify TimeOptimalParameterization adapter is active
3. Increase eef_step in computeCartesianPath

## Advanced Features

### Multi-Pipeline Planning

Configure multiple planning pipelines:
```yaml
planning_pipelines:
  - ompl
  - chomp
  - pilz_industrial_motion_planner
```

### Path Constraints

Constrain end-effector orientation:
```cpp
moveit_msgs::msg::OrientationConstraint ocm;
ocm.link_name = "link7";
ocm.header.frame_id = "world";
ocm.orientation = target_orientation;
ocm.absolute_x_axis_tolerance = 0.1;
ocm.absolute_y_axis_tolerance = 0.1;
ocm.absolute_z_axis_tolerance = 0.1;
ocm.weight = 1.0;

moveit_msgs::msg::Constraints constraints;
constraints.orientation_constraints.push_back(ocm);
move_group.setPathConstraints(constraints);
```

### Custom Planning Adapters

Write custom request adapters for specialized behavior (advanced).

## Configuration Best Practices

1. **Start with defaults** - RRTConnect, 10s planning time
2. **Profile your application** - Measure planning times
3. **Tune incrementally** - Change one parameter at a time
4. **Test edge cases** - Workspace boundaries, singularities
5. **Balance speed and quality** - Use RRT* only when needed
6. **Use planning scene** - Add obstacles for collision avoidance
7. **Validate in simulation** - Test with Gazebo before hardware
