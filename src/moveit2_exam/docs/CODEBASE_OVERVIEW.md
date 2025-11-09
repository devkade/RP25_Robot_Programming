# MoveIt2 Practice Sample - Codebase Overview

## Table of Contents
1. [Project Overview](#project-overview)
2. [Directory Structure](#directory-structure)
3. [Dependencies](#dependencies)
4. [Architecture](#architecture)
5. [Existing Examples](#existing-examples)
6. [Key MoveIt2 APIs](#key-moveit2-apis)
7. [How to Run](#how-to-run)
8. [Implementation Guide](#implementation-guide)

---

## Project Overview

This is a ROS2 package that demonstrates MoveIt2 usage with a Franka FR3 robotic arm. The package includes 4 working examples and 2 to-be-implemented examples for drawing shapes.

**Robot**: Franka FR3 (arm_id: fr3)
**End Effector**: Franka Hand (ee_id: franka_hand)
**ROS2 Distribution**: Assumed to be ROS2 Humble or later
**Build System**: ament_cmake

---

## Directory Structure

```
moveit2_exam/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── src/
│   └── moveit2_practice_sample.cpp  # Main source file with all examples
├── launch/
│   ├── display_franka.launch.py     # RViz display launch file
│   ├── gazebo_franka.launch.py      # Gazebo simulation launch file
│   ├── moveit2_franka.launch.py     # MoveIt2 configuration launch file
│   └── moveit2_practice.launch.py   # Main practice launch file
├── urdf/                       # Robot URDF files
├── srdf/                       # Semantic Robot Description Format files
├── meshes/                     # Robot mesh files
├── rviz/                       # RViz configuration files
├── moveit_config/              # MoveIt configuration files
│   ├── kinematics.yaml
│   ├── joint_limits_6dof_gripper.yaml
│   ├── controllers_6dof_gripper.yaml
│   └── ompl_planning.yaml
└── controller/                 # Controller configuration
```

---

## Dependencies

### Core Dependencies (from package.xml and CMakeLists.txt):
- `rclcpp` - ROS2 C++ client library
- `std_msgs` - Standard ROS2 messages
- `tf2` - Transform library
- `tf2_ros` - ROS2 bindings for TF2
- `tf2_geometry_msgs` - TF2 geometry message conversions
- `geometry_msgs` - Geometry-related messages
- `moveit_ros_planning_interface` - MoveIt2 planning interface
- `urdf` - URDF parser
- `rviz2` - 3D visualization tool
- `xacro` - XML macro language for robot description

### Build Configuration:
- CMake minimum version: 3.5
- C Standard: C99
- C++ Standard: C++14

---

## Architecture

### Main Components

#### 1. Node Structure
The main executable `moveit2_practice_sample` is a single-purpose node that:
- Initializes ROS2 with `use_sim_time` set to true (for simulation)
- Creates two MoveGroupInterface objects:
  - `arm` - Controls the "manipulator" planning group
  - `gripper` - Controls the "gripper" planning group
- Sets planning time to 45 seconds
- Accepts a `cmd` parameter to select which example to run

#### 2. MoveGroupInterface
The primary interface to MoveIt2, providing methods for:
- Motion planning
- Execution
- Joint and pose target setting
- Cartesian path computation

---

## Existing Examples

### Example 1: Move Sample (cmd=1)
**File**: `src/moveit2_practice_sample.cpp:40-70`

**Purpose**: Demonstrates basic pose-based movement along X, Y, and Z axes.

**Implementation Details**:
```cpp
void move_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                 rclcpp::Node::SharedPtr node)
```

**What it does**:
1. Sets initial pose: x=0.4, y=0.2, z=0.5, orientation=(-π/2, 0, 0)
2. Moves along Y-axis: +0.05, -0.1, +0.05 (net: 0)
3. Moves along X-axis: +0.02, -0.05, +0.02 (net: -0.01)
4. Moves along Z-axis: +0.1, -0.1 (net: 0)

**Key API Usage**:
- `list_to_pose()` - Converts x, y, z, roll, pitch, yaw to Pose message
- `go_to_pose_goal()` - Plans and executes movement to target pose
- `setPoseTarget()` - Sets the target pose
- `plan()` - Computes motion plan
- `execute()` - Executes the planned trajectory

---

### Example 2: Rotation Sample (cmd=2)
**File**: `src/moveit2_practice_sample.cpp:72-86`

**Purpose**: Demonstrates orientation changes while maintaining the same position.

**Implementation Details**:
```cpp
void rotation_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface)
```

**What it does**:
1. Moves to position: x=0.34, y=0.2, z=0.3, orientation=(-π/2, 0, 0)
2. Changes pitch to 0.3 radians
3. Changes pitch to -0.3 radians
4. Returns to original orientation (pitch=0)

**Key Concept**:
- Only the pitch angle (second value in RPY) changes
- Demonstrates pure rotational movement without translation
- Uses the same `list_to_pose()` helper function with different orientation values

---

### Example 3: Gripper Sample (cmd=3)
**File**: `src/moveit2_practice_sample.cpp:88-106`

**Purpose**: Demonstrates joint-space control for opening and closing the gripper.

**Implementation Details**:
```cpp
void gripper_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                    double value, rclcpp::Node::SharedPtr node)
```

**What it does**:
1. Gets current joint values of the gripper
2. Sets joint[0] to the specified value (0.07 for open, 0.0 for close)
3. Sets joint[1] to 0
4. Plans and executes the motion

**Main Function Calls**:
```cpp
gripper_sample(gripper, 0.07, node);  // Open gripper
gripper_sample(gripper, 0.0, node);   // Close gripper
```

**Key API Usage**:
- `getCurrentJointValues()` - Retrieves current joint positions
- `setJointValueTarget()` - Sets target in joint space (vs. Cartesian space)
- This is different from pose-based planning; uses joint angles directly

---

### Example 4: Waypoint Sample (cmd=4)
**File**: `src/moveit2_practice_sample.cpp:108-138`

**Purpose**: Demonstrates Cartesian path planning through multiple waypoints.

**Implementation Details**:
```cpp
void waypoint_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     rclcpp::Node::SharedPtr node)
```

**What it does**:
1. Moves to starting position: x=0.4, y=0.2, z=0.6
2. Waits 2 seconds
3. Creates waypoints:
   - Waypoint 1: z -= 0.1 (move down)
   - Waypoint 2: x += 0.1 (move forward)
4. Computes Cartesian path with 0.005m step size
5. Executes the planned trajectory

**Key API Usage**:
- `computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory)`
  - `waypoints`: Vector of poses to pass through
  - `eef_step`: Maximum step size (0.005m = 5mm)
  - `jump_threshold`: 0.0 = no jump threshold
  - Returns `fraction`: Percentage of path successfully computed (0.0 to 1.0)

**Important Concept**:
- Cartesian paths guarantee the end-effector follows a straight line between waypoints
- Regular `setPoseTarget()` does NOT guarantee straight-line paths
- Used for tasks requiring precise path control (drawing, painting, etc.)

---

## Key MoveIt2 APIs

### 1. Helper Functions

#### `list_to_pose(x, y, z, roll, pitch, yaw)`
**Location**: `src/moveit2_practice_sample.cpp:27-38`

Converts position and RPY orientation to a `geometry_msgs::msg::Pose`:
```cpp
geometry_msgs::msg::Pose list_to_pose(double x, double y, double z,
                                       double roll, double pitch, double yaw)
```

**Key Components**:
- Uses `tf2::Quaternion` for orientation conversion
- `setRPY()` converts Euler angles to quaternion
- Returns a Pose message ready for MoveIt2

---

#### `go_to_pose_goal(move_group_interface, target_pose)`
**Location**: `src/moveit2_practice_sample.cpp:13-25`

Plans and executes movement to a target pose:
```cpp
void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                     geometry_msgs::msg::Pose &target_pose)
```

**Process**:
1. Sets the target pose
2. Plans a trajectory
3. If planning succeeds, executes the plan
4. If planning fails, does nothing

---

### 2. MoveGroupInterface Core Methods

#### Motion Planning - Pose Target
```cpp
move_group_interface.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
if(success) {
    move_group_interface.execute(my_plan);
}
```

#### Motion Planning - Joint Target
```cpp
std::vector<double> joint_group_positions = move_group_interface.getCurrentJointValues();
joint_group_positions[0] = value;
move_group_interface.setJointValueTarget(joint_group_positions);
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
if(success) {
    move_group_interface.execute(my_plan);
}
```

#### Cartesian Path Planning
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(pose1);
waypoints.push_back(pose2);

moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_interface.computeCartesianPath(
    waypoints,  // waypoints to follow
    0.005,      // eef_step (max step size in meters)
    0.0,        // jump_threshold (0.0 = disabled)
    trajectory  // output trajectory
);

moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
cartesian_plan.trajectory_ = trajectory;
move_group_interface.execute(cartesian_plan);
```

---

## How to Run

### Prerequisites
```bash
# Build the workspace
cd /home/kangdongkyu/colcon_ws
colcon build --packages-select moveit2_exam
source install/setup.bash
```

### Launch Examples

#### Example 1 - Move Sample
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=1 use_sim_time:=true
```

#### Example 2 - Rotation Sample
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=2 use_sim_time:=true
```

#### Example 3 - Gripper Sample
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=3 use_sim_time:=true
```

#### Example 4 - Waypoint Sample
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=4 use_sim_time:=true
```

#### Example 5 - Rectangle Sample (To Be Implemented)
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=5 use_sim_time:=true
```

#### Example 6 - Circle Sample (To Be Implemented)
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=6 use_sim_time:=true
```

### Launch File Parameters

From `launch/moveit2_practice.launch.py`:
- `cmd`: Command value (1-6) to select which example to run
- `use_sim_time`: Set to "true" for simulation (default: "false")
- `name`: Robot name (default: "fr3")
- `prefix`: Joint name prefix for multi-robot setup

---

## Implementation Guide

### For Rectangle Drawing (cmd=5)

**Location**: `src/moveit2_practice_sample.cpp:140-148`

**Goal**: Draw a rectangle using Cartesian path planning.

**Suggested Approach**:
1. Move to a starting position
2. Create 4 waypoints representing rectangle corners
3. Use `computeCartesianPath()` similar to waypoint_sample
4. Execute the Cartesian plan

**Pseudocode**:
```cpp
void rectangle_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                      rclcpp::Node::SharedPtr node)
{
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "rectangle_sample");

    // 1. Move to starting position
    geometry_msgs::msg::Pose start_pose = list_to_pose(x, y, z, roll, pitch, yaw);
    go_to_pose_goal(move_group_interface, start_pose);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // 2. Create waypoints for rectangle (4 corners)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose waypoint = start_pose;

    // Side 1: Move along X
    waypoint.position.x += length;
    waypoints.push_back(waypoint);

    // Side 2: Move along Y
    waypoint.position.y += width;
    waypoints.push_back(waypoint);

    // Side 3: Move along -X
    waypoint.position.x -= length;
    waypoints.push_back(waypoint);

    // Side 4: Move along -Y (return to start)
    waypoint.position.y -= width;
    waypoints.push_back(waypoint);

    // 3. Compute and execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
    RCLCPP_INFO(logger, "Rectangle path fraction: %.3f", fraction);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);
}
```

**Key Considerations**:
- Choose appropriate rectangle dimensions (e.g., 0.1m x 0.05m)
- Ensure starting position is within robot's workspace
- Keep Z position constant to draw in a plane
- Consider orientation (keep constant for 2D drawing)

---

### For Circle Drawing (cmd=6)

**Location**: `src/moveit2_practice_sample.cpp:150-158`

**Goal**: Draw a circle using Cartesian path planning.

**Suggested Approach**:
1. Move to starting position (on the circle perimeter)
2. Generate multiple waypoints around the circle circumference
3. Use parametric circle equation: x = cx + r*cos(θ), y = cy + r*sin(θ)
4. Use `computeCartesianPath()` to connect the waypoints
5. Execute the Cartesian plan

**Pseudocode**:
```cpp
void circle_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,
                   rclcpp::Node::SharedPtr node)
{
    auto logger = node->get_logger();
    RCLCPP_INFO(logger, "circle_sample");

    // Circle parameters
    double center_x = 0.4;
    double center_y = 0.2;
    double center_z = 0.5;
    double radius = 0.05;  // 5cm radius
    int num_waypoints = 36;  // 36 points = 10 degrees per point

    // 1. Move to starting position (on circle perimeter at θ=0)
    geometry_msgs::msg::Pose start_pose;
    start_pose = list_to_pose(center_x + radius, center_y, center_z, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, start_pose);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // 2. Create waypoints around the circle
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (int i = 1; i <= num_waypoints; i++) {
        double theta = 2.0 * M_PI * i / num_waypoints;

        geometry_msgs::msg::Pose waypoint;
        waypoint.position.x = center_x + radius * cos(theta);
        waypoint.position.y = center_y + radius * sin(theta);
        waypoint.position.z = center_z;

        // Keep orientation constant
        tf2::Quaternion orientation;
        orientation.setRPY(-M_PI/2, 0, 0);
        waypoint.orientation = tf2::toMsg(orientation);

        waypoints.push_back(waypoint);
    }

    // 3. Compute and execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);
    RCLCPP_INFO(logger, "Circle path fraction: %.3f", fraction);

    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);
}
```

**Key Considerations**:
- Use enough waypoints for smooth circle (20-50 points recommended)
- Smaller radius = smoother circle with fewer points
- Ensure circle is within robot's reachable workspace
- Keep Z constant for 2D circle
- Consider starting at θ=0 and going full 2π to close the circle

---

## Additional Notes

### Coordinate System
- **X**: Forward/backward (base_link frame)
- **Y**: Left/right
- **Z**: Up/down
- **Roll, Pitch, Yaw**: Rotation around X, Y, Z axes respectively

### Error Handling
- Always check if `plan()` returns SUCCESS before executing
- `computeCartesianPath()` returns fraction (0.0 to 1.0)
  - 1.0 = 100% of path was successfully planned
  - <1.0 = Only partial path was planned (may still be usable)

### Performance Tips
- `setPlanningTime(45.0)` gives the planner 45 seconds max
- Smaller `eef_step` in `computeCartesianPath()` = more accurate but slower
- Too many waypoints can slow down planning

### Debugging
- Use `RCLCPP_INFO()` for logging
- Check RViz to visualize planned paths before execution
- Monitor planning time and success rates

---

## Summary

This codebase provides a clean structure for learning MoveIt2:
1. **move_sample**: Basic pose-to-pose movement
2. **rotation_sample**: Pure rotational movement
3. **gripper_sample**: Joint-space control
4. **waypoint_sample**: Cartesian path planning (foundation for drawing)
5. **rectangle_sample**: TO IMPLEMENT - Apply waypoint concepts to draw a rectangle
6. **circle_sample**: TO IMPLEMENT - Use parametric equations with Cartesian planning

The key to implementing examples 5 and 6 is understanding `computeCartesianPath()` from example 4 and applying it with appropriate waypoint generation.
