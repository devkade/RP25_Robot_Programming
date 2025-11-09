# MoveIt2 API Reference

This document provides detailed API information for the MoveIt2 interfaces used in this project.

---

## Table of Contents
1. [MoveGroupInterface](#movegroupinterface)
2. [Helper Functions](#helper-functions)
3. [Message Types](#message-types)
4. [Planning Parameters](#planning-parameters)

---

## MoveGroupInterface

The primary interface to MoveIt2 for motion planning and execution.

### Constructor
```cpp
moveit::planning_interface::MoveGroupInterface::MoveGroupInterface(
    const rclcpp::Node::SharedPtr& node,
    const std::string& group_name
)
```

**Parameters**:
- `node`: Shared pointer to ROS2 node
- `group_name`: Name of the planning group (e.g., "manipulator", "gripper")

**Example**:
```cpp
auto node = rclcpp::Node::make_shared("my_node");
moveit::planning_interface::MoveGroupInterface arm(node, "manipulator");
moveit::planning_interface::MoveGroupInterface gripper(node, "gripper");
```

---

### Core Methods

#### setPoseTarget()
```cpp
bool setPoseTarget(const geometry_msgs::msg::Pose& pose)
```

Sets the target pose for the end-effector.

**Parameters**:
- `pose`: Target pose (position + orientation)

**Returns**: `true` if target is set successfully

**Example**:
```cpp
geometry_msgs::msg::Pose target;
target = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
move_group_interface.setPoseTarget(target);
```

---

#### setJointValueTarget()
```cpp
bool setJointValueTarget(const std::vector<double>& joint_values)
```

Sets target joint positions directly.

**Parameters**:
- `joint_values`: Vector of joint angles (in radians)

**Returns**: `true` if target is set successfully

**Example**:
```cpp
std::vector<double> joint_positions = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
move_group_interface.setJointValueTarget(joint_positions);
```

---

#### plan()
```cpp
moveit::planning_interface::MoveItErrorCode plan(
    moveit::planning_interface::MoveGroupInterface::Plan& plan
)
```

Computes a motion plan to the set target.

**Parameters**:
- `plan`: Output parameter that will contain the planned trajectory

**Returns**: `MoveItErrorCode` indicating success or failure

**Example**:
```cpp
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
```

---

#### execute()
```cpp
moveit::planning_interface::MoveItErrorCode execute(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan
)
```

Executes a previously computed plan.

**Parameters**:
- `plan`: The plan to execute

**Returns**: `MoveItErrorCode` indicating success or failure

**Example**:
```cpp
if (success) {
    move_group_interface.execute(my_plan);
}
```

---

#### computeCartesianPath()
```cpp
double computeCartesianPath(
    const std::vector<geometry_msgs::msg::Pose>& waypoints,
    double eef_step,
    double jump_threshold,
    moveit_msgs::msg::RobotTrajectory& trajectory,
    bool avoid_collisions = true
)
```

Computes a Cartesian path through a sequence of waypoints.

**Parameters**:
- `waypoints`: Vector of poses to pass through
- `eef_step`: Maximum step size for interpolation (meters)
- `jump_threshold`: Maximum joint-space jump threshold (0.0 = disabled)
- `trajectory`: Output trajectory
- `avoid_collisions`: Whether to check for collisions (default: true)

**Returns**: Fraction of path successfully computed (0.0 to 1.0)

**Example**:
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(pose1);
waypoints.push_back(pose2);

moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    0.005,      // 5mm step size
    0.0,        // no jump threshold
    trajectory
);

RCLCPP_INFO(logger, "Cartesian path: %.2f%% achieved", fraction * 100.0);
```

---

#### getCurrentJointValues()
```cpp
std::vector<double> getCurrentJointValues()
```

Gets the current joint positions.

**Returns**: Vector of current joint angles

**Example**:
```cpp
std::vector<double> current_joints = move_group_interface.getCurrentJointValues();
for (size_t i = 0; i < current_joints.size(); i++) {
    RCLCPP_INFO(logger, "Joint %zu: %.3f", i, current_joints[i]);
}
```

---

#### getCurrentPose()
```cpp
geometry_msgs::msg::PoseStamped getCurrentPose()
```

Gets the current pose of the end-effector.

**Returns**: Current pose with timestamp and frame

**Example**:
```cpp
geometry_msgs::msg::PoseStamped current_pose = move_group_interface.getCurrentPose();
RCLCPP_INFO(logger, "Current position: x=%.3f, y=%.3f, z=%.3f",
    current_pose.pose.position.x,
    current_pose.pose.position.y,
    current_pose.pose.position.z);
```

---

#### setPlanningTime()
```cpp
void setPlanningTime(double seconds)
```

Sets the maximum time allowed for planning.

**Parameters**:
- `seconds`: Maximum planning time

**Example**:
```cpp
move_group_interface.setPlanningTime(45.0);  // 45 seconds
```

---

#### setMaxVelocityScalingFactor()
```cpp
void setMaxVelocityScalingFactor(double factor)
```

Sets the velocity scaling factor (0.0 to 1.0).

**Parameters**:
- `factor`: Scaling factor (0.1 = 10% of max velocity, 1.0 = 100%)

**Example**:
```cpp
move_group_interface.setMaxVelocityScalingFactor(0.5);  // 50% speed
```

---

#### setMaxAccelerationScalingFactor()
```cpp
void setMaxAccelerationScalingFactor(double factor)
```

Sets the acceleration scaling factor (0.0 to 1.0).

**Parameters**:
- `factor`: Scaling factor

**Example**:
```cpp
move_group_interface.setMaxAccelerationScalingFactor(0.3);  // 30% accel
```

---

## Helper Functions

### list_to_pose()
```cpp
geometry_msgs::msg::Pose list_to_pose(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw
)
```

Converts position and RPY orientation to a Pose message.

**Parameters**:
- `x`, `y`, `z`: Position in meters
- `roll`, `pitch`, `yaw`: Orientation in radians

**Returns**: `geometry_msgs::msg::Pose`

**Example**:
```cpp
auto pose = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
```

**Implementation Details**:
- Uses `tf2::Quaternion` for orientation conversion
- Calls `setRPY()` to convert Euler angles to quaternion
- Returns a complete Pose message ready for MoveIt2

---

### go_to_pose_goal()
```cpp
void go_to_pose_goal(
    moveit::planning_interface::MoveGroupInterface& move_group_interface,
    geometry_msgs::msg::Pose& target_pose
)
```

Plans and executes movement to a target pose.

**Parameters**:
- `move_group_interface`: Reference to MoveGroupInterface
- `target_pose`: Target pose to move to

**Returns**: void

**Behavior**:
- Plans a trajectory to the target pose
- Executes the plan if planning succeeds
- Does nothing if planning fails

**Example**:
```cpp
geometry_msgs::msg::Pose target = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
go_to_pose_goal(arm, target);
```

---

## Message Types

### geometry_msgs::msg::Pose
```cpp
struct Pose {
    Point position;        // x, y, z
    Quaternion orientation; // x, y, z, w
}
```

**Fields**:
- `position.x`, `position.y`, `position.z`: Position coordinates (meters)
- `orientation`: Quaternion representation of orientation

**Usage**:
```cpp
geometry_msgs::msg::Pose pose;
pose.position.x = 0.4;
pose.position.y = 0.2;
pose.position.z = 0.5;

tf2::Quaternion q;
q.setRPY(-M_PI/2, 0, 0);
pose.orientation = tf2::toMsg(q);
```

---

### moveit::planning_interface::MoveGroupInterface::Plan
```cpp
struct Plan {
    moveit_msgs::msg::RobotTrajectory trajectory_;
    double planning_time_;
}
```

**Fields**:
- `trajectory_`: The planned robot trajectory
- `planning_time_`: Time taken to compute the plan (seconds)

**Usage**:
```cpp
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
if (success) {
    move_group_interface.execute(my_plan);
}
```

---

### moveit_msgs::msg::RobotTrajectory
```cpp
struct RobotTrajectory {
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
}
```

Contains the joint-space trajectory for the robot.

**Usage with Cartesian Path**:
```cpp
moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_interface.computeCartesianPath(
    waypoints, 0.005, 0.0, trajectory);

moveit::planning_interface::MoveGroupInterface::Plan plan;
plan.trajectory_ = trajectory;
move_group_interface.execute(plan);
```

---

### tf2::Quaternion
```cpp
class Quaternion {
public:
    void setRPY(double roll, double pitch, double yaw);
    // ... other methods
}
```

**Methods**:
- `setRPY(roll, pitch, yaw)`: Sets quaternion from Euler angles (radians)
- `setRotation(axis, angle)`: Sets quaternion from axis-angle
- `normalize()`: Normalizes the quaternion

**Usage**:
```cpp
tf2::Quaternion orientation;
orientation.setRPY(-M_PI/2, 0, 0);  // Roll=-90°, Pitch=0, Yaw=0

geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(orientation);
pose.orientation = msg_quat;
```

---

## Planning Parameters

### Planning Time
```cpp
arm.setPlanningTime(45.0);  // Maximum 45 seconds for planning
```

Affects:
- Maximum time spent searching for a valid plan
- Longer time = higher chance of finding solution
- Does NOT affect execution time

---

### Velocity Scaling
```cpp
arm.setMaxVelocityScalingFactor(0.5);  // 50% of max velocity
```

Affects:
- Maximum velocity during trajectory execution
- Range: 0.0 (stopped) to 1.0 (max velocity)
- Lower values = slower, smoother motion

---

### Acceleration Scaling
```cpp
arm.setMaxAccelerationScalingFactor(0.3);  // 30% of max acceleration
```

Affects:
- Maximum acceleration during trajectory execution
- Range: 0.0 to 1.0
- Lower values = gentler acceleration/deceleration

---

### Cartesian Path Parameters

#### eef_step
```cpp
double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    0.005,  // eef_step = 5mm
    0.0,
    trajectory
);
```

- Step size for interpolation between waypoints
- Smaller values = more accurate path, but slower planning
- Typical values: 0.001 to 0.01 (1mm to 10mm)

#### jump_threshold
```cpp
double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    0.005,
    0.0,  // jump_threshold = disabled
    trajectory
);
```

- Maximum allowed jump in joint space
- 0.0 = disabled (no check)
- Prevents sudden large joint movements

---

## Error Codes

### MoveItErrorCode Values
```cpp
moveit::planning_interface::MoveItErrorCode::SUCCESS    // Planning succeeded
moveit::planning_interface::MoveItErrorCode::FAILURE    // Planning failed
moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED
moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN
moveit::planning_interface::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE
moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED
moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA
moveit::planning_interface::MoveItErrorCode::TIMED_OUT
moveit::planning_interface::MoveItErrorCode::PREEMPTED
```

**Usage**:
```cpp
auto result = move_group_interface.plan(my_plan);
if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(logger, "Planning succeeded");
} else if (result == moveit::planning_interface::MoveItErrorCode::TIMED_OUT) {
    RCLCPP_ERROR(logger, "Planning timed out");
} else {
    RCLCPP_ERROR(logger, "Planning failed");
}
```

---

## Best Practices

### Always Check Planning Success
```cpp
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
if (!success) {
    RCLCPP_ERROR(logger, "Planning failed");
    return;
}
move_group_interface.execute(my_plan);
```

### Check Cartesian Path Fraction
```cpp
double fraction = move_group_interface.computeCartesianPath(...);
if (fraction < 0.9) {
    RCLCPP_WARN(logger, "Only %.1f%% of path was planned", fraction * 100.0);
}
```

### Use Appropriate Planning Time
```cpp
// Simple movements
arm.setPlanningTime(10.0);

// Complex movements with obstacles
arm.setPlanningTime(45.0);
```

### Scale Velocity for Safety
```cpp
// During testing/debugging
arm.setMaxVelocityScalingFactor(0.1);  // 10% speed

// Normal operation
arm.setMaxVelocityScalingFactor(0.5);  // 50% speed
```

---

## Common Issues and Solutions

### Issue: Planning Always Fails
**Solutions**:
- Increase planning time
- Check if target is within workspace
- Verify no self-collisions
- Check joint limits

### Issue: Cartesian Path Fraction < 1.0
**Solutions**:
- Reduce waypoint distances
- Check for obstacles
- Verify all waypoints are reachable
- Increase planning time

### Issue: Robot Moves Too Fast
**Solutions**:
```cpp
arm.setMaxVelocityScalingFactor(0.3);
arm.setMaxAccelerationScalingFactor(0.3);
```

### Issue: Jerky Motion
**Solutions**:
- Use Cartesian paths for smooth motion
- Reduce eef_step in computeCartesianPath()
- Add more intermediate waypoints
