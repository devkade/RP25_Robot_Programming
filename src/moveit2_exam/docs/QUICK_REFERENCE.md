# MoveIt2 Quick Reference Guide

## Common Commands

### Build and Source
```bash
cd ~/colcon_ws
colcon build --packages-select moveit2_exam
source install/setup.bash
```

### Run Examples
```bash
# Example 1: Move
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=1 use_sim_time:=true

# Example 2: Rotate
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=2 use_sim_time:=true

# Example 3: Gripper
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=3 use_sim_time:=true

# Example 4: Waypoint
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=4 use_sim_time:=true

# Example 5: Rectangle (to implement)
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=5 use_sim_time:=true

# Example 6: Circle (to implement)
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=6 use_sim_time:=true
```

---

## Code Snippets

### 1. Create a Pose
```cpp
geometry_msgs::msg::Pose target_pose;
target_pose = list_to_pose(x, y, z, roll, pitch, yaw);
```

### 2. Move to a Pose
```cpp
go_to_pose_goal(move_group_interface, target_pose);
```

### 3. Move with Manual Planning
```cpp
move_group_interface.setPoseTarget(target_pose);
moveit::planning_interface::MoveGroupInterface::Plan my_plan;
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
if(success) {
    move_group_interface.execute(my_plan);
}
```

### 4. Joint Space Control
```cpp
std::vector<double> joint_group_positions;
joint_group_positions = move_group_interface.getCurrentJointValues();
joint_group_positions[0] = value;  // Set joint 0
move_group_interface.setJointValueTarget(joint_group_positions);
```

### 5. Cartesian Path Planning
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;

// Add waypoints
geometry_msgs::msg::Pose waypoint1 = start_pose;
waypoint1.position.x += 0.1;
waypoints.push_back(waypoint1);

geometry_msgs::msg::Pose waypoint2 = waypoint1;
waypoint2.position.y += 0.1;
waypoints.push_back(waypoint2);

// Compute path
moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_interface.computeCartesianPath(
    waypoints,   // waypoints to follow
    0.005,       // eef_step (5mm)
    0.0,         // jump_threshold
    trajectory   // output
);

// Execute
moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
cartesian_plan.trajectory_ = trajectory;
move_group_interface.execute(cartesian_plan);
```

---

## Common Patterns

### Pattern 1: Simple Movement Sequence
```cpp
void example(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
    geometry_msgs::msg::Pose pose1 = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, pose1);

    geometry_msgs::msg::Pose pose2 = list_to_pose(0.5, 0.2, 0.5, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, pose2);
}
```

### Pattern 2: Incremental Movement
```cpp
void example(moveit::planning_interface::MoveGroupInterface &move_group_interface) {
    geometry_msgs::msg::Pose target_pose = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, target_pose);

    // Move incrementally
    target_pose.position.x += 0.1;
    go_to_pose_goal(move_group_interface, target_pose);

    target_pose.position.y -= 0.05;
    go_to_pose_goal(move_group_interface, target_pose);
}
```

### Pattern 3: Waypoint-based Drawing
```cpp
void example(moveit::planning_interface::MoveGroupInterface &move_group_interface,
             rclcpp::Node::SharedPtr node) {
    auto logger = node->get_logger();

    // 1. Go to start
    geometry_msgs::msg::Pose start = list_to_pose(0.4, 0.2, 0.6, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, start);

    rclcpp::sleep_for(std::chrono::seconds(1));

    // 2. Create waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose point = start;

    point.position.x += 0.1;
    waypoints.push_back(point);

    point.position.y += 0.1;
    waypoints.push_back(point);

    // 3. Execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.005, 0.0, trajectory);
    RCLCPP_INFO(logger, "Path completion: %.3f", fraction);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_interface.execute(plan);
}
```

---

## Coordinate Reference

### Typical Workspace Bounds (Franka FR3)
- **X**: 0.2 to 0.8 meters (forward)
- **Y**: -0.3 to 0.3 meters (left/right)
- **Z**: 0.1 to 0.8 meters (height)

### Common Orientations
```cpp
// Downward facing (typical for picking)
list_to_pose(x, y, z, -M_PI/2, 0, 0)

// Horizontal (gripper parallel to ground)
list_to_pose(x, y, z, 0, 0, 0)

// Rotated 90 degrees around Z
list_to_pose(x, y, z, -M_PI/2, 0, M_PI/2)
```

---

## Debugging Tips

### Check Planning Success
```cpp
bool success = (move_group_interface.plan(my_plan) ==
                moveit::planning_interface::MoveItErrorCode::SUCCESS);
if (!success) {
    RCLCPP_ERROR(logger, "Planning failed!");
    return;
}
```

### Log Cartesian Path Fraction
```cpp
double fraction = move_group_interface.computeCartesianPath(...);
RCLCPP_INFO(logger, "Cartesian path %.2f%% complete", fraction * 100.0);
if (fraction < 0.9) {
    RCLCPP_WARN(logger, "Path may be incomplete!");
}
```

### Add Delays Between Movements
```cpp
go_to_pose_goal(move_group_interface, pose1);
rclcpp::sleep_for(std::chrono::seconds(2));  // Wait 2 seconds
go_to_pose_goal(move_group_interface, pose2);
```

---

## Mathematical Helpers

### Circle Generation
```cpp
// Generate points on a circle
double center_x = 0.4;
double center_y = 0.2;
double radius = 0.05;
int num_points = 36;

for (int i = 0; i < num_points; i++) {
    double theta = 2.0 * M_PI * i / num_points;
    double x = center_x + radius * cos(theta);
    double y = center_y + radius * sin(theta);
    // Create waypoint with x, y
}
```

### Rectangle Generation
```cpp
// Rectangle with corners
double x_start = 0.4, y_start = 0.2;
double length = 0.1, width = 0.05;

// Corner 1: (x_start, y_start)
// Corner 2: (x_start + length, y_start)
// Corner 3: (x_start + length, y_start + width)
// Corner 4: (x_start, y_start + width)
```

---

## File Locations Quick Reference

| Component | File Path |
|-----------|-----------|
| Main source | `src/moveit2_practice_sample.cpp` |
| Launch file | `launch/moveit2_practice.launch.py` |
| CMake config | `CMakeLists.txt` |
| Package info | `package.xml` |
| Kinematics | `moveit_config/kinematics.yaml` |
| Joint limits | `moveit_config/joint_limits_6dof_gripper.yaml` |
| Controllers | `moveit_config/controllers_6dof_gripper.yaml` |
| OMPL config | `moveit_config/ompl_planning.yaml` |

---

## Function Reference Summary

| Function | Purpose | Location |
|----------|---------|----------|
| `list_to_pose()` | Convert x,y,z,r,p,y to Pose | Line 27 |
| `go_to_pose_goal()` | Plan and execute to pose | Line 13 |
| `move_sample()` | Example 1: Basic movement | Line 40 |
| `rotation_sample()` | Example 2: Rotation | Line 72 |
| `gripper_sample()` | Example 3: Gripper control | Line 88 |
| `waypoint_sample()` | Example 4: Cartesian path | Line 108 |
| `rectangle_sample()` | Example 5: TO IMPLEMENT | Line 140 |
| `circle_sample()` | Example 6: TO IMPLEMENT | Line 150 |

---

## Implementation Checklist

### For Rectangle Drawing
- [ ] Define rectangle dimensions (length, width)
- [ ] Choose starting position
- [ ] Create 4 corner waypoints
- [ ] Use computeCartesianPath()
- [ ] Check fraction value
- [ ] Execute plan
- [ ] Test and verify

### For Circle Drawing
- [ ] Define circle center and radius
- [ ] Choose number of waypoints (20-50)
- [ ] Generate points using parametric equation
- [ ] Create waypoint vector
- [ ] Use computeCartesianPath()
- [ ] Check fraction value
- [ ] Execute plan
- [ ] Test and verify
