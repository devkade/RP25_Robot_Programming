# Implementation Comparison: Example 4 vs Examples 5 & 6

This document shows how examples 5 (rectangle) and 6 (circle) are implemented using the same pattern as example 4 (waypoint).

---

## The Common Pattern

All three examples follow this structure:

```
1. Move to starting position
2. Create vector of waypoints
3. Compute Cartesian path
4. Execute the trajectory
```

---

## Example 4: Waypoint Sample (Lines 108-138)

### Purpose
Demonstrates basic Cartesian path planning with 2 waypoints (L-shape).

### Code Structure
```cpp
void waypoint_sample(...) {
    // 1. Move to start
    geometry_msgs::msg::Pose target_pose = list_to_pose(0.4, 0.2, 0.6, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, target_pose);

    // 2. Create waypoints
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose start = target_pose;

    start.position.z -= 0.1;  // Move down
    waypoints.push_back(start);

    start.position.x += 0.1;  // Move forward
    waypoints.push_back(start);

    // 3. Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    // 4. Execute
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    move_group_interface.execute(cartesian_plan);
}
```

### Key Points
- **2 waypoints**: Creates an L-shape path
- **Manual waypoint creation**: Each waypoint is manually specified
- **No validation**: Executes regardless of fraction value

---

## Example 5: Rectangle Sample (Lines 140-198)

### Purpose
Draws a rectangle using 4 waypoints (4 corners).

### Code Structure
```cpp
void rectangle_sample(...) {
    // Rectangle dimensions
    double length = 0.1;  // 10cm
    double width = 0.05;  // 5cm

    // 1. Move to start (bottom-left corner)
    geometry_msgs::msg::Pose start_pose = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, start_pose);

    // 2. Create waypoints (4 corners)
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose waypoint = start_pose;

    // Corner 1: +X
    waypoint.position.x += length;
    waypoints.push_back(waypoint);

    // Corner 2: +Y
    waypoint.position.y += width;
    waypoints.push_back(waypoint);

    // Corner 3: -X
    waypoint.position.x -= length;
    waypoints.push_back(waypoint);

    // Corner 4: -Y (back to start)
    waypoint.position.y -= width;
    waypoints.push_back(waypoint);

    // 3. Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    // 4. Execute with validation
    if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group_interface.execute(cartesian_plan);
    }
}
```

### Differences from Example 4
| Aspect | Example 4 | Example 5 |
|--------|-----------|-----------|
| Waypoints | 2 | 4 |
| Shape | L-shape | Rectangle (closed loop) |
| Waypoint creation | Manual increments | Incremental with dimensions |
| Validation | None | Checks fraction > 0.8 |
| Logging | Basic | Detailed corner logging |

### Key Improvements
- **Parametric dimensions**: Easy to change rectangle size
- **Closed path**: Returns to starting position
- **Validation**: Only executes if >80% of path computed
- **Better logging**: Shows each corner position

---

## Example 6: Circle Sample (Lines 200-263)

### Purpose
Draws a circle using parametric equations with 36 waypoints.

### Code Structure
```cpp
void circle_sample(...) {
    // Circle parameters
    double center_x = 0.4, center_y = 0.2, center_z = 0.5;
    double radius = 0.05;      // 5cm
    int num_waypoints = 36;    // 10° per point

    // 1. Move to start (angle=0 on circle)
    geometry_msgs::msg::Pose start_pose = list_to_pose(
        center_x + radius, center_y, center_z, -M_PI/2, 0, 0);
    go_to_pose_goal(move_group_interface, start_pose);

    // 2. Create waypoints using parametric equations
    std::vector<geometry_msgs::msg::Pose> waypoints;

    for (int i = 1; i <= num_waypoints; i++) {
        double theta = 2.0 * M_PI * i / num_waypoints;

        geometry_msgs::msg::Pose waypoint;
        waypoint.position.x = center_x + radius * cos(theta);  // Parametric X
        waypoint.position.y = center_y + radius * sin(theta);  // Parametric Y
        waypoint.position.z = center_z;

        // Set orientation
        tf2::Quaternion orientation;
        orientation.setRPY(-M_PI/2, 0, 0);
        waypoint.orientation = tf2::toMsg(orientation);

        waypoints.push_back(waypoint);
    }

    // 3. Compute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, 0.005, 0.0, trajectory);

    // 4. Execute with validation
    if (fraction > 0.8) {
        moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
        cartesian_plan.trajectory_ = trajectory;
        move_group_interface.execute(cartesian_plan);
    }
}
```

### Differences from Example 4
| Aspect | Example 4 | Example 6 |
|--------|-----------|-----------|
| Waypoints | 2 | 36 |
| Shape | L-shape | Circle (smooth curve) |
| Waypoint creation | Manual | Loop with math (cos/sin) |
| Math required | None | Parametric equations |
| Validation | None | Checks fraction > 0.8 |
| Orientation handling | Inherited | Explicitly set per waypoint |

### Key Improvements
- **Parametric generation**: Uses circle equation (x = cx + r*cos(θ), y = cy + r*sin(θ))
- **Configurable smoothness**: Change `num_waypoints` for smoother/coarser circle
- **Closed curve**: Full 360° rotation
- **Explicit orientation**: Sets orientation for each waypoint (needed for consistency)
- **Advanced logging**: Shows progress at key angles (every 60°)

---

## Side-by-Side Comparison

### Waypoint Creation Pattern

#### Example 4: Manual Increments
```cpp
geometry_msgs::msg::Pose start = target_pose;
start.position.z -= 0.1;
waypoints.push_back(start);
start.position.x += 0.1;
waypoints.push_back(start);
```

#### Example 5: Incremental with Parameters
```cpp
geometry_msgs::msg::Pose waypoint = start_pose;
waypoint.position.x += length;    // Use parameter
waypoints.push_back(waypoint);
waypoint.position.y += width;     // Use parameter
waypoints.push_back(waypoint);
// ... etc
```

#### Example 6: Algorithmic Generation
```cpp
for (int i = 1; i <= num_waypoints; i++) {
    double theta = 2.0 * M_PI * i / num_waypoints;
    waypoint.position.x = center_x + radius * cos(theta);  // Calculate
    waypoint.position.y = center_y + radius * sin(theta);  // Calculate
    waypoints.push_back(waypoint);
}
```

---

## Common Elements (Same in All 3)

### 1. Initial Movement
All three move to a starting position first:
```cpp
geometry_msgs::msg::Pose start_pose = list_to_pose(x, y, z, roll, pitch, yaw);
go_to_pose_goal(move_group_interface, start_pose);
rclcpp::sleep_for(std::chrono::seconds(1));  // Wait before drawing
```

### 2. Waypoint Container
All three use the same vector:
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
```

### 3. Cartesian Path Computation
All three use identical API call:
```cpp
moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_interface.computeCartesianPath(
    waypoints,   // The vector of poses
    0.005,       // eef_step (5mm max step size)
    0.0,         // jump_threshold (disabled)
    trajectory   // Output trajectory
);
```

### 4. Execution
All three execute the same way:
```cpp
moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
cartesian_plan.trajectory_ = trajectory;
move_group_interface.execute(cartesian_plan);
```

---

## Complexity Progression

### Example 4: Beginner
- **Concepts**: Basic Cartesian planning
- **Waypoints**: 2 (hardcoded)
- **Math**: None (just addition/subtraction)
- **Shape**: Simple L-shape

### Example 5: Intermediate
- **Concepts**: Closed paths, parametric dimensions
- **Waypoints**: 4 (calculated from parameters)
- **Math**: Basic arithmetic
- **Shape**: Rectangle (closed loop)
- **Added**: Validation, detailed logging

### Example 6: Advanced
- **Concepts**: Parametric curves, algorithmic generation
- **Waypoints**: 36 (generated in loop)
- **Math**: Trigonometry (sin, cos)
- **Shape**: Circle (smooth closed curve)
- **Added**: Explicit orientation, progress logging

---

## Key Takeaways

### What Stays the Same
1. **Core API**: `computeCartesianPath()` is used identically
2. **Execution pattern**: Always move → create waypoints → compute → execute
3. **Data structures**: Same Pose and trajectory types

### What Changes
1. **Number of waypoints**: 2 → 4 → 36
2. **Generation method**: Manual → incremental → algorithmic
3. **Complexity**: Simple → moderate → advanced
4. **Validation**: None → basic → comprehensive

### Learning Path
1. **Start with Example 4**: Understand the basic pattern
2. **Move to Example 5**: Learn parametric dimensions
3. **Advance to Example 6**: Master algorithmic generation

---

## Testing Your Implementation

### Build
```bash
cd ~/colcon_ws
colcon build --packages-select moveit2_exam
source install/setup.bash
```

### Test Rectangle
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=5 use_sim_time:=true
```

### Test Circle
```bash
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=6 use_sim_time:=true
```

### Expected Output

#### Rectangle
```
[INFO] rectangle_sample
[INFO] Corner 1: x=0.500, y=0.200, z=0.500
[INFO] Corner 2: x=0.500, y=0.250, z=0.500
[INFO] Corner 3: x=0.400, y=0.250, z=0.500
[INFO] Corner 4: x=0.400, y=0.200, z=0.500
[INFO] Rectangle path fraction: 1.000 (100.0% complete)
[INFO] Rectangle drawing completed!
```

#### Circle
```
[INFO] circle_sample
[INFO] Waypoint 6 (60°): x=0.425, y=0.243, z=0.500
[INFO] Waypoint 12 (120°): x=0.425, y=0.243, z=0.500
[INFO] ...
[INFO] Generated 36 waypoints for circle
[INFO] Circle path fraction: 1.000 (100.0% complete)
[INFO] Circle drawing completed!
```

---

## Customization Ideas

### Modify Rectangle Size
```cpp
double length = 0.15;  // 15cm instead of 10cm
double width = 0.08;   // 8cm instead of 5cm
```

### Modify Circle Smoothness
```cpp
int num_waypoints = 50;  // Smoother (more points)
int num_waypoints = 20;  // Coarser (fewer points)
```

### Modify Circle Radius
```cpp
double radius = 0.08;  // 8cm circle instead of 5cm
```

---

## Summary

Examples 5 and 6 are **direct extensions** of Example 4:
- Same API (`computeCartesianPath`)
- Same pattern (move → waypoints → compute → execute)
- Different waypoint generation (manual → parametric → algorithmic)

Once you understand Example 4, Examples 5 and 6 are just about **how you generate the waypoints**.
