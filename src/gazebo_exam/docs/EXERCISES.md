# Practice Exercises and Homework

This document describes the learning progression, practice exercises, and homework assignments in the gazebo_exam package.

## Learning Progression

The package follows a structured learning path from basic visualization to advanced manipulation:

```
Week 1-2: Understanding the Robot
  └─ Visualization and basic simulation

Week 3-4: Motion Planning Basics
  └─ MoveIt2 and reference examples

Week 5: Basic Assignment
  └─ Step 1 Practice

Week 6-7: Advanced Assignment
  └─ Step 2 Homework (Pick and Place)
```

## Source Files Overview

| File | Lines | Status | Difficulty | Purpose |
|------|-------|--------|------------|---------|
| moveit2_practice_sample.cpp | 210 | Complete | Reference | 6 demonstrations |
| step1_practice.cpp | 226 | Template | Beginner | Basic motion |
| step2_hw.cpp | 17 | Incomplete | Advanced | Pick-and-place |

---

## Reference Implementation: moveit2_practice_sample.cpp

### Overview

Complete reference implementation demonstrating MoveIt2 API usage with 6 different motion scenarios.

### File Location
`src/moveit2_practice_sample.cpp`

### Dependencies
```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
```

### Utility Functions

#### 1. list_to_pose()
```cpp
geometry_msgs::msg::Pose list_to_pose(
    double x, double y, double z,
    double roll, double pitch, double yaw
)
```

**Purpose:** Convert position and Euler angles to Pose message

**Parameters:**
- x, y, z: Position in meters
- roll, pitch, yaw: Orientation in radians

**Returns:** `geometry_msgs::msg::Pose` with quaternion orientation

**Example:**
```cpp
auto pose = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
```

#### 2. go_to_pose_goal()
```cpp
void go_to_pose_goal(
    moveit::planning_interface::MoveGroupInterface &move_group_interface,
    geometry_msgs::msg::Pose &target_pose
)
```

**Purpose:** Plan and execute motion to target pose

**Parameters:**
- move_group_interface: MoveIt planning group
- target_pose: Desired end-effector pose

**Behavior:**
- Calls `setPoseTarget()`
- Plans trajectory
- Executes if planning succeeds

### Demonstrations

#### Demo 1: move_sample() (cmd=1)

**Scenario:** Basic incremental movements along X, Y, Z axes

**Code Pattern:**
```cpp
geometry_msgs::msg::Pose target_pose;
target_pose = list_to_pose(0.4, 0.2, 0.5, -M_PI/2, 0, 0);
go_to_pose_goal(move_group_interface, target_pose);

target_pose.position.y += 0.05;  // Move along Y
go_to_pose_goal(move_group_interface, target_pose);

target_pose.position.x += 0.05;  // Move along X
go_to_pose_goal(move_group_interface, target_pose);

target_pose.position.z += 0.05;  // Move along Z
go_to_pose_goal(move_group_interface, target_pose);
```

**Learning Objectives:**
- Understanding Cartesian space motion
- Pose message structure
- Incremental position changes

#### Demo 2: rotation_sample() (cmd=2)

**Scenario:** Rotate end-effector around pitch axis

**Code Pattern:**
```cpp
target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2, 0, 0);
go_to_pose_goal(move_group_interface, target_pose);

target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2, 0.3, 0);  // +0.3 rad pitch
go_to_pose_goal(move_group_interface, target_pose);

target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2, -0.3, 0); // -0.3 rad pitch
go_to_pose_goal(move_group_interface, target_pose);

target_pose = list_to_pose(0.34, 0.2, 0.3, -M_PI/2, 0, 0);    // Back to 0
go_to_pose_goal(move_group_interface, target_pose);
```

**Learning Objectives:**
- Orientation control
- Roll-pitch-yaw Euler angles
- Quaternion conversion (automatic)

#### Demo 3: gripper_sample() (cmd=3)

**Scenario:** Open and close gripper

**Code Pattern:**
```cpp
void gripper_sample(
    moveit::planning_interface::MoveGroupInterface &gripper_group_interface,
    rclcpp::Node::SharedPtr node
)
{
    std::vector<double> gripper_joint_values = {0.07};  // Open (70mm)
    gripper_group_interface.setJointValueTarget(gripper_joint_values);
    gripper_group_interface.move();

    // Wait for user input
    std::cin.get();

    gripper_joint_values = {0.0};  // Close (0mm)
    gripper_group_interface.setJointValueTarget(gripper_joint_values);
    gripper_group_interface.move();
}
```

**Learning Objectives:**
- Joint space control (vs Cartesian)
- Gripper planning group
- Synchronized motion

#### Demo 4: waypoint_sample() (cmd=4)

**Scenario:** Cartesian path through multiple waypoints

**Code Pattern:**
```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;

geometry_msgs::msg::Pose current_pose = move_group_interface.getCurrentPose().pose;
waypoints.push_back(current_pose);

// Move down
current_pose.position.z -= 0.1;
waypoints.push_back(current_pose);

// Move horizontally
current_pose.position.y += 0.1;
waypoints.push_back(current_pose);

// Compute Cartesian path
moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_interface.computeCartesianPath(
    waypoints,
    0.01,  // eef_step (1cm resolution)
    0.0,   // jump_threshold (disabled)
    trajectory
);

// Execute if successful
if (fraction > 0.95) {
    move_group_interface.execute(trajectory);
}
```

**Learning Objectives:**
- Cartesian path planning
- Waypoint sequences
- Path fraction (0-1 success metric)
- Straight-line motion

#### Demo 5: rectangle_sample() (cmd=5) - HOMEWORK

**Assignment:** Implement rectangular trajectory

**Expected Implementation:**
```cpp
void rectangle_sample(...) {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    // Starting pose
    auto current_pose = list_to_pose(0.3, 0.3, 0.3, -M_PI/2, 0, 0);
    waypoints.push_back(current_pose);

    // Rectangle corners (student implements)
    // Corner 1: Move right
    current_pose.position.y -= 0.2;
    waypoints.push_back(current_pose);

    // Corner 2: Move down
    current_pose.position.z -= 0.2;
    waypoints.push_back(current_pose);

    // Corner 3: Move left
    current_pose.position.y += 0.2;
    waypoints.push_back(current_pose);

    // Corner 4: Move up (back to start)
    current_pose.position.z += 0.2;
    waypoints.push_back(current_pose);

    // Execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory
    );

    if (fraction > 0.95) {
        move_group_interface.execute(trajectory);
    }
}
```

#### Demo 6: circle_sample() (cmd=6) - HOMEWORK

**Assignment:** Implement circular trajectory

**Expected Implementation:**
```cpp
void circle_sample(...) {
    std::vector<geometry_msgs::msg::Pose> waypoints;

    double center_y = 0.0;
    double center_z = 0.4;
    double radius = 0.1;
    int num_points = 50;

    for (int i = 0; i <= num_points; i++) {
        double angle = 2.0 * M_PI * i / num_points;
        double y = center_y + radius * cos(angle);
        double z = center_z + radius * sin(angle);

        auto waypoint = list_to_pose(0.4, y, z, -M_PI/2, 0, 0);
        waypoints.push_back(waypoint);
    }

    // Execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(
        waypoints, 0.01, 0.0, trajectory
    );

    if (fraction > 0.95) {
        move_group_interface.execute(trajectory);
    }
}
```

### Running Reference Code

```bash
# Move sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=1

# Rotation sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=2

# Gripper sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=3

# Waypoint sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=4

# Rectangle (if implemented)
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=5

# Circle (if implemented)
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=6
```

---

## Step 1 Practice: step1_practice.cpp

### Overview

Basic student assignment - implement motion planning functions similar to moveit2_practice_sample.

### File Location
`src/step1_practice.cpp`

### Current Status
Template file with basic structure - students must implement motion logic.

### Expected Structure

```cpp
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Data classes
class Location {
public:
    double x, y;
    Location(double x, double y) : x(x), y(y) {}
};

class Block {
public:
    double width, length, height, radius;
    Location location;
    Block(double w, double l, double h, double r, Location loc)
        : width(w), length(l), height(h), radius(r), location(loc) {}
};

// Utility functions
geometry_msgs::msg::Pose list_to_pose(...);
void go_to_pose_goal(...);

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("step1_practice");

    // Initialize MoveGroupInterface
    auto move_group = moveit::planning_interface::MoveGroupInterface(node, "manipulator");

    // Set planning parameters
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);

    // TODO: Implement motion logic

    rclcpp::shutdown();
    return 0;
}
```

### Assignment Requirements

1. **Implement list_to_pose()**
   - Convert position and orientation to Pose

2. **Implement go_to_pose_goal()**
   - Plan and execute motion

3. **Implement basic movements**
   - Move to initial position
   - Move to block locations
   - Return to home position

4. **Test with simulation**
   - Use step1_practice_gazebo.launch.py
   - Verify motion with 2 boxes

### Development Workflow

1. **Study Reference Code**
   ```bash
   # Read moveit2_practice_sample.cpp
   cat src/moveit2_practice_sample.cpp
   ```

2. **Implement Functions**
   ```bash
   # Edit step1_practice.cpp
   # Copy relevant functions from reference
   ```

3. **Build**
   ```bash
   colcon build --packages-select gazebo_exam
   source install/setup.bash
   ```

4. **Test Without Gazebo** (faster)
   ```bash
   ros2 launch gazebo_exam step1_practice.launch.py
   ```

5. **Test With Gazebo** (full simulation)
   ```bash
   ros2 launch gazebo_exam step1_practice_gazebo.launch.py
   ```

### Grading Criteria (Example)

- [ ] Code compiles without errors (20%)
- [ ] Utility functions implemented correctly (20%)
- [ ] Robot moves to target positions (30%)
- [ ] No collisions with objects (15%)
- [ ] Code quality and comments (15%)

---

## Step 2 Homework: step2_hw.cpp

### Overview

Advanced assignment - implement pick-and-place manipulation using object-oriented design.

### File Location
`src/step2_hw.cpp`

### Current Status
Minimal stub - students must implement entire PickAndLiftNode class.

### Expected Architecture

```cpp
class PickAndLiftNode : public rclcpp::Node {
public:
    PickAndLiftNode() : Node("pick_and_lift_node") {
        // Initialize move groups
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "manipulator"
        );
        gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "gripper"
        );
    }

    void run() {
        // Main execution logic
        add_ground_plane();
        initial_pose();

        // Pick and place sequence
        lift_block(block1);
        place_block(block1, target_location);

        initial_pose();
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;

    void initial_pose();
    void lift_block(const Block& block);
    void place_block(const Block& block, const Location& target);
    void open_gripper();
    void close_gripper(double closing_value = 0.0);
    void add_ground_plane();
};
```

### Required Functions

#### 1. initial_pose()
```cpp
void initial_pose() {
    std::vector<double> joint_values = {
        0.0,      // joint0
        -2.03,    // joint1
        1.58,     // joint2
        -1.19,    // joint3
        -1.58,    // joint4
        0.78      // joint5
    };
    move_group_->setJointValueTarget(joint_values);
    move_group_->move();
}
```

**Purpose:** Move robot to safe starting configuration

#### 2. lift_block()
```cpp
void lift_block(const Block& block) {
    open_gripper();

    // Approach from above
    double approach_height = 0.4;
    auto approach_pose = list_to_pose(
        0.4, block.location.y, approach_height,
        -M_PI/2, 0, 0
    );
    go_to_pose_goal(*move_group_, approach_pose);

    // Descend to grasp height
    double grasp_z = block.height / 2.0 + gripper_offset;
    auto grasp_pose = approach_pose;
    grasp_pose.position.z = grasp_z;
    go_to_pose_goal(*move_group_, grasp_pose);

    // Close gripper
    close_gripper(0.01);  // Slight closing force

    // Lift
    grasp_pose.position.z = approach_height;
    go_to_pose_goal(*move_group_, grasp_pose);
}
```

**Purpose:** Pick up block from location

#### 3. place_block()
```cpp
void place_block(const Block& block, const Location& target) {
    // Move to target location (high)
    auto approach_pose = list_to_pose(
        0.4, target.y, 0.4,
        -M_PI/2, 0, 0
    );
    go_to_pose_goal(*move_group_, approach_pose);

    // Descend to place height
    double place_z = block.height / 2.0 + gripper_offset;
    auto place_pose = approach_pose;
    place_pose.position.z = place_z;
    go_to_pose_goal(*move_group_, place_pose);

    // Release
    open_gripper();

    // Retract
    place_pose.position.z = 0.4;
    go_to_pose_goal(*move_group_, place_pose);
}
```

**Purpose:** Place block at target location

#### 4. open_gripper() / close_gripper()
```cpp
void open_gripper() {
    std::vector<double> gripper_values = {0.07};  // 70mm
    gripper_group_->setJointValueTarget(gripper_values);
    gripper_group_->move();
}

void close_gripper(double closing_value = 0.0) {
    std::vector<double> gripper_values = {closing_value};
    gripper_group_->setJointValueTarget(gripper_values);
    gripper_group_->move();
}
```

**Purpose:** Control gripper

#### 5. add_ground_plane()
```cpp
void add_ground_plane() {
    moveit_msgs::msg::CollisionObject ground;
    ground.header.frame_id = "world";
    ground.id = "ground_plane";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {2.0, 2.0, 0.01};

    geometry_msgs::msg::Pose pose;
    pose.position.z = -0.005;
    pose.orientation.w = 1.0;

    ground.primitives.push_back(primitive);
    ground.primitive_poses.push_back(pose);
    ground.operation = ground.ADD;

    planning_scene_.applyCollisionObject(ground);
}
```

**Purpose:** Add collision object for safety

### Task Specification

**Objective:** Pick box1 from location1 and place at location2

**Initial Setup:**
```cpp
Block block1(0.05, 0.05, 0.05, 0.025, Location(0.4, 0.1));
Block block2(0.05, 0.05, 0.05, 0.025, Location(0.4, -0.1));

Location target = block2.location;
```

**Expected Behavior:**
1. Move to initial pose
2. Open gripper
3. Approach block1 from above
4. Descend and grasp
5. Lift block1
6. Move to block2 location
7. Place block1 on top of block2
8. Release and retract
9. Return to initial pose

### Advanced Features

#### Threading
```cpp
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndLiftNode>();

    std::thread executor_thread([node]() {
        rclcpp::spin(node);
    });

    node->run();

    rclcpp::shutdown();
    executor_thread.join();
    return 0;
}
```

#### Collision Avoidance

```cpp
// Check for collisions before execution
moveit::planning_interface::MoveGroupInterface::Plan plan;
auto result = move_group_->plan(plan);

if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Planning failed!");
    return;
}

move_group_->execute(plan);
```

### Testing

**Quick Test (no Gazebo):**
```bash
ros2 launch gazebo_exam step2_hw.launch.py
```

**Full Simulation (8 objects):**
```bash
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
```

### Grading Criteria (Example)

- [ ] Code compiles without errors (10%)
- [ ] PickAndLiftNode class implemented (20%)
- [ ] Pick operation successful (25%)
- [ ] Place operation successful (25%)
- [ ] Collision avoidance working (10%)
- [ ] Code quality and documentation (10%)

---

## Tips and Best Practices

### General Tips

1. **Start Simple**
   - Test individual functions first
   - Add complexity incrementally

2. **Use Logging**
   ```cpp
   RCLCPP_INFO(node->get_logger(), "Moving to position: %.2f, %.2f, %.2f", x, y, z);
   ```

3. **Check Return Values**
   ```cpp
   bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
   if (!success) {
       RCLCPP_ERROR(logger, "Planning failed!");
       return;
   }
   ```

4. **Visualize in RViz**
   - Always run with RViz to see planned paths
   - Check for unexpected trajectories

5. **Test Without Gazebo First**
   - Much faster iteration
   - Same planning logic
   - Add physics later

### Common Pitfalls

**Pitfall 1: Unreachable Poses**
```cpp
// BAD: May be outside workspace
auto pose = list_to_pose(2.0, 0.0, 0.0, 0, 0, 0);

// GOOD: Within typical workspace
auto pose = list_to_pose(0.5, 0.2, 0.4, -M_PI/2, 0, 0);
```

**Pitfall 2: Ignoring Planning Failures**
```cpp
// BAD: No error checking
move_group.setPoseTarget(pose);
move_group.move();

// GOOD: Check success
auto result = move_group.plan(plan);
if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
    move_group.execute(plan);
} else {
    RCLCPP_ERROR(logger, "Planning failed!");
}
```

**Pitfall 3: Collision Checking Disabled**
```cpp
// GOOD: Always use planning scene for obstacles
planning_scene_interface.applyCollisionObject(obstacle);
```

**Pitfall 4: Gripper Not Waiting**
```cpp
// BAD: Move immediately after gripper command
close_gripper();
go_to_pose_goal(move_group, next_pose);

// GOOD: Gripper command already waits (blocking)
close_gripper();  // Waits for completion
go_to_pose_goal(move_group, next_pose);
```

### Debugging

**Enable Verbose Logging:**
```bash
ros2 launch gazebo_exam step1_practice.launch.py --ros-args --log-level debug
```

**Monitor Planning:**
```bash
ros2 topic echo /move_group/result
```

**Check Joint States:**
```bash
ros2 topic echo /joint_states
```

**Visualize Planning Scene:**
- In RViz, enable "PlanningScene" display
- Check collision objects

### Performance Optimization

1. **Reduce Planning Time**
   ```cpp
   move_group.setPlanningTime(5.0);  // Instead of 45s default
   ```

2. **Limit Planning Attempts**
   ```cpp
   move_group.setNumPlanningAttempts(5);  // Instead of 10
   ```

3. **Use Faster Planner**
   ```cpp
   move_group.setPlannerId("RRTConnectkConfigDefault");
   ```

## Summary

| Exercise | Difficulty | Key Concepts | Duration |
|----------|------------|--------------|----------|
| moveit2_practice_sample | Reference | MoveIt2 API, motion primitives | Study 2-3 hours |
| step1_practice | Beginner | Basic planning, pose goals | 5-10 hours |
| step2_hw | Advanced | Pick-place, planning scene, OOP | 15-20 hours |

**Learning Outcomes:**
- Understand MoveIt2 motion planning
- Implement manipulation tasks in C++
- Use Gazebo for physics validation
- Debug complex robotic systems
