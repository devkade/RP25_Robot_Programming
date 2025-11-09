# MoveIt2 Practice Sample Documentation

Welcome to the MoveIt2 practice sample documentation! This documentation will help you understand the codebase and implement the rectangle and circle drawing examples.

---

## Documentation Files

### 1. [CODEBASE_OVERVIEW.md](./CODEBASE_OVERVIEW.md)
**Comprehensive guide to the entire codebase**

This is your main reference document. It covers:
- Complete project structure and organization
- Detailed explanation of all 4 existing examples
- In-depth analysis of MoveIt2 APIs used
- Step-by-step implementation guides for rectangle and circle drawing
- Coordinate system and workspace information
- Error handling and debugging tips

**Start here** if you want a complete understanding of the project.

---

### 2. [QUICK_REFERENCE.md](./QUICK_REFERENCE.md)
**Fast lookup guide for common tasks**

Quick access to:
- Common commands (build, launch)
- Ready-to-use code snippets
- Common patterns for movement and planning
- File locations reference
- Implementation checklists
- Mathematical helpers (circle, rectangle generation)

**Use this** when you need quick code examples or can't remember syntax.

---

### 3. [API_REFERENCE.md](./API_REFERENCE.md)
**Complete API documentation**

Detailed reference for:
- MoveGroupInterface methods with full signatures
- All helper functions
- Message types and their fields
- Planning parameters and their effects
- Error codes and handling
- Best practices and common issues

**Refer to this** when you need detailed information about specific functions or parameters.

---

## Quick Start

### Understanding the Codebase
1. Read the "Project Overview" section in [CODEBASE_OVERVIEW.md](./CODEBASE_OVERVIEW.md)
2. Study the 4 existing examples:
   - Example 1: Move Sample (basic movement)
   - Example 2: Rotation Sample (orientation control)
   - Example 3: Gripper Sample (joint control)
   - Example 4: Waypoint Sample (Cartesian planning)

### Running Examples
```bash
# Build the project
cd ~/colcon_ws
colcon build --packages-select moveit2_exam
source install/setup.bash

# Run any example (cmd=1 to 6)
ros2 launch moveit2_exam moveit2_practice.launch.py cmd:=4 use_sim_time:=true
```

See [QUICK_REFERENCE.md](./QUICK_REFERENCE.md) for all launch commands.

### Implementing New Features

#### Rectangle Drawing (cmd=5)
1. Review the waypoint_sample in [CODEBASE_OVERVIEW.md](./CODEBASE_OVERVIEW.md#example-4-waypoint-sample-cmd4)
2. Check the implementation guide in [CODEBASE_OVERVIEW.md](./CODEBASE_OVERVIEW.md#for-rectangle-drawing-cmd5)
3. Use the rectangle pattern from [QUICK_REFERENCE.md](./QUICK_REFERENCE.md#mathematical-helpers)
4. Reference API details in [API_REFERENCE.md](./API_REFERENCE.md#computecartesianpath)

#### Circle Drawing (cmd=6)
1. Review the waypoint_sample in [CODEBASE_OVERVIEW.md](./CODEBASE_OVERVIEW.md#example-4-waypoint-sample-cmd4)
2. Check the implementation guide in [CODEBASE_OVERVIEW.md](./CODEBASE_OVERVIEW.md#for-circle-drawing-cmd6)
3. Use the circle generation code from [QUICK_REFERENCE.md](./QUICK_REFERENCE.md#circle-generation)
4. Reference API details in [API_REFERENCE.md](./API_REFERENCE.md#computecartesianpath)

---

## Key Concepts

### 1. Pose-based Planning
Used in move_sample and rotation_sample.
- Set a target pose (position + orientation)
- MoveIt2 finds a path in joint space
- No guarantee of straight-line paths in Cartesian space

### 2. Joint-based Planning
Used in gripper_sample.
- Directly control joint angles
- Fastest and most reliable
- Used for simple motions like gripper open/close

### 3. Cartesian Path Planning
Used in waypoint_sample (and needed for rectangle/circle).
- Guarantees straight-line paths between waypoints
- Essential for drawing tasks
- Returns a "fraction" indicating success (0.0 to 1.0)

---

## Learning Path

### Beginner
1. Read the Project Overview
2. Study Example 1 (move_sample)
3. Run Example 1 and observe the robot
4. Study the `list_to_pose()` and `go_to_pose_goal()` functions

### Intermediate
1. Study Example 4 (waypoint_sample) in detail
2. Understand `computeCartesianPath()` API
3. Review the Cartesian path section in API_REFERENCE.md
4. Try modifying waypoint_sample to draw a simple "L" shape

### Advanced
1. Implement rectangle_sample
2. Implement circle_sample
3. Experiment with different shapes (triangle, star, etc.)
4. Optimize parameters (eef_step, num_waypoints) for smooth motion

---

## Code Location Reference

| What | Where | Line |
|------|-------|------|
| Main source file | `src/moveit2_practice_sample.cpp` | - |
| Example 1: move_sample | moveit2_practice_sample.cpp | 40-70 |
| Example 2: rotation_sample | moveit2_practice_sample.cpp | 72-86 |
| Example 3: gripper_sample | moveit2_practice_sample.cpp | 88-106 |
| Example 4: waypoint_sample | moveit2_practice_sample.cpp | 108-138 |
| Example 5: rectangle_sample | moveit2_practice_sample.cpp | 140-148 |
| Example 6: circle_sample | moveit2_practice_sample.cpp | 150-158 |
| Helper: list_to_pose | moveit2_practice_sample.cpp | 27-38 |
| Helper: go_to_pose_goal | moveit2_practice_sample.cpp | 13-25 |
| Launch file | `launch/moveit2_practice.launch.py` | - |
| CMakeLists.txt | `CMakeLists.txt` | - |

---

## Common Tasks

### View Documentation for a Specific Topic

**Want to understand how to create a Pose?**
- Quick snippet: [QUICK_REFERENCE.md - Create a Pose](./QUICK_REFERENCE.md#1-create-a-pose)
- Detailed explanation: [CODEBASE_OVERVIEW.md - list_to_pose()](./CODEBASE_OVERVIEW.md#list_to_posex-y-z-roll-pitch-yaw)
- API details: [API_REFERENCE.md - list_to_pose()](./API_REFERENCE.md#list_to_pose)

**Want to understand Cartesian path planning?**
- Quick snippet: [QUICK_REFERENCE.md - Cartesian Path](./QUICK_REFERENCE.md#5-cartesian-path-planning)
- Detailed explanation: [CODEBASE_OVERVIEW.md - Waypoint Sample](./CODEBASE_OVERVIEW.md#example-4-waypoint-sample-cmd4)
- API details: [API_REFERENCE.md - computeCartesianPath()](./API_REFERENCE.md#computecartesianpath)

**Want to implement rectangle drawing?**
- Implementation guide: [CODEBASE_OVERVIEW.md - Rectangle Drawing](./CODEBASE_OVERVIEW.md#for-rectangle-drawing-cmd5)
- Code pattern: [QUICK_REFERENCE.md - Rectangle Generation](./QUICK_REFERENCE.md#rectangle-generation)
- Checklist: [QUICK_REFERENCE.md - Implementation Checklist](./QUICK_REFERENCE.md#for-rectangle-drawing)

**Want to implement circle drawing?**
- Implementation guide: [CODEBASE_OVERVIEW.md - Circle Drawing](./CODEBASE_OVERVIEW.md#for-circle-drawing-cmd6)
- Code pattern: [QUICK_REFERENCE.md - Circle Generation](./QUICK_REFERENCE.md#circle-generation)
- Checklist: [QUICK_REFERENCE.md - Implementation Checklist](./QUICK_REFERENCE.md#for-circle-drawing)

---

## Tips for Success

### Understanding Before Implementing
- Don't rush to implement rectangle and circle examples
- First, thoroughly understand waypoint_sample (Example 4)
- The rectangle and circle examples are just extensions of waypoint_sample with different waypoint generation

### Testing Strategy
1. Start with simple parameters (small rectangle, few circle points)
2. Verify the robot can reach all waypoints
3. Check the fraction value from computeCartesianPath()
4. Gradually increase complexity

### Debugging
- Always log the fraction value from computeCartesianPath()
- If fraction < 1.0, some waypoints may be unreachable
- Use RCLCPP_INFO to log waypoint positions
- Visualize in RViz before executing

### Safety
- Start with slow velocities (setMaxVelocityScalingFactor(0.1))
- Use simulation first (use_sim_time:=true)
- Keep robot workspace in mind (see CODEBASE_OVERVIEW.md)

---

## Getting Help

### If something doesn't work:
1. Check the error message in the terminal
2. Review "Common Issues and Solutions" in [API_REFERENCE.md](./API_REFERENCE.md#common-issues-and-solutions)
3. Verify your target poses are within workspace bounds
4. Check planning time is sufficient (45 seconds is recommended)

### If you don't understand a concept:
1. Read the relevant section in CODEBASE_OVERVIEW.md
2. Check the API details in API_REFERENCE.md
3. Look at the code examples in the existing implementations
4. Try the quick reference snippets in QUICK_REFERENCE.md

---

## Next Steps

After understanding the base codebase:

1. **Implement rectangle_sample**
   - Should take ~30-60 minutes if you understand waypoint_sample
   - Creates 4 waypoints in a rectangle pattern
   - Tests your understanding of Cartesian planning

2. **Implement circle_sample**
   - More challenging than rectangle
   - Requires parametric equation (cos/sin)
   - Tests your understanding of waypoint generation

3. **Experiment and extend**
   - Try other shapes (triangle, hexagon, star)
   - Combine shapes (draw multiple shapes)
   - Add pen-up/pen-down functionality (Z-axis movement)

---

## Summary

This documentation is organized to support different learning styles:

- **Top-down learners**: Start with CODEBASE_OVERVIEW.md
- **Bottom-up learners**: Start with examples in QUICK_REFERENCE.md
- **Reference seekers**: Jump directly to API_REFERENCE.md

All three documents cross-reference each other, so you can easily navigate between overview, examples, and detailed API information.

Good luck with your MoveIt2 learning journey!
