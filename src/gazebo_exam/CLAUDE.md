# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 package (`gazebo_exam`) for robotics manipulation using the Franka FR3 robot arm with MoveIt 2. The package provides both simulation (Gazebo) and visualization (RViz) capabilities for motion planning and control exercises.

## Build and Run Commands

### Building
```bash
# From colcon workspace root (/home/kangdongkyu/colcon_ws)
colcon build --packages-select gazebo_exam
source install/setup.bash
```

### Running Launch Files

The package provides paired launch files for most exercises - one for visualization-only and one for Gazebo simulation:

**Display/Visualization (RViz only):**
```bash
ros2 launch gazebo_exam display_franka.launch.py
```

**Basic MoveIt Practice:**
```bash
# Without Gazebo
ros2 launch gazebo_exam moveit2_practice.launch.py

# With Gazebo simulation
ros2 launch gazebo_exam moveit2_franka.launch.py
```

**Step 1 Practice (Block manipulation):**
```bash
# Without Gazebo
ros2 launch gazebo_exam step1_practice.launch.py

# With Gazebo simulation
ros2 launch gazebo_exam step1_practice_gazebo.launch.py
```

**Step 2 Homework:**
```bash
# Without Gazebo
ros2 launch gazebo_exam step2_hw.launch.py

# With Gazebo simulation
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
```

### Running Executables Directly
```bash
ros2 run gazebo_exam moveit2_practice_sample
ros2 run gazebo_exam step1_practice
ros2 run gazebo_exam step2_hw
```

## Architecture

### Robot Configuration

**Robot Model:** Franka Emika FR3 (7-DOF arm)
**End Effector:** Franka Hand gripper (`franka_hand`)

The robot model is defined using URDF/xacro files with modular configuration:
- **Main URDF:** `urdf/robots/fr3/fr3.urdf.xacro` (includes macro)
- **Robot Macro:** `urdf/robots/fr3/fr3_macro.xacro` (actual robot definition)
- **End Effector:** `urdf/end_effectors/franka_hand/`
- **Config Files:** `urdf/robots/fr3/config/` contains:
  - `joint_limits.yaml` - Joint position/velocity/acceleration limits
  - `kinematics.yaml` - DH parameters and kinematic chain
  - `inertials.yaml` - Link masses and inertia tensors
  - `visual_parameters.yaml` - Visual appearance settings
  - `initial_positions.yaml` - Default joint positions

### MoveIt 2 Configuration

MoveIt configuration files are in `moveit_config/`:
- `kinematics.yaml` - Kinematics solver configuration (KDL)
- `joint_limits_6dof_gripper.yaml` - Planning-specific joint limits
- `ompl_planning.yaml` - OMPL planner configurations (RRTConnect, etc.)
- `controllers_6dof_gripper.yaml` - Controller definitions for execution

SRDF files in `srdf/` define:
- Planning groups (e.g., "fr3_manipulator")
- End effector definitions
- Self-collision disable matrices
- Pre-defined poses

### Controller Configuration

ROS 2 control configurations in `controller/`:
- `fr3_controller.yaml` - Joint trajectory controller for arm-only
- `fr3_controller_gripper.yaml` - Controllers for arm + gripper

### Source Code Architecture

The C++ executables follow a common pattern:

1. **Utility Functions:**
   - `list_to_pose(x, y, z, roll, pitch, yaw)` - Converts position/orientation to `geometry_msgs::msg::Pose`
   - `go_to_pose_goal(move_group_interface, target_pose)` - Plans and executes motion to target pose

2. **Data Structures (step1_practice.cpp):**
   - `Location` class - Represents 2D coordinates (x, y)
   - `Block` class - Represents block entities with dimensions and location

3. **Main Pattern:**
   - Initialize ROS 2 node
   - Create `MoveGroupInterface` for planning group
   - Set planning parameters (workspace, planning time, etc.)
   - Execute motion sequences using pose goals

### Launch File Architecture

Launch files use a common structure:
- Robot description loaded via xacro with `sim_gazebo` parameter
- SRDF loaded for MoveIt semantic description
- MoveIt nodes: `move_group`, planning scene, and trajectory execution
- RViz configuration with MoveIt motion planning plugin
- Gazebo variants include Gazebo server/client and controller spawning

Key parameters across launch files:
- `arm_id`: "fr3"
- `ee_id`: "franka_hand"
- `use_sim_time`: true/false depending on simulation
- `launch_rviz_moveit`: true/false to control RViz startup

## Development Notes

### Simulation vs Real Robot

The URDF xacro accepts `sim_gazebo` argument:
- `sim_gazebo:=true` - Enables Gazebo plugins and ros2_control hardware interface
- `sim_gazebo:=false` (default) - Uses fake hardware for visualization-only mode

### Working with MoveIt 2

When writing motion planning code:
- Always check planning success before execution
- Use `setPoseTarget()` for Cartesian goals
- Use `setJointValueTarget()` for joint space goals
- The planning group name is typically "fr3_manipulator"
- End effector link is "fr3_hand_tcp" or similar

### File Organization

- `src/` - C++ executables (student exercises and samples)
- `launch/` - Launch files (pairs of non-Gazebo and Gazebo variants)
- `urdf/` - Robot models (modular structure with robots/ and end_effectors/)
- `meshes/` - 3D mesh files for visualization
- `models/` - Gazebo world models
- `rviz/` - RViz configuration files
- `moveit_config/` - MoveIt-specific configurations
- `controller/` - ROS 2 control configurations

### Common Patterns

**Pose Goal Planning:**
```cpp
auto target_pose = list_to_pose(x, y, z, roll, pitch, yaw);
go_to_pose_goal(move_group_interface, target_pose);
```

**Planning Group Setup:**
```cpp
moveit::planning_interface::MoveGroupInterface move_group_interface(node, "fr3_manipulator");
move_group_interface.setPlanningTime(10.0);
move_group_interface.setNumPlanningAttempts(10);
```

### Dependencies

Key ROS 2 dependencies:
- `rclcpp` - ROS 2 C++ client library
- `moveit_ros_planning_interface` - MoveIt 2 planning interface
- `tf2_geometry_msgs` - TF2 geometry message utilities
- `urdf`, `xacro` - Robot model processing
- `rviz2` - Visualization
- Gazebo (for simulation variants)
