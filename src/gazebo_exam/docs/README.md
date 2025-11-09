# Gazebo Exam Documentation

This directory contains comprehensive documentation for the `gazebo_exam` ROS 2 package.

## Package Overview

**gazebo_exam** is an educational ROS 2 robotics package for learning motion planning and manipulation with the Franka Emika FR3 robot (7-DOF) and Franka Hand gripper. The package integrates MoveIt2 for motion planning and Gazebo for physics simulation.

## Documentation Index

### Quick Start
- **[BUILD_AND_RUN.md](BUILD_AND_RUN.md)** - Build instructions, launch commands, and common workflows

### Architecture & Design
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - System architecture, component overview, and integration points
- **[ROBOT_MODEL.md](ROBOT_MODEL.md)** - URDF/SRDF structure, kinematics, joint specifications
- **[MOVEIT_CONFIG.md](MOVEIT_CONFIG.md)** - MoveIt2 configuration, planners, controllers

### Development
- **[LAUNCH_FILES.md](LAUNCH_FILES.md)** - Detailed launch file documentation and use cases
- **[EXERCISES.md](EXERCISES.md)** - Practice exercises, homework assignments, learning progression

### Reference
- **[COMPREHENSIVE.md](COMPREHENSIVE.md)** - Complete technical reference (38KB, all details)

## Package Statistics

- **Total Files:** 71
- **C++ Source Files:** 3 (moveit2_practice_sample, step1_practice, step2_hw)
- **Python Launch Files:** 8
- **Xacro Configuration Files:** 16 (URDF/SRDF)
- **YAML Configuration Files:** 14
- **Mesh Files:** 25 (DAE visual + STL collision)

## Key Technologies

- **ROS 2** - Robot Operating System (Humble or later)
- **MoveIt2** - Motion planning framework
- **Gazebo** - Physics simulation
- **ros2_control** - Unified control interface
- **OMPL** - Open Motion Planning Library
- **KDL** - Kinematics and Dynamics Library

## Quick Commands

### Build
```bash
cd /home/kangdongkyu/colcon_ws
colcon build --packages-select gazebo_exam
source install/setup.bash
```

### Launch Examples
```bash
# Visualization only
ros2 launch gazebo_exam display_franka.launch.py

# Gazebo simulation + MoveIt2
ros2 launch gazebo_exam moveit2_franka.launch.py

# Step 1 practice with simulation
ros2 launch gazebo_exam step1_practice_gazebo.launch.py

# Step 2 homework with 8 objects
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
```

## Learning Path

1. **Week 1-2:** Visualization and Gazebo basics
2. **Week 3-4:** MoveIt2 motion planning and reference examples
3. **Week 5:** Step 1 practice assignment
4. **Week 6-7:** Step 2 advanced pick-and-place assignment

## Support

For detailed information on any topic, refer to the specific documentation files listed above.
