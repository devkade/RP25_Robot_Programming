# Build and Run Guide

This guide covers how to build and run the gazebo_exam package.

## Prerequisites

- ROS 2 (Humble or later)
- MoveIt2
- Gazebo (11 or later)
- ros2_control

## Build Instructions

### Building the Package

From your colcon workspace root:

```bash
cd /home/kangdongkyu/colcon_ws
colcon build --packages-select gazebo_exam
source install/setup.bash
```

### Build Targets

The package builds three C++ executables:

1. **moveit2_practice_sample** - Reference implementation with 6 demonstrations
2. **step1_practice** - Student practice template
3. **step2_hw** - Advanced pick-and-place homework

### Clean Build

To perform a clean rebuild:

```bash
rm -rf build/gazebo_exam install/gazebo_exam
colcon build --packages-select gazebo_exam
```

## Launch Files Overview

The package provides 8 launch files organized into logical groups:

### Visualization Only

**display_franka.launch.py**
- Static URDF visualization in RViz
- Interactive joint sliders (no simulation)
- Use for exploring robot structure

```bash
ros2 launch gazebo_exam display_franka.launch.py
```

### Gazebo Simulation (No MoveIt2)

**gazebo_franka.launch.py**
- Full physics simulation with controllers
- Optional RViz visualization
- Direct controller access (no motion planning)

```bash
ros2 launch gazebo_exam gazebo_franka.launch.py
ros2 launch gazebo_exam gazebo_franka.launch.py launch_rviz:=true
```

### MoveIt2 + Gazebo Integration

**moveit2_franka.launch.py**
- Complete simulation with motion planning
- Interactive planning in RViz
- No automated execution

```bash
ros2 launch gazebo_exam moveit2_franka.launch.py
```

### Practice Exercises

**moveit2_practice.launch.py**
- Runs reference implementation (moveit2_practice_sample)
- MoveIt2 only (no Gazebo physics)
- Supports --cmd parameter for different demonstrations

```bash
# Run different demonstration scenarios
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=1  # move sample
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=2  # rotation
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=3  # gripper
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=4  # waypoints
```

### Step 1 Practice

**step1_practice.launch.py**
- Runs step1_practice executable
- MoveIt2 planning only (no physics)

```bash
ros2 launch gazebo_exam step1_practice.launch.py
```

**step1_practice_gazebo.launch.py**
- Full Gazebo simulation with 2 objects (red and blue boxes)
- Physics-based testing

```bash
ros2 launch gazebo_exam step1_practice_gazebo.launch.py
```

### Step 2 Homework

**step2_hw.launch.py**
- Runs step2_hw executable
- MoveIt2 planning only

```bash
ros2 launch gazebo_exam step2_hw.launch.py
```

**step2_hw_gazebo.launch.py**
- Full simulation with 8 manipulation objects
- Complex pick-and-place environment

```bash
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
```

## Running Executables Directly

You can also run the compiled executables directly:

```bash
ros2 run gazebo_exam moveit2_practice_sample
ros2 run gazebo_exam step1_practice
ros2 run gazebo_exam step2_hw
```

Note: You must have the necessary nodes running (move_group, robot_state_publisher, etc.) for the executables to work properly.

## Common Workflows

### Development Workflow

1. **Edit Source Code**
   ```bash
   # Edit src/step1_practice.cpp or src/step2_hw.cpp
   ```

2. **Build**
   ```bash
   colcon build --packages-select gazebo_exam
   source install/setup.bash
   ```

3. **Test Without Physics** (faster iteration)
   ```bash
   ros2 launch gazebo_exam step1_practice.launch.py
   ```

4. **Test With Physics** (final validation)
   ```bash
   ros2 launch gazebo_exam step1_practice_gazebo.launch.py
   ```

### Debugging Workflow

1. **Check Robot Visualization**
   ```bash
   ros2 launch gazebo_exam display_franka.launch.py
   ```

2. **Test Controllers**
   ```bash
   ros2 launch gazebo_exam gazebo_franka.launch.py launch_rviz:=true
   ```

3. **Interactive Motion Planning**
   ```bash
   ros2 launch gazebo_exam moveit2_franka.launch.py
   # Use RViz MotionPlanning plugin to test reachability
   ```

4. **Run Your Code**
   ```bash
   ros2 launch gazebo_exam step1_practice_gazebo.launch.py
   ```

### Learning Progression

**Week 1: Understand the Robot**
```bash
ros2 launch gazebo_exam display_franka.launch.py
# Explore joints, links, end-effector
```

**Week 2: Physics Simulation**
```bash
ros2 launch gazebo_exam gazebo_franka.launch.py launch_rviz:=true
# Watch robot respond to controller commands
```

**Week 3: Motion Planning**
```bash
ros2 launch gazebo_exam moveit2_franka.launch.py
# Plan trajectories interactively in RViz
```

**Week 4: Reference Examples**
```bash
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=1
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=4
# Study different motion patterns
```

**Week 5: Practice Assignment**
```bash
ros2 launch gazebo_exam step1_practice_gazebo.launch.py
# Implement and test your solution
```

**Week 6-7: Advanced Assignment**
```bash
ros2 launch gazebo_exam step2_hw_gazebo.launch.py
# Complete pick-and-place task
```

## Troubleshooting

### Build Errors

**Missing dependencies:**
```bash
rosdep install --from-paths src --ignore-src -r -y
```

**CMake errors:**
```bash
# Check that all dependencies are installed
apt search ros-humble-moveit
apt search ros-humble-gazebo-ros2-control
```

### Runtime Errors

**"No planning group found":**
- Check that move_group node is running
- Verify SRDF is loaded correctly

**"Controller not available":**
- Ensure controllers are spawned (check with `ros2 control list_controllers`)
- Wait for Gazebo to fully load before execution

**"IK solver failed":**
- Target pose may be unreachable
- Check joint limits in URDF
- Try different orientation or position

**Gazebo crashes:**
- Reduce simulation complexity
- Check for URDF errors with `check_urdf`

## Advanced Usage

### Custom Launch Parameters

Most launch files support parameters:

```bash
# Launch with custom RViz
ros2 launch gazebo_exam moveit2_franka.launch.py \
    launch_rviz:=true \
    use_sim_time:=true

# Launch practice with specific command
ros2 launch gazebo_exam moveit2_practice.launch.py cmd:=4
```

### Monitoring Topics

```bash
# Watch joint states
ros2 topic echo /joint_states

# Monitor move_group status
ros2 topic echo /move_group/status

# View trajectory execution
ros2 topic echo /manipulator_controller/follow_joint_trajectory/status
```

### Controller Commands

```bash
# List active controllers
ros2 control list_controllers

# View controller info
ros2 control list_hardware_interfaces
```

## Performance Tips

1. **Use non-Gazebo launches for development** - Much faster iteration
2. **Reduce planning time** - Default is 45s, can be reduced for simple tasks
3. **Disable visualization** - Set `launch_rviz:=false` when not needed
4. **Use simpler planners** - RRTConnect is fast for point-to-point planning
