# GitHub Copilot Instructions for AMR Autonomy

This repository contains the source code for an Autonomous Mobile Robot (AMR) system built on ROS 2. The project is structured as a standard colcon workspace.

## 🏗 Architecture & Structure

The codebase is organized into a ROS 2 workspace (`ros2_ws`) with modular packages prefixed with `amr_`:

- **`amr_bringup`**: System-wide launch files and configurations. This is the entry point for starting the robot.
- **`amr_navigation`**: Navigation stack integration (Nav2), path planning, and custom control logic (e.g., `teleop_override.py`).
- **`amr_description`**: Robot model definitions (URDF/Xacro) and meshes.
    - **Robot Type**: Mecanum drive (omnidirectional) mobile robot.
    - **Sensors**: LIDAR (top-mounted), IMU (internal), and Front Camera.
    - **Simulation**: Configured for Gazebo Harmonic (`gz-sim`) with `gz-sim-mecanum-drive-system` plugin.
- **`amr_slam`**: SLAM (Simultaneous Localization and Mapping) configurations and nodes.
- **`amr_teleop`**: Teleoperation tools and scripts.
- **`amr_comm_bridge`**: Communication interfaces for external systems (currently a skeleton).
- **`amr_msgs`**: Custom ROS 2 message and service definitions (currently empty).

**Note**: The `resources/` directory contains reference materials and course content. The active development happens in `ros2_ws/`.

## 🛠 Developer Workflow

### Build & Environment
- **Root Directory**: Always run build commands from `ros2_ws/`.
- **Build Command**: `colcon build --symlink-install` (symlink-install is recommended for Python development).
- **Source Environment**: `source install/setup.bash` after every build.

### Running Nodes
- Use `ros2 launch` for starting complex subsystems.
- Example: `ros2 launch amr_bringup bringup_real.launch.py` (once implemented).

## 📝 Coding Conventions

### Python Nodes (ROS 2)
- **Structure**: Define nodes as classes inheriting from `rclpy.node.Node`.
- **Entry Point**: Use a `main()` function with `rclpy.init()` and `rclpy.spin()`.
- **Type Hinting**: Use Python type hints for function arguments and return values.
- **Logging**: Use `self.get_logger().info/warn/error()` instead of `print()`.
- **Parameters**: Declare parameters in `__init__` using `self.declare_parameter()`.

**Example Pattern (`amr_navigation/amr_navigation/teleop_override.py`)**:
```python
class TeleopOverrideNode(Node):
    def __init__(self):
        super().__init__('teleop_override_node')
        self.declare_parameter('teleop_timeout', 0.5)
        # ... setup subscribers/publishers ...

    def timer_callback(self):
        # ... control logic ...
```

### Launch Files
- Use Python-based launch files (`.launch.py`).
- Return a `LaunchDescription` containing `Node` actions.

## 🧩 Integration & Dependencies
- **Nav2**: The navigation stack is a key dependency. Ensure compatibility with the installed ROS 2 distribution.
- **Gazebo Harmonic**: The simulation environment uses `gz-sim`. Ensure plugins like `gz-sim-mecanum-drive-system` are correctly configured in URDF.
- **URDF/Xacro**: Robot descriptions are in `amr_description/urdf`. Use macros for reusable components (e.g., `wheel_macro.xacro`).
- **Control Topics**:
    - `cmd_vel`: Main velocity command topic.
    - `odom`: Odometry topic published by the simulation plugin.
    - `tf`: Transform tree (base_footprint -> base_link -> sensors).
