# AMR Autonomy Project Instructions

## Project Context
- **Framework**: ROS 2 Jazzy
- **Simulation**: Gazebo Harmonic
- **Build System**: `colcon`
- **Workspace Root**: `/home/yossef/amr_autonomy/ros2_ws`

## Build & Workflow
- **Build Command**: Always use `colcon build --symlink-install` from the `ros2_ws` directory.
- **Partial Build**: Use `--packages-select <pkg_name>` to rebuild specific packages quickly.
- **Sourcing**: Run `source install/setup.bash` after every build to refresh the environment.
- **Entry Point**: Use `ros2 launch amr_bringup slam_bringup.launch.py` to start the full system (Sim + SLAM + RViz + Teleop).

## Architecture & Patterns

### Launch System
- **Orchestration**: `amr_bringup` is the master package. `slam_bringup.launch.py` coordinates `amr_description` (Sim), `amr_slam` (Algo), and `amr_teleop` (Control).
- **Parameter Injection**: Use `GroupAction` combined with `SetParameter` to override parameters in included launch files (e.g., injecting `map_file_name` into `slam_toolbox`).
- **Argument Passing**: When using `IncludeLaunchDescription`, pass arguments using `LaunchConfiguration('arg_name')`. **NEVER** pass the `DeclareLaunchArgument` object itself in the `launch_arguments` dictionary.

### SLAM & Map Management
- **Dual Map Strategy**:
  - **Serialized Maps** (`.posegraph`, `.data`): Saved to `src/amr_slam/maps/` for "Lifelong SLAM" (resuming sessions).
  - **Standard Maps** (`.pgm`, `.yaml`): Saved to `src/amr_navigation/maps/` for AMCL/Nav2.
- **Map Saver**: Use `ros2 run amr_slam save_map.py <world_name>` to save both formats simultaneously.
- **Modes**:
  - `mapping`: Fresh start using `online_async_launch.py`.
  - `continue_mapping`: Loads serialized map to extend it.
  - `localization`: Loads serialized map for localization only.

### Robot Description (URDF)
- **Drive Modes**: `robot.xacro` supports `diff` and `mecanum` via the `drive_mode` argument.
- **Odometry Conflict**: When using `drive_mode:='mecanum'`, the `MecanumDrive` plugin publishes odometry. **Disable** any separate `OdometryPublisher` plugin to prevent TF drift/jumping caused by conflicting transforms.

### Python Scripts
- **Path Detection**: Scripts (like `save_map.py`) must robustly detect the workspace root (`ros2_ws`) to locate `src/` directories, as they may run from `install/` or arbitrary locations. Use `ament_index` or path traversal to find the source.
