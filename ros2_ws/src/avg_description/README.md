# AGV Description Package

This package contains the URDF/Xacro description for a 4-wheeled warehouse Autonomous Guided Vehicle (AGV) robot compatible with ROS 2 Jazzy.

## Robot Specifications

### Physical Dimensions

- **Chassis**: 0.6m (L) × 0.45m (W) × 0.25m (H)
- **Wheels**: 0.05m radius, 0.02m width
- **Wheel Separation**: 0.4m (left-right)
- **Wheelbase**: 0.45m (front-rear)
- **Total Mass**: ~16kg

### Drive System

- **Type**: 4-wheel skid-steer
- **Wheels**: Continuous rotation joints
- 4 wheels: `wheel_front_left`, `wheel_front_right`, `wheel_rear_left`, `wheel_rear_right`

### Sensors

1. **LIDAR** (`lidar_link`)
   - Position: Top center of chassis
   - Type: 2D laser scanner
   - Range: 0.12m - 10m
   - Topic: `/scan`

2. **IMU** (`imu_link`)
   - Position: Geometric center of chassis
   - Update rate: 100 Hz
   - Topic: `/imu`

3. **Camera** (`camera_link`)
   - Position: Front-mounted
   - Resolution: 640×480
   - FOV: ~62°
   - Topic: `/camera/image_raw`

## Package Structure

```plain
avg_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── urdf/
│   ├── robot.xacro              # Main robot description
│   ├── wheel_macro.xacro        # Reusable wheel macro
│   ├── sensor_macro.xacro       # LIDAR, IMU, camera macros
│   └── materials.xacro          # Color definitions
├── meshes/
│   └── README.md                # Placeholder for 3D meshes
├── rviz/
│   └── display_config.rviz      # RViz visualization config
├── launch/
│   └── display.launch.py        # Launch file for RViz
├── config/                       # Configuration files
└── worlds/                       # Gazebo world files
```

## Usage

### 1. Build the Package

```bash
cd ~/avg_autonomy/ros2_ws
colcon build --packages-select avg_description
source install/setup.bash
```

### 2. Visualize in RViz

Launch the robot visualization with joint state publisher GUI:

```bash
ros2 launch avg_description display.launch.py
```

This will open:

- **RViz2**: 3D visualization
- **Joint State Publisher GUI**: Manual control of wheel joints

### 3. Launch Options

You can customize the launch with arguments:

```bash
# Use custom URDF file
ros2 launch avg_description display.launch.py model:=/path/to/custom.xacro

# Use custom RViz config
ros2 launch avg_description display.launch.py rvizconfig:=/path/to/config.rviz

# Without GUI (for headless operation)
ros2 launch avg_description display.launch.py use_gui:=false

# With simulation time (for Gazebo)
ros2 launch avg_description display.launch.py use_sim_time:=true
```

### 4. View Robot Description

To see the processed URDF:

```bash
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix avg_description)/share/avg_description/urdf/robot.xacro)"
```

### 5. Check TF Tree

View the transform tree:

```bash
ros2 run tf2_tools view_frames
```

## Gazebo Simulation

The robot includes Gazebo tags for simulation. To use in Gazebo Harmonic:

```bash
# Launch Gazebo (example - requires separate launch file)
ros2 launch avg_description gazebo.launch.py
```

**Note**: Gazebo launch files and control plugins will be added in future updates.

## Topics

### Published (when in simulation)

- `/scan` - sensor_msgs/LaserScan (LIDAR data)
- `/imu` - sensor_msgs/Imu (IMU data)
- `/camera/image_raw` - sensor_msgs/Image (Camera images)
- `/joint_states` - sensor_msgs/JointState (Wheel positions)
- `/tf` - TF transforms

### Subscribed (future control integration)

- `/cmd_vel` - geometry_msgs/Twist (Velocity commands)

## Frame Hierarchy

```plain
base_footprint
└── base_link
    ├── wheel_front_left
    ├── wheel_front_right
    ├── wheel_rear_left
    ├── wheel_rear_right
    ├── lidar_link
    ├── imu_link
    └── camera_link
        └── camera_link_optical
```

## Integration with Other Packages

This description package is designed to integrate with:

- **avg_slam**: SLAM using LIDAR data
- **avg_navigation**: Nav2 for autonomous navigation
- **avg_teleop**: Joystick teleoperation
- **avg_control**: Motor control and odometry

## Customization

### Modifying Dimensions

Edit properties in `urdf/robot.xacro`:

```xml
<xacro:property name="chassis_length" value="0.6"/>
<xacro:property name="chassis_width" value="0.45"/>
<!-- ... -->
```

### Adding New Sensors

Use macros in `urdf/sensor_macro.xacro` or create new ones:

```xml
<xacro:lidar_sensor parent="base_link">
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
</xacro:lidar_sensor>
```

### Using 3D Meshes

Place mesh files in `meshes/` and update visual/collision geometry:

```xml
<visual>
  <geometry>
    <mesh filename="package://avg_description/meshes/chassis.stl" scale="1 1 1"/>
  </geometry>
</visual>
```

## Dependencies

- `ament_cmake`
- `robot_state_publisher`
- `joint_state_publisher_gui`
- `xacro`
- `rviz2`
- `gazebo_ros` (for simulation)

## Troubleshooting

### Robot not visible in RViz

1. Check that `/robot_description` topic is published
2. Verify RobotModel display is enabled
3. Check Fixed Frame is set to `base_link` or `base_footprint`

### Xacro processing errors

```bash
# Check for syntax errors
xacro $(ros2 pkg prefix avg_description)/share/avg_description/urdf/robot.xacro
```

### Missing dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## License

Apache-2.0

## Author

Yossef Hady (<yossefhady53@gmail.com>)

## Version

0.0.0 (Initial Release)
