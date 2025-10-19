# AVG SLAM Package

## 📋 Overview

The `avg_slam` package provides **Simultaneous Localization and Mapping (SLAM)** capabilities for the AVG (Autonomous Guided Vehicle) mobile robot using `slam_toolbox`. This package enables the robot to create maps of unknown environments while simultaneously tracking its position within those maps.

## 🎯 Features

- **Online Async SLAM** using slam_toolbox for real-time mapping
- **RViz visualization** with custom SLAM view configuration
- **Gazebo Harmonic** simulation support
- **LIDAR-based mapping** using 2D laser scan data
- **Odometry fusion** for improved localization accuracy
- Integration with robot teleoperation for manual exploration

## 📦 Package Structure

```plaintext
avg_slam/
├── launch/
│   └── slam.launch.py          # Main SLAM launch file
├── params/
│   └── slam_params.yaml        # slam_toolbox configuration
├── rviz/
│   └── slam_view.rviz          # RViz configuration for SLAM
├── avg_slam/
│   └── __init__.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 🔧 Dependencies

- `slam_toolbox` - SLAM algorithm implementation
- `rviz2` - Visualization
- `sensor_msgs` - Laser scan messages
- `nav_msgs` - Odometry and map messages
- `tf2_ros` - Transform library
- `rclpy` - ROS 2 Python client library

## 🚀 Usage

### 1. Launch SLAM Only

If you already have simulation and teleoperation running:

```bash
ros2 launch avg_slam slam.launch.py
```

### 2. Launch Complete SLAM System (Recommended)

Launch everything together (simulation + teleop + SLAM):

```bash
# From workspace root
cd /home/yossef/avg_autonomy/ros2_ws
source install/setup.bash

# Launch complete SLAM bringup
ros2 launch avg_bringup slam_bringup.launch.py
```

This will start:

- ✅ Gazebo Harmonic simulation with warehouse world
- ✅ Robot description and state publisher
- ✅ Joystick teleoperation
- ✅ SLAM Toolbox for mapping
- ✅ RViz with SLAM visualization

### 3. Launch Arguments

#### slam.launch.py

```bash
# Disable RViz
ros2 launch avg_slam slam.launch.py use_rviz:=false

# Use custom parameters
ros2 launch avg_slam slam.launch.py slam_params_file:=/path/to/params.yaml

# Use custom RViz config
ros2 launch avg_slam slam.launch.py rviz_config:=/path/to/config.rviz

# Disable simulation time (for real robot)
ros2 launch avg_slam slam.launch.py use_sim_time:=false
```

#### slam_bringup.launch.py (Complete System)

```bash
# Disable RViz
ros2 launch avg_bringup slam_bringup.launch.py use_rviz:=false

# Disable joystick
ros2 launch avg_bringup slam_bringup.launch.py use_joystick:=false

# Spawn robot at different position
ros2 launch avg_bringup slam_bringup.launch.py x:=2.0 y:=3.0 yaw:=1.57

# Use different world
ros2 launch avg_bringup slam_bringup.launch.py world:=/path/to/world.sdf
```

## 🎮 Creating a Map

### Step-by-Step Instructions

1. **Launch the SLAM system:**

   ```bash
   ros2 launch avg_bringup slam_bringup.launch.py
   ```

2. **Verify topics:**

   ```bash
   # In a new terminal
   source install/setup.bash
   ros2 topic list
   
   # You should see:
   # /scan        - LIDAR data
   # /odom        - Odometry
   # /map         - Generated map
   # /cmd_vel     - Velocity commands
   ```

3. **Control the robot:**
   - Use joystick or keyboard to drive the robot
   - Drive slowly for better map quality
   - Ensure good overlap between scans
   - Cover the entire area you want to map

4. **Monitor in RViz:**
   - **LaserScan (white points):** Real-time LIDAR data
   - **Map (grid):** Building map in real-time
   - **RobotModel:** Your robot's position
   - **TF tree:** Frame relationships
   - **SLAM Graph:** Loop closure visualization

5. **Save the map:**

   ```bash
   # In a new terminal
   source install/setup.bash
   
   # Save map using slam_toolbox service
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/yossef/avg_navigation/maps/my_warehouse_map'}"
   
   # Alternative: Save as PNG/YAML using map_server
   ros2 run nav2_map_server map_saver_cli -f /home/yossef/avg_navigation/maps/my_map
   ```

## 📊 Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 2D LIDAR scan data |
| `/odom` | `nav_msgs/Odometry` | Robot odometry from wheel encoders/simulation |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/map` | `nav_msgs/OccupancyGrid` | Generated occupancy grid map |
| `/map_metadata` | `nav_msgs/MapMetaData` | Map metadata information |
| `/slam_toolbox/scan_visualization` | `sensor_msgs/LaserScan` | Processed scan for visualization |
| `/slam_toolbox/graph_visualization` | `visualization_msgs/MarkerArray` | SLAM graph with loop closures |

## 🔧 Configuration

### Key Parameters (slam_params.yaml)

```yaml
slam_toolbox:
  ros__parameters:
    # Frame configuration
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    
    # Sensor settings
    scan_topic: /scan
    
    # Map resolution (meters per pixel)
    resolution: 0.05
    
    # Maximum laser range (meters)
    max_laser_range: 12.0
    
    # Movement thresholds before updating map
    minimum_travel_distance: 0.2  # meters
    minimum_travel_heading: 0.2   # radians
    
    # Loop closure (helps correct accumulated drift)
    do_loop_closing: true
```

### Tuning Tips

**For better map quality:**

- Decrease `resolution` (e.g., 0.03) for finer details
- Decrease `minimum_travel_distance` for more frequent updates
- Drive slowly and smoothly

**For faster mapping:**

- Increase `minimum_travel_distance` (e.g., 0.3)
- Increase `throttle_scans` to process fewer scans

**For large environments:**

- Ensure `do_loop_closing: true` to reduce drift
- Increase `max_laser_range` if your LIDAR supports it

## 🐛 Troubleshooting

### No map appearing in RViz

```bash
# Check if SLAM is publishing
ros2 topic echo /map --once

# Check if LIDAR data is available
ros2 topic echo /scan --once

# Check TF frames
ros2 run tf2_ros tf2_echo map odom
```

### Map drifts or is inaccurate

- Ensure odometry is publishing correctly: `ros2 topic echo /odom`
- Verify TF tree is complete: `ros2 run tf2_tools view_frames`
- Drive more slowly
- Ensure good feature visibility (walls, obstacles)

### SLAM node crashes

```bash
# Check logs
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --log-level debug

# Verify parameters are loading
ros2 param list /slam_toolbox
```

### RViz doesn't start

```bash
# Check if rviz config exists
ls -la $(ros2 pkg prefix avg_slam)/share/avg_slam/rviz/

# Launch manually with default config
rviz2
```

## 📚 Additional Resources

- [slam_toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 SLAM Guide](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)
- [ROS 2 TF2 Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Tf2-Main.html)

## 🤝 Integration with Other Packages

### With Navigation (avg_navigation)

After creating a map, use it for autonomous navigation:

```bash
# 1. Save map
ros2 run nav2_map_server map_saver_cli -f my_map

# 2. Launch navigation with saved map
ros2 launch avg_navigation navigation.launch.py map:=my_map.yaml
```

### With Real Hardware (avg_comm_bridge)

For real robot deployment:

```bash
# Launch SLAM with real robot
ros2 launch avg_slam slam.launch.py use_sim_time:=false
```

## 📝 Notes

- **Simulation time:** Always use `use_sim_time:=true` in Gazebo simulation
- **Frame names:** Must match URDF: `base_link`, `odom`, `map`
- **Topic names:** LIDAR must publish to `/scan`, odometry to `/odom`
- **Map saving:** Save maps periodically during exploration to avoid data loss

## 🎓 Learning Points

This package demonstrates:

1. ✅ ROS 2 launch file composition with multiple includes
2. ✅ Parameter file management for complex nodes
3. ✅ RViz configuration for SLAM visualization
4. ✅ Integration of simulation, teleoperation, and SLAM
5. ✅ TF frame tree management (map → odom → base_link)
6. ✅ Sensor data processing pipeline

---

**Author:** AVG Team  
**ROS 2 Version:** Jazzy  
**Last Updated:** 2025-10-19
