# AMR Teleoperation Package

This package provides teleoperation capabilities for the AMR mobile robot using joystick and keyboard input.

## Overview

The `amr_teleop` package includes:

- **Joystick control** using standard game controllers (PS4, Xbox, etc.)
- **Keyboard control** for manual operation
- **Launch files** to start the complete teleoperation system
- **Configuration files** for customizing control parameters

## Prerequisites

Install the required ROS 2 packages:

```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-teleop-twist-keyboard
```

## Usage

### Launch Complete Teleop System

Launch both joystick and keyboard control:

```bash
ros2 launch amr_teleop teleop.launch.py
```

### Launch with Options

Launch only joystick control:

```bash
ros2 launch amr_teleop teleop.launch.py use_keyboard:=false
```

Launch only keyboard control:

```bash
ros2 launch amr_teleop teleop.launch.py use_joystick:=false
```

Change the cmd_vel output topic:

```bash
ros2 launch amr_teleop teleop.launch.py cmd_vel_topic:=/robot/cmd_vel
```

### Individual Nodes

Run joystick control only:

```bash
# Terminal 1: Start joy node
ros2 run joy joy_node

# Terminal 2: Start teleop_twist_joy
ros2 run teleop_twist_joy teleop_node --ros-args --params-file src/amr_teleop/params/teleop.yaml
```

Run keyboard control only:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Test Joystick

To verify your joystick is working and see button/axis mappings:

```bash
ros2 run amr_teleop test_joy
```

This will open a graphical window showing real-time joystick input.

## Joystick Control

### PS4 Controller Mapping

|         Control           |             Function              |
|---------------------------|-----------------------------------|
| **Left Stick Up/Down**    | Forward/Backward (linear.x)       |
| **Left Stick Left/Right** | Rotate Left/Right (angular.z)     |
| **L1 (Left Bumper)**      | Enable button (must hold to move) |
| **R1 (Right Bumper)**     | Turbo mode (faster speeds)        |

### Button Reference (PS4)

- **Button 0**: X (Cross)
- **Button 1**: O (Circle)
- **Button 2**: Triangle
- **Button 3**: Square
- **Button 4**: L1 (Enable control)
- **Button 5**: R1 (Turbo mode)
- **Button 6**: L2
- **Button 7**: R2
- **Button 8**: Share
- **Button 9**: Options
- **Button 10**: PS button
- **Button 11**: Left stick press
- **Button 12**: Right stick press

### Speed Settings

Normal mode (L1 only):

- Linear: 0.5 m/s
- Angular: 1.0 rad/s

Turbo mode (L1 + R1):

- Linear: 1.0 m/s
- Angular: 1.5 rad/s

## Keyboard Control

The keyboard teleoperation node runs in a separate terminal window (xterm).

Use the following keys:

```plain
Moving around:
   u    i    o
   j    k    l
   m    ,    .

u/o : forward + turn left/right
i   : forward
j/l : turn left/right  
k   : stop
m/. : backward + turn left/right
,   : backward

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
```

Press `Ctrl+C` to quit.

## Configuration

Edit `params/teleop.yaml` to customize:

- Joystick axis mappings
- Speed limits (normal and turbo)
- Enable/turbo button assignments
- Deadzone settings

Example customization:

```yaml
teleop_twist_joy_node:
  ros__parameters:
    scale_linear:
      x: 0.8          # Increase normal speed
    enable_button: 6  # Change to L2
```

## Troubleshooting

### Joystick not detected

1. Check if joystick device exists:

   ```bash
   ls -l /dev/input/js*
   ```

2. Test joystick with `jstest`:

   ```bash
   sudo apt install joystick
   jstest /dev/input/js0
   ```

3. Check ROS 2 joy messages:

   ```bash
   ros2 topic echo /joy
   ```

### No cmd_vel output

1. Verify you're holding the enable button (L1 by default)
2. Check if teleop_twist_joy is running:

   ```bash
   ros2 node list | grep teleop
   ```

3. Echo the cmd_vel topic:

   ```bash
   ros2 topic echo /cmd_vel
   ```

### Keyboard control not working

1. Make sure xterm is installed:

   ```bash
   sudo apt install xterm
   ```

2. The keyboard node requires focus on its terminal window

### Wrong direction of movement

Edit the axis values in `params/teleop.yaml` to invert directions. For example, to invert forward/backward:

- Change `axis_linear: x: 1` to `axis_linear: x: -1`

## Integration with Robot

To use teleoperation with your robot:

1. Launch your robot's base controller
2. Launch the teleop system
3. Ensure both publish/subscribe to the same `/cmd_vel` topic

Example launch sequence:

```bash
# Terminal 1: Robot base
ros2 launch amr_bringup bringup_real.launch.py

# Terminal 2: Teleoperation
ros2 launch amr_teleop teleop.launch.py
```

## Safety Notes

- Always have a clear area around the robot when testing
- Keep the enable button (L1) released when not actively controlling
- Start with normal mode before using turbo
- Test keyboard controls carefully as they don't have an enable button
- Consider lowering speed limits in `teleop.yaml` during initial testing

## Package Structure

```plain
amr_teleop/
├── amr_teleop/
│   ├── __init__.py
│   ├── teleop.py          # Custom teleop nodes (if any)
│   └── test_joy.py        # Joystick testing utility
├── launch/
│   └── teleop.launch.py   # Main launch file
├── params/
│   └── teleop.yaml        # Configuration parameters
├── package.xml
├── setup.py
└── README.md
```

## License

Apache 2.0

## Contributors

AMR Autonomy Team
