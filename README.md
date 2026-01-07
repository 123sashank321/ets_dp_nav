# ETS Dynamic Point Navigation (ets_dp_nav)

A ROS 2 Humble package for autonomous drone navigation using Nav2, enabling dynamic object following and waypoint navigation. Designed for PX4-based drones with complete integration between Nav2's planning capabilities and PX4's flight control.

## Overview

This package provides a complete navigation solution for drones that need to follow moving targets or navigate through dynamic environments. It bridges the gap between Nav2 (ground robot navigation) and PX4 (drone flight control), handling all necessary coordinate transformations and message conversions.

### Key Features

- **Dynamic Target Following**: Follow moving targets in 3D space with configurable trajectories
- **Nav2 Integration**: Leverages Nav2's robust path planning and obstacle avoidance
- **PX4 Bridge**: Seamless conversion between Nav2 commands and PX4 setpoints
- **Coordinate Frame Handling**: Automatic ENU ↔ NED conversion
- **Multiple Trajectory Types**: Support for circular, linear, and custom trajectories
- **3D Navigation**: Full 3D costmap and path planning support
- **Simulation Ready**: Works with Gazebo and PX4 SITL

## System Architecture

```
┌─────────────────┐
│ Target Generator│ Publishes moving target positions
└────────┬────────┘
         │ /goal_update
         ▼
┌─────────────────┐
│   Goal Relay    │ Relays goals to Nav2 action server
└────────┬────────┘
         │ NavigateToPose Action
         ▼
┌─────────────────┐
│   Nav2 Stack    │ Path planning & control
└────────┬────────┘
         │ /cmd_vel (ENU)
         ▼
┌─────────────────┐
│ Nav2→PX4 Bridge │ Converts ENU to NED, velocity to setpoints
└────────┬────────┘
         │ TrajectorySetpoint (NED)
         ▼
┌─────────────────┐
│   PX4 Autopilot │ Flight control
└────────┬────────┘
         │ VehicleOdometry (NED)
         ▼
┌─────────────────┐
│ PX4 TF Publisher│ Publishes TF transforms & odometry (ENU)
└─────────────────┘
```

## Package Components

### Nodes

#### 1. `target_generator`
Generates dynamic target positions for the drone to follow.

**Published Topics:**
- `/goal_update` (`geometry_msgs/PoseStamped`) - Target position updates

**Parameters:**
- `trajectory_type` (string, default: "line") - Type of trajectory: "circle" or "line"
- `radius` (double, default: 10.0) - Radius for circular trajectory [m]
- `height` (double, default: 10.0) - Target height [m]
- `frequency` (double, default: 10.0) - Update frequency [Hz]
- `angular_velocity` (double, default: 0.4) - Angular velocity for circle [rad/s]
- `linear_velocity` (double, default: 0.5) - Linear velocity for straight line [m/s]
- `line_distance` (double, default: 40.0) - Distance before looping [m]

#### 2. `goal_relay`
Relays dynamic goal updates to Nav2's action server, enabling continuous target following.

**Subscribed Topics:**
- `/goal_update` (`geometry_msgs/PoseStamped`) - Target position updates

**Action Clients:**
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) - Nav2 navigation action

**Parameters:**
- `update_distance_threshold` (double, default: 0.5) - Minimum distance change to trigger update [m]

#### 3. `nav2_to_px4_bridge`
Converts Nav2 velocity commands to PX4 trajectory setpoints with proper coordinate frame conversion.

**Subscribed Topics:**
- `/cmd_vel` (`geometry_msgs/Twist`) - Nav2 velocity commands (ENU frame)
- `/fmu/out/vehicle_odometry` (`px4_msgs/VehicleOdometry`) - PX4 odometry

**Published Topics:**
- `/fmu/in/offboard_control_mode` (`px4_msgs/OffboardControlMode`) - Offboard mode heartbeat
- `/fmu/in/trajectory_setpoint` (`px4_msgs/TrajectorySetpoint`) - Velocity setpoints (NED frame)

**Features:**
- 20Hz heartbeat for PX4 offboard mode
- 0.5s timeout with safe position holding
- ENU to NED coordinate conversion: `x_ned = y_enu`, `y_ned = x_enu`, `z_ned = -z_enu`

#### 4. `px4_tf_publisher`
Publishes TF transforms and odometry from PX4 to ROS 2 TF tree.

**Subscribed Topics:**
- `/fmu/out/vehicle_odometry` (`px4_msgs/VehicleOdometry`) - PX4 odometry (NED)

**Published Topics:**
- `/tf` - Dynamic transforms (odom → base_link)
- `/tf_static` - Static transforms (map → odom)
- `/odom` (`nav_msgs/Odometry`) - ROS 2 standard odometry (ENU)

**Coordinate Frames:**
- `map` - Global reference frame (ENU)
- `odom` - Odometry frame (ENU)
- `base_link` - Robot/drone body frame (ENU)

## Installation

### Prerequisites

- ROS 2 Humble
- Nav2 stack
- PX4 Autopilot with microXRCE-DDS agent
- px4_msgs package

### Build Instructions

```bash
# Navigate to your ROS 2 workspace
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/123sashank321/ets_dp_nav.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select ets_dp_nav

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Complete System

```bash
ros2 launch ets_dp_nav bringup_follow.launch.py
```

**Launch Arguments:**
- `use_sim_time` (default: true) - Use simulation clock
- `params_file` - Path to Nav2 parameters file
- `bt_xml_file` - Path to behavior tree XML file

### Running Individual Nodes

```bash
# Target generator
ros2 run ets_dp_nav target_generator --ros-args -p trajectory_type:=circle

# Goal relay
ros2 run ets_dp_nav goal_relay

# Nav2 to PX4 bridge
ros2 run ets_dp_nav nav2_to_px4_bridge

# PX4 TF publisher
ros2 run ets_dp_nav px4_tf_publisher
```

### Visualization with RViz

```bash
rviz2 -d $(ros2 pkg prefix ets_dp_nav)/share/ets_dp_nav/config/nav_debug.rviz
```

## Configuration

### Nav2 Parameters

The package includes optimized Nav2 parameters for aerial vehicles in `config/nav2_params.yaml`:

- **Controller**: MPPI controller with omnidirectional motion support
- **Costmaps**: 3D voxel layer with configurable height limits
- **Planner**: NavFn planner with unknown space allowance
- **Velocity Limits**: Configured for typical drone performance (5 m/s max)

### Behavior Tree

Custom behavior tree in `config/follow_point_bt.xml` optimized for dynamic object following.

## PX4 Setup

### 1. Start PX4 SITL (Simulation)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

### 2. Start MicroXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3. Enable Offboard Mode

In PX4 console or QGroundControl:
```bash
commander mode offboard
```

## Troubleshooting

### No odometry received
- Check if MicroXRCE-DDS agent is running
- Verify PX4 is publishing on `/fmu/out/vehicle_odometry`
- Check topic: `ros2 topic hz /fmu/out/vehicle_odometry`

### Nav2 not following target
- Ensure goal relay is connected to Nav2: `ros2 action list`
- Check if target is being published: `ros2 topic echo /goal_update`
- Verify TF tree is complete: `ros2 run tf2_tools view_frames`

### Drone not moving
- Verify offboard mode is enabled in PX4
- Check bridge is receiving cmd_vel: `ros2 topic echo /cmd_vel`
- Ensure trajectory setpoints are being published: `ros2 topic hz /fmu/in/trajectory_setpoint`

### Coordinate frame issues
- Verify TF transforms: `ros2 run tf2_ros tf2_echo map base_link`
- Check frame_id in messages matches expected frames
- Ensure `use_sim_time` parameter is consistent across all nodes

## Utility Scripts

### `analyze_bag.py`

Analyze recorded ROS 2 bag files for debugging:

```bash
# Record a bag
ros2 bag record /cmd_vel /goal_update /odom /tf

# Analyze it
python3 scripts/analyze_bag.py <bag_directory>
```

## Advanced Usage

### Custom Trajectory Generator

Create your own trajectory generator by publishing `geometry_msgs/PoseStamped` to `/goal_update`:

```python
import rclpy
from geometry_msgs.msg import PoseStamped

# ... node setup ...

msg = PoseStamped()
msg.header.frame_id = 'map'
msg.header.stamp = node.get_clock().now().to_msg()
msg.pose.position.x = target_x
msg.pose.position.y = target_y
msg.pose.position.z = target_z
# ... set orientation as quaternion ...

publisher.publish(msg)
```

### Tuning Nav2 for Your Drone

Key parameters to adjust in `config/nav2_params.yaml`:

- **Velocity limits**: `vx_max`, `vy_max`, `wz_max` in controller settings
- **Robot footprint**: `robot_radius` in costmap settings
- **Goal tolerance**: `xy_goal_tolerance`, `yaw_goal_tolerance` in goal checker
- **Obstacle height**: `max_obstacle_height` in voxel layer

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

Apache License 2.0

## Maintainer

- **ets** - [123sashanksasi321@gmail.com](mailto:123sashanksasi321@gmail.com)

## Acknowledgments

- Nav2 team for the robust navigation stack
- PX4 team for the flight control platform
- ROS 2 community for the excellent ecosystem