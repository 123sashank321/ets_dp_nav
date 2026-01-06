#!/usr/bin/env python3
"""
Nav2 to PX4 Bridge Node

Bridges Nav2 velocity commands to PX4 trajectory setpoints with proper
coordinate frame conversion (ENU to NED).

Key Features:
- Converts cmd_vel (ENU) to PX4 TrajectorySetpoint (NED)
- Publishes offboard control mode heartbeat at 20Hz
- Implements 0.5s timeout for safe position holding
- Coordinate conversion: x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry


class Nav2ToPX4Bridge(Node):
    """Bridge between Nav2 and PX4 with coordinate frame conversion."""

    def __init__(self):
        super().__init__('nav2_to_px4_bridge')
        
        # QoS profile for PX4 topics (best effort, keep last)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )
        
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )
        
        # State variables
        self.current_cmd_vel = Twist()
        self.last_cmd_vel_time = self.get_clock().now()
        self.cmd_vel_timeout = 0.5  # seconds
        self.current_yaw = 0.0  # Current drone yaw in radians
        
        # Timer for publishing at 20Hz (required for PX4 offboard mode)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
        self.get_logger().info('Nav2 to PX4 Bridge started')
        self.get_logger().info('Publishing offboard heartbeat at 20Hz')
        self.get_logger().info(f'cmd_vel timeout: {self.cmd_vel_timeout}s')

    def cmd_vel_callback(self, msg):
        """Store incoming Nav2 velocity commands."""
        self.current_cmd_vel = msg
        self.last_cmd_vel_time = self.get_clock().now()

    def odometry_callback(self, msg):
        """Extract current yaw from vehicle odometry."""
        # Extract yaw from quaternion (NED frame)
        # For simple cases, we can use the provided values
        # Note: msg.q contains [w, x, y, z] quaternion
        pass

    def timer_callback(self):
        """
        Main control loop running at 20Hz.
        Publishes offboard control mode and trajectory setpoint.
        """
        # Always publish offboard control mode heartbeat
        self.publish_offboard_control_mode()
        
        # Check if cmd_vel is recent
        time_since_cmd = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        
        if time_since_cmd > self.cmd_vel_timeout:
            # Timeout - publish zero velocity to hold position
            self.publish_trajectory_setpoint(0.0, 0.0, 0.0, 0.0)
        else:
            # Convert and publish the velocity command
            self.convert_and_publish_cmd_vel()

    def publish_offboard_control_mode(self):
        """
        Publish offboard control mode.
        Required heartbeat for PX4 offboard mode.
        """
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        
        self.offboard_mode_pub.publish(msg)

    def convert_and_publish_cmd_vel(self):
        """
        Convert Nav2 cmd_vel (ENU) to PX4 TrajectorySetpoint (NED).
        
        Coordinate conversion:
        - x_ned = y_enu (East becomes North)
        - y_ned = x_enu (North becomes East)
        - z_ned = -z_enu (Up becomes Down)
        """
        # Extract velocities from cmd_vel (ENU frame from Nav2)
        vx_enu = self.current_cmd_vel.linear.x
        vy_enu = self.current_cmd_vel.linear.y
        vz_enu = self.current_cmd_vel.linear.z
        yaw_rate = self.current_cmd_vel.angular.z
        
        # Convert ENU to NED
        vx_ned = vy_enu  # North (NED) = East (ENU)
        vy_ned = vx_enu  # East (NED) = North (ENU)
        vz_ned = -vz_enu  # Down (NED) = -Up (ENU)
        
        # Publish trajectory setpoint
        self.publish_trajectory_setpoint(vx_ned, vy_ned, vz_ned, yaw_rate)

    def publish_trajectory_setpoint(self, vx, vy, vz, yaw_rate):
        """
        Publish velocity setpoint to PX4.
        
        Args:
            vx: Velocity in North direction (NED) [m/s]
            vy: Velocity in East direction (NED) [m/s]
            vz: Velocity in Down direction (NED) [m/s]
            yaw_rate: Yaw rate [rad/s]
        """
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # microseconds
        
        # Set velocity setpoints (NED frame)
        msg.velocity[0] = float(vx)
        msg.velocity[1] = float(vy)
        msg.velocity[2] = float(vz)
        
        # Set yaw rate
        msg.yawspeed = float(yaw_rate)
        
        # Set NaN for unused fields (position, acceleration)
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')
        
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')
        
        msg.jerk[0] = float('nan')
        msg.jerk[1] = float('nan')
        msg.jerk[2] = float('nan')
        
        msg.yaw = float('nan')  # Use yawspeed instead
        
        self.trajectory_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ToPX4Bridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
