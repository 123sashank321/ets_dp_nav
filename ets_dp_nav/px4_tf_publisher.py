#!/usr/bin/env python3
"""
PX4 TF Publisher Node

Publishes TF transforms from PX4 VehicleOdometry to ROS 2 TF tree.
Required for Nav2 to work with PX4 drone.

Publishes:
- map -> odom (static, initially identity)
- odom -> base_link (from PX4 odometry in ENU frame)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

class PX4TFPublisher(Node):
    """Publishes TF transforms and Odometry from PX4 for Nav2 integration."""

    def __init__(self):
        super().__init__('px4_tf_publisher')
        
        # QoS profile for PX4 topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # TF broadcasters
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Odometry publisher (Standard ROS 2 message)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Track if we're receiving odometry
        self.odom_received = False
        self.last_odom_time = self.get_clock().now()
        
        # Subscribe to PX4 odometry
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )
        
        # Publish static map -> odom transform
        self.publish_static_transforms()
        
        # Timer to republish static transform periodically (in case it gets lost)
        self.static_timer = self.create_timer(1.0, self.publish_static_transforms)
        
        # Status timer to log odometry reception
        self.status_timer = self.create_timer(5.0, self.log_status)
        
        self.get_logger().info('PX4 TF Publisher started')
        self.get_logger().info('Publishing: map->odom (static), odom->base_link (TF), /odom (Odometry)')
        self.get_logger().info('Waiting for PX4 odometry on /fmu/out/vehicle_odometry...')

    def log_status(self):
        """Log status of odometry reception."""
        if not self.odom_received:
            self.get_logger().warn(
                'No odometry received yet. Is PX4 running and MicroXRCE agent connected?',
                throttle_duration_sec=10.0
            )
        else:
            time_since_odom = (self.get_clock().now() - self.last_odom_time).nanoseconds / 1e9
            if time_since_odom > 2.0:
                self.get_logger().warn(
                    f'No odometry received for {time_since_odom:.1f}s. Connection lost?',
                    throttle_duration_sec=5.0
                )

    def publish_static_transforms(self):
        """Publish static transform from map to odom frame."""
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = 'odom'
        
        # Identity transform (map and odom aligned)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(static_transform)

    def odometry_callback(self, msg):
        """
        Convert PX4 VehicleOdometry to:
        1. TF transform (odom -> base_link)
        2. ROS 2 Odometry message (/odom)
        
        Note: PX4 odometry is in NED frame, we convert to ENU for ROS 2.
        """
        # Track odometry reception
        if not self.odom_received:
            self.get_logger().info('First odometry message received! Publishing TF and /odom.')
            self.odom_received = True
        
        self.last_odom_time = self.get_clock().now()
        current_time = self.get_clock().now().to_msg()
        
        # 1. Broadcast TF Transform
        t = TransformStamped()
        t.header.stamp = current_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # ENU Conversion
        t.transform.translation.x = float(msg.position[1])   # East
        t.transform.translation.y = float(msg.position[0])   # North
        t.transform.translation.z = float(-msg.position[2])  # Up
        
        q_ned = msg.q
        t.transform.rotation.w = float(q_ned[0])
        t.transform.rotation.x = float(q_ned[2])
        t.transform.rotation.y = float(q_ned[1])
        t.transform.rotation.z = float(-q_ned[3])
        
        self.tf_broadcaster.sendTransform(t)
        
        # 2. Publish Standard ROS Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position (Same as TF)
        odom_msg.pose.pose.position.x = t.transform.translation.x
        odom_msg.pose.pose.position.y = t.transform.translation.y
        odom_msg.pose.pose.position.z = t.transform.translation.z
        odom_msg.pose.pose.orientation = t.transform.rotation
        
        # Velocity (Convert NED to ENU)
        # PX4 velocity is in NED and usually Body frame, but VehicleOdometry.velocity can be local frame.
        # Assuming msg.velocity is in NED:
        # vx_enu = vy_ned, vy_enu = vx_ned, vz_enu = -vz_ned
        odom_msg.twist.twist.linear.x = float(msg.velocity[1]) # East
        odom_msg.twist.twist.linear.y = float(msg.velocity[0]) # North
        odom_msg.twist.twist.linear.z = float(-msg.velocity[2]) # Up
        
        # Angular rate (NED to ENU)
        odom_msg.twist.twist.angular.x = float(msg.angular_velocity[1])
        odom_msg.twist.twist.angular.y = float(msg.angular_velocity[0])
        odom_msg.twist.twist.angular.z = float(-msg.angular_velocity[2])
        
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PX4TFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
