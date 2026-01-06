#!/usr/bin/env python3
"""
Dynamic Point Generator Node for Drone Following

Publishes a moving target point in a circular trajectory:
- Radius: 5.0m
- Height: 2.0m (constant)
- Frequency: 10Hz
- Frame: map (ENU)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TargetGenerator(Node):
    """Generates a circular moving target for the drone to follow."""

    def __init__(self):
        super().__init__('target_generator')
        
        # Parameters
        self.radius = 5.0  # meters
        self.height = 2.0  # meters
        self.frequency = 10.0  # Hz
        self.angular_velocity = 0.2  # rad/s (completes circle in ~31 seconds)
        
        # Publisher
        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_update',
            10
        )
        
        # Timer for publishing at desired frequency
        timer_period = 1.0 / self.frequency
        self.timer = self.create_timer(timer_period, self.publish_goal)
        
        # Internal state
        self.start_time = None
        
        self.get_logger().info(
            f'Target Generator started: radius={self.radius}m, '
            f'height={self.height}m, freq={self.frequency}Hz'
        )

    def publish_goal(self):
        """Compute and publish the current target position."""
        current_time = self.get_clock().now()
        
        # Initialize start time on first valid clock message
        if self.start_time is None:
            # Wait for valid clock if using sim time
            self.get_logger().info(f'Current time: {current_time.nanoseconds/1e9:.2f}')
            if current_time.nanoseconds == 0:
                self.get_logger().info('Waiting for valid clock...')
                return
            self.start_time = current_time
            self.get_logger().info(f'Target generation started at time: {current_time.nanoseconds/1e9:.2f}')
        
        # Log every 5 seconds to show it's alive
        if (current_time.nanoseconds % 5000000000) < 100000000:
             self.get_logger().info(f'Publishing goal at time: {current_time.nanoseconds/1e9:.2f}')
            
        # Calculate elapsed time
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Parametric equations for circular motion in ENU frame
        # Circle in the XY plane at constant height
        theta = self.angular_velocity * elapsed
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        z = self.height
        
        # Create and populate message
        msg = PoseStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Orientation: face tangent to the circle (forward direction)
        # Yaw = theta + pi/2 (perpendicular to radius)
        yaw = theta + math.pi / 2.0
        
        # Convert yaw to quaternion (rotation around z-axis)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Publish
        self.goal_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
