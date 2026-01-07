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
        
        # Declare parameters
        self.declare_parameter('trajectory_type', 'line')  # 'circle' or 'line'
        self.declare_parameter('radius', 10.0)
        self.declare_parameter('height', 10.0)
        self.declare_parameter('frequency', 10.0)
        self.declare_parameter('angular_velocity', 0.4)
        self.declare_parameter('linear_velocity', 0.5)  # For straight line
        self.declare_parameter('line_distance', 40.0)  # Loop after this distance
        
        # Get parameters
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.radius = self.get_parameter('radius').value
        self.height = self.get_parameter('height').value
        self.frequency = self.get_parameter('frequency').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.line_distance = self.get_parameter('line_distance').value
        
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
            f'Target Generator started: type={self.trajectory_type}, '
            f'radius={self.radius}m, height={self.height}m, freq={self.frequency}Hz'
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
        
        # Calculate position based on trajectory type
        if self.trajectory_type == 'circle':
            # Parametric equations for circular motion in ENU frame
            theta = self.angular_velocity * elapsed
            x = self.radius * math.cos(theta)
            y = self.radius * math.sin(theta)
            z = self.height
            # Orientation: face tangent to the circle
            yaw = theta + math.pi / 2.0
            
        elif self.trajectory_type == 'line':
            # Straight line motion along X-axis (with looping)
            x = (self.linear_velocity * elapsed) % self.line_distance
            y = 0.0
            z = self.height
            # Orientation: face forward (along X-axis)
            yaw = 0.0
            
        else:
            self.get_logger().error(f'Unknown trajectory type: {self.trajectory_type}')
            return
        
        # Create and populate message
        msg = PoseStamped()
        msg.header.stamp = current_time.to_msg()
        msg.header.frame_id = 'map'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
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
