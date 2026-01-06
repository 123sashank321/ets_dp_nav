#!/usr/bin/env python3
"""
Goal Relay Node

Subscribes to /goal_update and relays goals to Nav2's navigate_to_pose action.
This enables dynamic object following by continuously updating the navigation target.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalRelay(Node):
    """Relays dynamic goals from topic to Nav2 action server."""

    def __init__(self):
        super().__init__('goal_relay')
        
        # Parameters
        self.update_distance_threshold = 0.5  # meters - only update if target moved this much
        
        # Subscribe to goal updates
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_update',
            self.goal_callback,
            10
        )
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        # State tracking
        self.last_goal = None
        self.current_goal_handle = None
        
        self.get_logger().info('Goal Relay started')
        self.get_logger().info(f'Update threshold: {self.update_distance_threshold}m')
        self.get_logger().info('Waiting for Nav2 action server...')
        
        # Wait for action server
        self.nav_client.wait_for_server()
        self.get_logger().info('Connected to Nav2!')

    def goal_callback(self, msg):
        """Handle incoming goal updates."""
        # Check if goal has moved significantly
        if self.last_goal is not None:
            dx = msg.pose.position.x - self.last_goal.pose.position.x
            dy = msg.pose.position.y - self.last_goal.pose.position.y
            dz = msg.pose.position.z - self.last_goal.pose.position.z
            distance = (dx**2 + dy**2 + dz**2)**0.5
            
            if distance < self.update_distance_threshold:
                # Target hasn't moved enough, skip update
                return
        
        # Store new goal
        self.last_goal = msg
        
        # Cancel previous goal if it exists
        if self.current_goal_handle is not None:
            self.get_logger().info('Canceling previous goal...')
            future = self.current_goal_handle.cancel_goal_async()
            # Don't wait for cancellation, just send new goal
        
        # Send new goal to Nav2
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        
        self.get_logger().info(
            f'Sending new goal: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
        )
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle Nav2's response to goal."""
        self.current_goal_handle = future.result()
        
        if not self.current_goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            return
        
        self.get_logger().info('Goal accepted by Nav2')
        
        # Get result asynchronously
        result_future = self.current_goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle Nav2's final result."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal reached!')
        elif status == 5:  # CANCELED
            self.get_logger().info('Goal was canceled (likely preempted by new goal)')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
