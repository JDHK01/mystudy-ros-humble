#!/usr/bin/env python3
"""
Action server node
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_tutorials_interfaces.action import Fibonacci


class ActionServerNode(Node):
    """Action server implementation"""
    
    def __init__(self):
        super().__init__('action_server_node')
        
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        
        self.get_logger().info('Action server ready')
    
    def execute_callback(self, goal_handle):
        """Execute the action"""
        self.get_logger().info('Executing goal...')
        
        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        # Process goal
        for i in range(1, goal_handle.request.order):
            # Check if goal is canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            # Update feedback
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence')}
        
        # Set result
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('Goal succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ActionServerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()