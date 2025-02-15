#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from turtle_msgs.msg import ExoState

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        # Initialize the current and target positions in radians
        self.current_position = 0.0
        self.target_position = 0.0 
        self.publisher_ = self.create_publisher(Float64, 'position_cmd', 10)
        self.get_logger().warn(f'init <---------------trajectory planner')

        self.exo_subscription = self.create_subscription(
                ExoState,
                'exo_state',
                self.current_exo_callback,
                10)

    def current_exo_callback(self, msg):
        # Update the current position from the topic directly in radians
        self.get_logger().warn(f'exo_callback <---------------trajectory planner')
        self.current_position = msg.jdata
       
        # Check if button 1 is pushed
        if msg.tdata[0]:
            self.get_logger().warn(f'button 1 <---------------trajectory planner')
            self.target_position = min(self.current_position + 0.52, 1.56)

        # Check if button 2 is pushed
        elif msg.tdata[1]:
            self.get_logger().warn(f'button 2 <---------------trajectory planner')
            self.target_position = max(self.current_position - 0.52, 0.0)

        self.publish_desired_position(self.target_position)               

    def publish_desired_position(self, position):
        self.get_logger().warn(f'publish <---------------trajectory planner')
        msg = Float64()
        msg.data = position
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    trajectory_planner = TrajectoryPlanner()
    rclpy.spin(trajectory_planner)
    trajectory_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()