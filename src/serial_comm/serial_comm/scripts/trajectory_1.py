#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from serial_comm.srv import TrajectoryCommand
import serial
import time
import inspect
import numpy as np

class TrajectoryPlannerNode(Node):
    def __init__(self):
        super().__init__('trajectory_planner')
        self.srv = self.create_service(TrajectoryCommand, 'calculate_trajectory', self.calculate_trajectory_callback)

    def calculate_trajectory_callback(self, request, response):
        current_angle = request.current_angle
        next_angle = current_angle + 1 if current_angle < 90 else current_angle - 1
        response.next_angle = next_angle
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
