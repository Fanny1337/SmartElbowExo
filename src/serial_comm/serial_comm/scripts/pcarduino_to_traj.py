#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from serial_comm.srv import TrajectoryCommand  
import serial
import time
import inspect
import numpy as np


class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        
        # Setting up the serial port
        self.ser = serial.Serial('/dev/ttyUSB1', 9600)  # replace '/dev/ttyUSB1' with your Arduino port
        self.current_angle = 90  # Start at the middle position to avoid large initial movements
        
        # Create a client for the trajectory calculation service
        self.client = self.create_client(TrajectoryCommand, 'calculate_trajectory')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = TrajectoryCommand.Request()

        # Timer to regularly update servo position
        self.timer = self.create_timer(1.5, self.update_servo)

    def update_servo(self):
        # Request the next target angle from the trajectory planning service
        self.req.current_angle = self.current_angle
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.handle_service_response)

    def handle_service_response(self, future):
        try:
            response = future.result()
            if response:
                target_angle = response.next_angle
                self.move_servo(target_angle)
                self.current_angle = target_angle  # Update the current angle
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))

    def move_servo(self, angle):
        self.ser.write(str(angle).encode())  # send the angle as a string encoded to bytes
        print(f"Moving to angle: {angle}")

    def read_serial(self):
        # Read response from Arduino
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            print(f"Arduino says: {line}")
        else:
            print(f"I got nothing!!!")

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicationNode()
    rclpy.spin(node)

    print("Closing serial port")
    node.ser.close()
    print("Destroying node")
    node.destroy_node()
    print("Shutting down")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
