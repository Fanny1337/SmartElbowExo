import serial
import time
import rclpy
from rclpy.node import Node
import numpy as np

class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        
        self.ser = serial.Serial('/dev/ttyUSB1', 9600)  # replace '/dev/ttyUSB1' with your Arduino port
        self.current_angle = 90  # Start at the middle position to avoid large initial movements
        self.increment = 0.5  # Set the angle increment for each timer call; adjust as needed for finer control

        # Timer to regularly update servo position, adjust the period for smoother operation
        self.timer = self.create_timer(1.5, self.update_servo)  # Timer calls update_servo every 0.1 seconds
        self.read_timer = self.create_timer(0.8, self.read_serial)  # Timer calls read_serial every 0.5 seconds
    def update_servo(self):
        # Incrementally adjust the angle for smoother movement
        self.current_angle += self.increment  # Increment the angle slightly
        if self.current_angle > 90:
            self.increment = -1  # Start decreasing the angle if it reaches the upper bound
        elif self.current_angle < 0:
            self.increment = 1  # Start increasing the angle if it reaches the lower bound

        # Ensure the angle remains within bounds
        limited_angle = max(0, min(90, int(self.current_angle)))
        self.move_servo(limited_angle)
        print(f"Moving to angle: {limited_angle}")

    def move_servo(self, angle):
        self.ser.write(str(angle).encode())  # send the angle as a string encoded to bytes
        print(f"target angle: {angle}")

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
