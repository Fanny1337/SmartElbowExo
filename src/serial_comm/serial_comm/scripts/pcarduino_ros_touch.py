import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import numpy as np

class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        
        self.ser = serial.Serial('/dev/ttyUSB1', 9600)  # replace '/dev/ttyUSB0' with your Arduino port
        self.timer = self.create_timer(1.5, self.read_serial)

        self.ti = self.get_clock().now()

    def move_servo(self, angle):
        print(f"target angle: {angle}")
        self.ser.write(str(angle).encode())  # send the angle as a string encoded to bytes

    def read_serial(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                self.process_line(line)
        else:
            print(f"I got nothing!!!")
        
        t = (self.get_clock().now().nanoseconds - self.ti.nanoseconds) / 1E9 
        T = 100.0
        A = 45
        w = 2 * np.pi / T
        angle = int(A * np.sin(w * t) + A)
        self.move_servo(angle)

    def process_line(self, line):
        try:
            parts = line.split(',')
            position_info = parts[0].split(': ')[1]
            angle_info = parts[1].split(': ')[1]
            encoder_info = parts[2].split(': ')[1]
            touch1_info = parts[3].split(': ')[1]
            touch2_info = parts[4].split(': ')[1]
            
            position = float(position_info)
            actual_angle = int(angle_info)
            encoder_value = int(encoder_info)
            touch1 = int(touch1_info)
            touch2 = int(touch2_info)

            print(f"Commanded Position: {position}, Actual Angle: {actual_angle}, Encoder Value: {encoder_value}, Touch1: {touch1}, Touch2: {touch2}")

        except ValueError as e:
            self.get_logger().error(f'Error parsing serial data: {str(e)}')
        except IndexError as e:
            self.get_logger().error(f'Invalid or incomplete data received: "{line}"')

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
