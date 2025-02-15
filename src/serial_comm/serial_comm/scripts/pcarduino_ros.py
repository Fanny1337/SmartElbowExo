import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import serial
import inspect
import numpy as np

def debug_message(msg):
    frame = inspect.currentframe()
    caller_frame = frame.f_back
    file_name = caller_frame.f_code.co_filename
    line_number = caller_frame.f_lineno
    print(f"DEBUG {file_name}:{line_number} - {msg}")

class SerialCommunicationNode(Node):
    def __init__(self):
        #debug_message("Sophie")
        super().__init__('serial_communication_node')
        
        self.ser = serial.Serial('/dev/ttyUSB1', 9600)  # replace 'COM_PORT' with your Arduino port
        # Timer to regularly read serial data from Arduino
        self.timer = self.create_timer(1.0, self.read_serial)

        self.ti=self.get_clock().now()

    def move_servo(self,angle):
        print(f"target angle: {angle}")
        self.ser.write(str(angle).encode())  # send the angle as a string encoded to bytes

    def read_serial(self):
        # Read response from Arduino

        # The node waits for the Arduino and prints out the data sent
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            print(f"Arduino says: {line}")
        else:
            print(f"I got nothing!!!")

        # Compute a sinusoidal angle
        t = (self.get_clock().now().nanoseconds-self.ti.nanoseconds)/1E9 
        T=100.0
        A=45
        w=2*3.14159/T
        angle = int(A*np.sin(w*t)+A)
        print(f"angle={angle}")

        # Send the new angle to Arduino
        self.move_servo(angle)


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