import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import serial
import threading

class SerialCommunicationNode(Node):
    def __init__(self):
        super().__init__('serial_communication_node')
        
        # Initialize serial port
        try:
            self.port = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.1)
        except serial.SerialException as e:
            self.get_logger().fatal(f'Failed to open serial port: {str(e)}')
            rclpy.shutdown()
        
        # Publishers for touch sensors and current position
        self.position_publisher = self.create_publisher(Float64, 'current_elbow_position', 10)
        self.touch1_publisher = self.create_publisher(Int32, 'touch1', 10)
        self.touch2_publisher = self.create_publisher(Int32, 'touch2', 10)

        # Subscriber to receive commanded positions
        self.position_subscriber = self.create_subscription(
            Float64, 'elbow_position_cmd', self.position_command_callback, 10)
        
        # Timer to regularly read serial data from Arduino
        self.timer = self.create_timer(0.1, self.read_serial)
        
    def position_command_callback(self, msg):
        degrees = max(0, min(180, int(round(msg.data))))
        command = f'{degrees}\n'
        try:
            self.port.write(command.encode())
            self.get_logger().info(f'Sending new position command to Arduino: {degrees} degrees')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to write to serial port: {str(e)}')
    
    def read_serial(self):
        try:
            if self.port.in_waiting > 0:
                line = self.port.readline().decode().strip()
                if line:
                    self.process_line(line)
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from serial port: {str(e)}')

    def process_line(self, line):
        parts = line.split(',')
        if len(parts) == 3:
            try:
                position = float(parts[0])
                touch1 = int(parts[1])
                touch2 = int(parts[2])

                self.position_publisher.publish(Float64(data=position))
                self.touch1_publisher.publish(Int32(data=touch1))
                self.touch2_publisher.publish(Int32(data=touch2))
            except ValueError as e:
                self.get_logger().error(f'Error parsing serial data: {str(e)}')
        else:
            self.get_logger().error(f'Invalid or incomplete data received: "{line}"')

def main(args=None):
    rclpy.init(args=args)
    node = SerialCommunicationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
