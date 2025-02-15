#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.animation import FuncAnimation

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.position_subscription = self.create_subscription(Float64, 'current_elbow_position', self.position_callback, 10)
        self.command_subscription = self.create_subscription(Float64, 'position_cmd', self.command_callback, 10)
        self.touch1_subscription = self.create_subscription(Int32, 'touch1', self.touch1_callback, 10)
        self.touch2_subscription = self.create_subscription(Int32, 'touch2', self.touch2_callback, 10)
        
        # New DataFrame columns for additional data
        self.data = pd.DataFrame(columns=['Time', 'Position', 'Commanded Position', 'Touch1', 'Touch2', 'Command Timestamp'])
        self.start_time = self.get_clock().now().nanoseconds

    def position_callback(self, msg):
        now = (self.get_clock().now().nanoseconds - self.start_time) / 1e9  # Convert to seconds
        self.data = self.data.append({'Time': now, 'Position': msg.data}, ignore_index=True)

    def command_callback(self, msg):
        now = (self.get_clock().now().nanoseconds - self.start_time) / 1e9  # Convert to seconds
        self.data = self.data.append({'Time': now, 'Commanded Position': msg.data}, ignore_index=True)

    def touch1_callback(self, msg):
        now = (self.get_clock().now().nanoseconds - self.start_time) / 1e9  # Convert to seconds
        self.data.loc[self.data.Time == now, 'Touch1'] = msg.data

    def touch2_callback(self, msg):
        now = (self.get_clock().now().nanoseconds - self.start_time) / 1e9  # Convert to seconds
        self.data.loc[self.data.Time == now, 'Touch2'] = msg.data

def plot_data(data):
    plt.figure(figsize=(12, 10))
    
    plt.subplot(4, 1, 1)
    plt.plot(data['Time'], data['Position'], label='Actual Position (rad)')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (rad)')
    plt.title('Position Over Time')
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(data['Time'], data['Commanded Position'], label='Commanded Position (rad)', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Commanded Position (rad)')
    plt.title('Commanded Position Over Time')
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.step(data['Time'], data['Touch1'], label='Touch1', where='post')
    plt.xlabel('Time (s)')
    plt.ylabel('Touch1 State')
    plt.title('Touch1 Over Time')
    plt.grid(True)
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.step(data['Time'], data['Touch2'], label='Touch2', where='post')
    plt.xlabel('Time (s)')
    plt.ylabel('Touch2 State')
    plt.title('Touch2 Over Time')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

def main(args=None):
    rclpy.init(args=args)
    collector = DataCollector()
    
    try:
        plt.ion()
        fig, ax = plt.subplots(4, 1, figsize=(12, 10))
        def update(frame):
            ax[0].clear()
            ax[1].clear()
            ax[2].clear()
            ax[3].clear()
            
            if not collector.data.empty:
                ax[0].plot(collector.data['Time'], collector.data['Position'], label='Actual Position (rad)')
                ax[1].plot(collector.data['Time'], collector.data['Commanded Position'], label='Commanded Position (rad)', color='red')
                ax[2].step(collector.data['Time'], collector.data['Touch1'], label='Touch1', where='post')
                ax[3].step(collector.data['Time'], collector.data['Touch2'], label='Touch2', where='post')
            
            ax[0].set_title('Actual Position Over Time')
            ax[1].set_title('Commanded Position Over Time')
            ax[2].set_title('Touch1 Over Time')
            ax[3].set_title('Touch2 Over Time')
            for a in ax:
                a.grid(True)
                a.legend()

        ani = FuncAnimation(fig, update, interval=1000)
        plt.show(block=True)
        
        rclpy.spin(collector)
    except KeyboardInterrupt:
        plot_data(collector.data)
    finally:
        collector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
