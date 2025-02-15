# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float64, Int32

# class TrajectoryPlanner(Node):
#     def __init__(self):
#         super().__init__('trajectory_planner')
#         self.publisher_ = self.create_publisher(Float64, 'elbow_position_des', 10)

#         # Subscription to current position of the elbow
#         self.position_subscription = self.create_subscription(
#             Float64,
#             'current_elbow_position',
#             self.current_position_callback,
#             10)

#         # Subscriptions to touch sensor states
#         self.touch1_subscription = self.create_subscription(
#             Int32,
#             'touch1',
#             self.touch1_callback,
#             10)
#         self.touch2_subscription = self.create_subscription(
#             Int32,
#             'touch2',
#             self.touch2_callback,
#             10)

#         # Initialize the current and target positions in radians
#         self.current_position = 0.0  # This will be updated with actual encoder values in radians
#         self.target_position = self.current_position  # Initially set to the current position

#     def current_position_callback(self, msg):
#         # Update the current position from the topic directly in radians
#         self.current_position = msg.data
#         self.get_logger().info(f'Current elbow position updated to: {self.current_position:.2f} radians')
#         self.update_target_position()

#     def touch1_callback(self, msg):
#         if msg.data == 1:
#             self.target_position += 0.261799  # Increase by 15 degrees converted to radians
#             self.update_target_position()

#     def touch2_callback(self, msg):
#         if msg.data == 1:
#             self.target_position -= 0.261799  # Decrease by 15 degrees converted to radians
#             self.update_target_position()

#     def update_target_position(self):
#         # Ensure the target position remains within practical limits of 0 to π/2 radians
#         self.target_position = max(min(self.target_position, 1.5708), 0)  # Limits adjusted here
#         self.publish_desired_position(self.target_position)
#         self.get_logger().info(f'Target position adjusted to: {self.target_position:.2f} radians')

#     def publish_desired_position(self, position):
#         msg = Float64()
#         msg.data = position
#         self.publisher_.publish(msg)
#         self.get_logger().info(f'Published desired elbow position: {position:.2f} radians')

# def main(args=None):
#     rclpy.init(args=args)
#     trajectory_planner = TrajectoryPlanner()
#     rclpy.spin(trajectory_planner)
#     trajectory_planner.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()