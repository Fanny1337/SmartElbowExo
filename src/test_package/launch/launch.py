from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_vr_package',
            executable='trajectory_planner',
            name='trajectory_planner'
        ),
        Node(
            package='control_vr_package',
            executable='control_node',
            name='control_node'
        ),
        Node(
            package='control_vr_package',
            executable='virtual_robot',
            name='virtual_robot'
        ),
        Node(
            package='test_package',
            executable='test_torque_publisher',
            name='test_torque_publisher'
        )
    ])
