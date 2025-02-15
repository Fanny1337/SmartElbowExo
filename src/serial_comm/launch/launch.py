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
        # Node(
        #     package='control_vr_package',
        #     executable='virtual_robot',
        #     name='virtual_robot'
        # ),
         Node(
            package='serial_communication_package',
            executable='serial_python_node',
            name='serial_python_node'
        )
    ])

       
    
