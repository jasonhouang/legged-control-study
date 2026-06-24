from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='legged_wbc_ros2',
            executable='legged_wbc_node',
            name='legged_wbc_node',
            output='screen',
            parameters=[{
                'control_frequency': 100.0,
                'robot_mass': 20.0,
            }]
        ),
    ])
