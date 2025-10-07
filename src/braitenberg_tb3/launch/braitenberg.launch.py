from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='braitenberg_tb3',
            executable='braitenberg_node',
            name='braitenberg_tb3',
            parameters=[{
                'gain': 1.2,
                'base_speed': 0.12,
                'turn_scale': 1.0,
            }],
            output='screen'
        )
    ])
