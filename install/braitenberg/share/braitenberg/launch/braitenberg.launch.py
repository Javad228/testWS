from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='braitenberg',
            executable='braitenberg_node',
            name='braitenberg',
            parameters=[{
                'gain': 1.2,
                'turn_scale': 1.0,
            }],
            output='screen'
        )
    ])
