from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obstakel_forward_pkg',
            executable='obstakel_forward',
            output='screen'),
    ])

