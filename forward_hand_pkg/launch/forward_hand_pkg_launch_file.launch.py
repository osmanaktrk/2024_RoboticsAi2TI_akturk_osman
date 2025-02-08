from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='forward_hand_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='forward_hand_pkg',
            executable='robot_control_node',
            name='robot_control_node',
            output='screen'
        )
    ])

