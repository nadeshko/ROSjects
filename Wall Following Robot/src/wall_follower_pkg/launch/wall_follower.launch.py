from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower_pkg',
            executable='wall_follower',
            name='wall_follower_node',
            output='screen'),
    ])