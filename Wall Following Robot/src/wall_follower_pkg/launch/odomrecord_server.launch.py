from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follower_pkg',
            executable='odomrecord_server',
            name='odomrecord_server_node',
            output='screen'),
    ])