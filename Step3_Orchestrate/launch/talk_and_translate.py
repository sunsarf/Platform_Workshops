from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hello_world_talker',
            namespace='hello_world_talker',
            executable='talk',
            name='talker'
        ),
        Node(
            package='hello_world_translator',
            namespace='hello_world_translator',
            executable='translate',
            name='translator'
        )
    ])