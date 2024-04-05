from launch import LaunchDescription
from launch_ros.actions import Node
from glob import glob
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'dev':'/dev/input/js0'}]
        )
    ])