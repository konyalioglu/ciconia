from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os
from ament_index_python import get_package_share_directory
import time

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_tools',
            executable='cam2image',
            name='Image_Data',
            output='log',
            parameters=[
                {
                    'frequency': 1.0,
                    'width': 1280,
                    'height': 720,
                }
            ]
        ),
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='Aruco',
            name='Aruco_Node'
        ),
    ])

