from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os
from ament_index_python import get_package_share_directory
import time

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='GPS',
            name='GPS_Data'
        ),
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='MPU6050',
            name='IMU_Data'
        ),
        Node(
            package='image_tools',
            executable='cam2image',
            name='Image_Data',
            #output='screen',
            parameters=[
                {
                    'frequency': 5.0,
                    'width': 480,
                    'height': 360,
                }
            ]
        ),
    ])

