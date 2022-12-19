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
            name='GPS_Node'
        ),
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='MPU6050',
            name='IMU_Node'
        ),
        Node(
            package='image_tools',
            executable='cam2image',
            namespace='akillipaket',
            name='Image_Data',
            output='log',
            parameters=[
                {
                    'frequency': 10.0,
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
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='StateEstimator',
            name='Kalman_Filter_Node'
        ),
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='Control',
            name='Control_Node'
        ),
        Node(
            package='akillipaket',
            namespace='akillipaket',
            executable='LOG',
            name='LOG_Node',
        ),
    ])

