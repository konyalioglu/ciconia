from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import os
from ament_index_python import get_package_share_directory
import time

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['rm','-r', 'src/akillipaket/log/IMU'],
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'src/akillipaket/log/IMU', '/akillipaket/IMU/Sensor_Data'],
        ),
        launch.actions.ExecuteProcess(
            cmd=['rm','-r', 'src/akillipaket/log/GPS'],
        ),
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'src/akillipaket/log/GPS', '/akillipaket/GPS/fix'],
        ),
    ])
