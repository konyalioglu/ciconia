import os
from setuptools import setup
from glob import glob


package_name = 'akillipaket'
mpu6050_driver   = 'akillipaket/MPU6050_driver'
calibration = 'calibration'
state_estimator = 'akillipaket/Sensor_Fusion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, mpu6050_driver, calibration, state_estimator],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Turan Konyalioglu',
    maintainer_email='turan.konyalioglu@gmail.com',
    description='ROS2 Package for a Parachute Drone',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MPU6050 = akillipaket.MPU6050_Node:main',
            'Magnet  = akillipaket.Magnetometer_Node:main',
            'GPS     = akillipaket.GPS_Node:main',
            'CameraCalibration = calibration.cameraCalibration:main',
            'Aruco   = akillipaket.ARUCO_Node:main',
            'MagnetometerCalibration = calibration.magnetometerCalibration:main',
            'GyroCalibration = calibration.gyroCalibration:main',
            'LevelCalibration = akillipaket.levelCalibration:main',
            'LOG  = akillipaket.LOG_Node:main',
            'StateEstimator = akillipaket.KALMAN_Node:main',
            'MotorNode = akillipaket.MOTOR_Node:main',
            'Control = akillipaket.Control_Node:main',
        ],
    },
)
