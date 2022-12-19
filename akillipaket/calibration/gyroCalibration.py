import yaml
import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
import FaBo9Axis_MPU9250



class gyroCalibration(Node):

    def __init__(self):
        super().__init__('Gyroscope_Calibration')

        self.mpu9250 = FaBo9Axis_MPU9250.MPU9250()

        self.sum_x = 0
        self.sum_y = 0
        self.sum_z = 0

        self.average_x = 0
        self.average_y = 0
        self.average_z = 0

        self.counter = 0



    def sensor_call(self):
        gyro = self.mpu9250.readGyro()
        self.sum_x += float(gyro['x'])
        self.sum_y += float(gyro['y'])
        self.sum_z += float(gyro['z'])
        self.counter += 1

    def calibrate(self):
        gyro_offset_x = self.sum_x / self.counter
        gyro_offset_y = self.sum_y / self.counter
        gyro_offset_z = self.sum_z / self.counter

        data = {'offset':{'x':gyro_offset_x, 'y':gyro_offset_y, 'z':gyro_offset_z}}

        self.write_yaml(data)
        print('\nOffset data was written to "gyroscope_calibration.yaml" file as the values are: ')
        print('\n x: ', str(gyro_offset_x), 'y: ', str(gyro_offset_y), 'z: ', str(gyro_offset_z))

    def write_yaml(self, data):
        with open('src/akillipaket/calibration/calibrationFiles/gyroscope_calibration.yaml', 'a') as f:
            f.truncate(0)
            yaml.dump(data, f, default_flow_style=False)






def main(args=None):
    rclpy.init(args=args)

    gyro = gyroCalibration()

    freq = 30
    calibration_duration = 40
    rate_counter = 0

    gyro.get_logger().info('Gyroscope calibration started and it will last ' + str(calibration_duration) + 'seconds')
    while True:
        if rate_counter >= calibration_duration*freq:
            gyro.calibrate()
            gyro.get_logger().info('Gyroscope calibration has been done!')
            break
        rate_counter += 1
        gyro.sensor_call()
        time.sleep(1/freq)


    gyro.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

