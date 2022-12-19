import rclpy
from rclpy.node import Node
import math
import time
from std_msgs.msg import String, Float64MultiArray, Float32
from .MPU6050_driver.MPU6050 import MPU6050
import FaBo9Axis_MPU9250
from sensor_msgs.msg import Imu
import numpy as np
import yaml

class levelCalibration(Node):

    def __init__(self):
        super().__init__('Level_Calibration')

        self.i = 0
        self.i2c_bus = 1
        self.device_address = 0x68
        self.x_accel_offset = -83
        self.y_accel_offset = 3934
        self.z_accel_offset = -801
        self.x_gyro_offset = 0
        self.y_gyro_offset = 0
        self.z_gyro_offset = 0
        self.gravity = 9.8065
        self.enable_debug_output = True

        self.mpu = MPU6050(self.i2c_bus, self.device_address, self.x_accel_offset, self.y_accel_offset,
                      self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset, self.z_gyro_offset,
                      self.enable_debug_output)

        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        self.mpu_int_status = self.mpu.get_int_status()
        print(hex(self.mpu_int_status))

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        print(self.packet_size)
        self.FIFO_count = self.mpu.get_FIFO_count()
        print(self.FIFO_count)
        self.value = 0
        self.count = 0
        self.FIFO_buffer = [0]*64
        self.counter = 0
        self.FIFO_count_list = list()

        self.calibre_a = 0.00123563121085
        self.calibre_g = 3.65345386383e-08

        self.sum_roll = 0
        self.sum_pitch = 0
        self.counter = 0
        self.linear_accel_sum_x = 0
        self.linear_accel_sum_y = 0
        self.linear_accel_sum_z = 0

        self.dmp_convergence = False


    def sensor_call(self):

        self.FIFO_count = self.mpu.get_FIFO_count()
        self.mpu_int_status = self.mpu.get_int_status()

        if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
            self.mpu.reset_FIFO()

        elif (self.mpu_int_status & 0x02):
            while self.FIFO_count < self.packet_size:
                self.FIFO_count = self.mpu.get_FIFO_count()

            self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
            gyro = self.mpu.get_rotation()
            quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
            grav = self.mpu.DMP_get_gravity(quat)
            euler_angles = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
            accel = self.mpu.get_acceleration(self.FIFO_buffer, self.calibre_a)
            grav = self.mpu.get_gravity(quat,self.calibre_g)
            linear_accel = self.mpu.linear_acceleration(accel, grav)
            self.mpu.reset_FIFO()

            print(linear_accel.x, ' ', linear_accel.y, ' ', linear_accel.z)
            if self.dmp_convergence == False and abs(linear_accel.x) < 0.01 and abs(linear_accel.y) < 0.01 and abs(linear_accel.z) < 0.05:
                self.dmp_convergence = True
                print('DMP has converged!')

            if self.dmp_convergence:
                self.sum_roll  += float(euler_angles.x)
                self.sum_pitch += float(euler_angles.y)
                self.counter += 1
                self.linear_accel_sum_x += linear_accel.x
                self.linear_accel_sum_y += linear_accel.y
                self.linear_accel_sum_z += linear_accel.z


    def calibrate(self):
        roll_offset  = self.sum_roll / self.counter
        pitch_offset = self.sum_pitch / self.counter
        accel_offset_x = self.linear_accel_sum_x / self.counter
        accel_offset_y = self.linear_accel_sum_y / self.counter
        accel_offset_z = self.linear_accel_sum_z / self.counter

        data = {'acceleration':{'offset':{'x':accel_offset_x, 'y':accel_offset_y, 'z':accel_offset_z}},
                'euler_angles':{'offset':{'x':roll_offset, 'y':pitch_offset}}}


        self.write_yaml(data)
        print('\nOffset data was written to "level_calibration.yaml" file as the values are: ')
        print('\n x: ', str(roll_offset), 'y: ', str(pitch_offset))

    def write_yaml(self, data):
        with open('src/akillipaket/calibration/calibrationFiles/level_calibration.yaml', 'a') as f:
            f.truncate(0)
            yaml.dump(data, f, default_flow_style=False)





def main(args=None):
    rclpy.init(args=args)

    level = levelCalibration()

    freq = 30
    calibration_duration = 20
    rate_counter = 0

    level.get_logger().info('Level calibration started and it will last ' + str(calibration_duration) + 'seconds after DMP convergence')
    while True:
        if rate_counter >= calibration_duration*freq:
            level.calibrate()
            level.get_logger().info('Level calibration has been done!')
            break
        if level.dmp_convergence:
            rate_counter += 1
        level.sensor_call()
        time.sleep(1/freq)


    level.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

