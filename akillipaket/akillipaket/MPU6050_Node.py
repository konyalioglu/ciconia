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
from scipy.signal import butter, lfilter_zi, lfilter


class MPU6050_Node(Node):

    def __init__(self):
        super().__init__('MPU6050')
        self.mpu6050_publisher_ = self.create_publisher(Imu, '/akillipaket/IMU/Sensor_Data', 4)
        self.magnetometer_publisher_ = self.create_publisher(Float32, '/akillipaket/IMU/Magnetometer', 4)

        self.variation = 4.528986*(np.pi/180)

        self.i = 0
        self.i2c_bus = 1
        self.device_address = 0x68
        # The offsets are different for each device and should be changed
        # accordingly using a calibration procedure
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

        time.sleep(1)
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


        print (' get ready for acceleration x1')
        time.sleep(1)
        print (' x1 started')
        self.mpu9250 = FaBo9Axis_MPU9250.MPU9250()
        self.magnetometer_msg = Float32()
        self.sensor_msg = Imu()
        self.prev_heading = False
        self.k = 0

        with open('src/akillipaket/calibration/calibrationFiles/magnetometer_calibration.yaml') as f:
            config = yaml.safe_load(f)

        self.mag_x_ave = config['averages']['x']
        self.mag_y_ave = config['averages']['y']
        self.mag_z_ave = config['averages']['z']
        self.mag_x_sca = config['scaling']['x']
        self.mag_y_sca = config['scaling']['y']
        self.mag_z_sca = config['scaling']['z']
        self.variation = config['variation']


        with open('src/akillipaket/calibration/calibrationFiles/gyroscope_calibration.yaml') as f:
            config = yaml.safe_load(f)

        self.gyro_offset_x = config['offset']['x']
        self.gyro_offset_y = config['offset']['y']
        self.gyro_offset_z = config['offset']['z']


        with open('src/akillipaket/calibration/calibrationFiles/level_calibration.yaml') as f:
            config = yaml.safe_load(f)


        self.linear_accel_offset_x = config['acceleration']['offset']['x']
        self.linear_accel_offset_y = config['acceleration']['offset']['y']
        self.linear_accel_offset_z = config['acceleration']['offset']['z']

        self.euler_angle_offset_x = config['euler_angles']['offset']['x']
        self.euler_angle_offset_y = config['euler_angles']['offset']['y']


    def butter_lowpass_filter(self, data, prev_data, cutoff, fs, order):
        #print("Cutoff freq " + str(cutoff))
        nyq = 0.5 * fs # Nyquist Frequency
        normal_cutoff = cutoff / nyq

        b, a = butter(order, normal_cutoff, btype='low', analog=False)
        zi = lfilter(b, a, data, zi = prev_data)
        return y


    def euler_angles_to_quaternions(self, psi, theta, phi):
        w = np.cos(phi * 0.5) * np.cos(theta * 0.5) * np.cos(psi * 0.5) \
            + np.sin(phi * 0.5) * np.sin(theta * 0.5) * np.sin(psi * 0.5)
        x = np.sin(phi * 0.5) * np.cos(theta * 0.5) * np.cos(psi * 0.5) \
            - np.cos(phi * 0.5) * np.sin(theta * 0.5) * np.sin(psi * 0.5)
        y = np.cos(phi * 0.5) * np.sin(theta * 0.5) * np.cos(psi * 0.5) \
            + np.sin(phi * 0.5) * np.cos(theta * 0.5) * np.sin(psi * 0.5)
        z = np.cos(phi * 0.5) * np.cos(theta * 0.5) * np.sin(psi * 0.5) \
            - np.sin(phi * 0.5) * np.sin(theta * 0.5) * np.cos(psi * 0.5)
        return w, x, y, z


    def wrap(self, angle, offset):
        yaw = angle + offset
        if yaw > offset - np.pi and yaw < -np.pi:
            yaw = 3*np.pi/2 + angle
        return yaw


    def tilt_compensated_heading(self, bx, by, bz, phi, theta):
        """ Takes in raw magnetometer values, pitch and roll and turns it into a tilt-compensated heading value ranging from -pi to pi (everything in this      function should be in radians). """
        # magnetic variation for Corpus Christi, should match your bx/by/bz and where they were measured from (a lookup table is beyond the scope of this gist)
        Xh = bx * np.cos(theta) + by * np.sin(phi) * np.sin(theta) + bz * np.cos(phi) * np.sin(theta)
        Yh = by * np.cos(phi) - bz * np.sin(phi)
        return self.wrap(-np.arctan2(-Yh, Xh) + self.variation, -np.pi/2)


    def sensor_callback(self):

        self.FIFO_count = self.mpu.get_FIFO_count()
        self.mpu_int_status = self.mpu.get_int_status()

        # If overflow is detected by status or fifo count we want to reset
        if (self.FIFO_count == 1024) or (self.mpu_int_status & 0x10):
            self.mpu.reset_FIFO()


        # Check if fifo data is ready
        elif (self.mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while self.FIFO_count < self.packet_size:
                self.FIFO_count = self.mpu.get_FIFO_count()


            self.FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
            #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
            gyro = self.mpu.get_rotation()
            quat = self.mpu.DMP_get_quaternion_int16(self.FIFO_buffer)
            grav = self.mpu.DMP_get_gravity(quat)
            euler_angles = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
            accel = self.mpu.get_acceleration(self.FIFO_buffer, self.calibre_a)
            grav = self.mpu.get_gravity(quat,self.calibre_g)
            linear_accel = self.mpu.linear_acceleration(accel, grav)
            gyro = self.mpu9250.readGyro()
            mag = self.mpu9250.readMagnet()

            euler_angles.x = np.deg2rad(float(euler_angles.x))
            euler_angles.y = np.deg2rad(float(euler_angles.y))
            euler_angles.z = np.deg2rad(float(euler_angles.z))

            mag_roll  = euler_angles.y
            mag_pitch = -euler_angles.x


            if mag['x']!=0 and mag['y']!=0 and mag['z']!=0:
                mag = [(mag['x']-self.mag_x_ave)*self.mag_x_sca, (mag['y']-self.mag_y_ave)*self.mag_y_sca, (mag['z']-self.mag_z_ave)*self.mag_z_sca]
                heading = self.tilt_compensated_heading(mag[0], mag[1], mag[2], mag_roll, mag_pitch)
                self.magnetometer_msg.data = heading * 180 / np.pi
                self.magnetometer_publisher_.publish(self.magnetometer_msg)


            #euler_angles.x = np.deg2rad(float(euler_angles.x) - self.euler_angle_offset_x)
            #euler_angles.y = np.deg2rad(float(euler_angles.y) - self.euler_angle_offset_y)
            #euler_angles.z = np.deg2rad(float(euler_angles.z))

            self.mpu.reset_FIFO()

            self.sensor_msg.orientation.w = 0.0
            self.sensor_msg.orientation.x = euler_angles.x
            self.sensor_msg.orientation.y = euler_angles.y
            self.sensor_msg.orientation.z = euler_angles.z

            self.sensor_msg.linear_acceleration.x = linear_accel.x  #- self.linear_accel_offset_x)
            self.sensor_msg.linear_acceleration.y = linear_accel.y  #- self.linear_accel_offset_y)
            self.sensor_msg.linear_acceleration.z = linear_accel.z  #- self.linear_accel_offset_z)

            self.sensor_msg.angular_velocity.x = (float(gyro['x']) - self.gyro_offset_x) * np.pi / 180
            self.sensor_msg.angular_velocity.y = (float(gyro['y']) - self.gyro_offset_y) * np.pi / 180
            self.sensor_msg.angular_velocity.z = (float(gyro['z']) - self.gyro_offset_z) * np.pi / 180
            self.mpu6050_publisher_.publish(self.sensor_msg)







def main(args=None):
    rclpy.init(args=args)

    mpu6050 = MPU6050_Node()

    while rclpy.ok():
        mpu6050.sensor_callback()

    mpu6050.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
