import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import math
from std_msgs.msg import Float64MultiArray, Float32
from sensor_msgs.msg import Imu
import time


class LogNode(Node):

    def __init__(self):
        super().__init__('log_node')
        self.subscription_aruco_pos = self.create_subscription(Float64MultiArray, '/akillipaket/Aruco/Marker/relativePose', self.__aruco_pose_data_handler, 4)
        self.subscription_imu = self.create_subscription(Imu, '/akillipaket/IMU/Sensor_Data', self.__imu_data_handler, 4)
        self.subscription_mag = self.create_subscription(Float32, '/akillipaket/IMU/Magnetometer', self.__magnetometer_data_handler, 4)
        self.subscription_gps = self.create_subscription(Float64MultiArray, '/akillipaket/GPS/fix', self.__gps_data_handler, 4)
        self.subscription_pwm = self.create_subscription(Float64MultiArray, '/akillipaket/motor/input', self.__motor_input_data_handler, 4)
        self.subscription_ste = self.create_subscription(Float64MultiArray, '/akillipaket/Kalman/States', self.__kalman_data_handler, 4)

        self.time_ref = time.time()
        self.dir = 'src/akillipaket/log'
        self.gps_dir = self.dir + '/gps_data.csv'
        self.aruco_position_dir = self.dir + '/aruco_position_data.csv'
        self.aruco_heading_dir = self.dir + '/aruco_heading_data.csv'
        self.mag_dir = self.dir + '/magnetometer_data.csv'
        self.euler_angles_dir = self.dir + '/euler_angles_data.csv'
        self.gyro_dir = self.dir + '/gyro_data.csv'
        self.acceleration_dir = self.dir + '/acceleration_data.csv'
        self.kalman_filter_dir = self.dir + '/kalman_filter_data.csv'
        self.motor_input_dir = self.dir + '/motor_input_data.csv'

        np.savetxt(self.gps_dir, np.array([[0,0,0,0]]), delimiter=",")
        np.savetxt(self.aruco_position_dir, np.array([[0,0,0,0]]), delimiter=",")
        np.savetxt(self.aruco_heading_dir, np.array([[0,0]]), delimiter=",")
        np.savetxt(self.acceleration_dir, np.array([[0,0,0,0]]), delimiter=",")
        np.savetxt(self.gyro_dir, np.array([[0,0,0,0]]), delimiter=",")
        np.savetxt(self.euler_angles_dir, np.array([[0,0,0,0]]), delimiter=",")
        np.savetxt(self.mag_dir, np.array([[0,0]]), delimiter=",")
        np.savetxt(self.kalman_filter_dir, np.array([[0,0,0,0,0,0,0,0,0,0,0,0]]), delimiter=",")
        np.savetxt(self.motor_input_dir, np.array([[0,0,0]]), delimiter=",")


    def quaternion_to_euler_angle(self, w, x, y, z):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z


    def __motor_input_data_handler(self, msg):
        timer = time.time() - self.time_ref
        data = np.array([[timer, msg.data[0], msg.data[1]]])
        with open(self.motor_input_dir, 'a') as f:
            np.savetxt(f, data, delimiter=",")


    def __kalman_data_handler(self, msg):
        timer = time.time() - self.time_ref
        x     = msg.data
        state = np.array([[timer, x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10]]])
        with open(self.kalman_filter_dir, 'a') as f:
            np.savetxt(f, state, delimiter=",")


    def __gps_data_handler(self, msg):
        timer = time.time() - self.time_ref
        data = np.array([[timer, msg.data[0], msg.data[1], msg.data[2]]])
        with open(self.gps_dir, 'a') as f:
            np.savetxt(f, data, delimiter=",")


    def __aruco_pose_data_handler(self, msg):
        timer    = time.time() - self.time_ref
        position = np.array([[timer, msg.data[0], msg.data[1], msg.data[2]]])
        heading  = np.array([[timer, msg.data[3]]])
        with open(self.aruco_position_dir, 'a') as f:
            np.savetxt(f, position, delimiter=",")
        with open(self.aruco_heading_dir, 'a') as f:
            np.savetxt(f, heading, delimiter=",")


    def __imu_data_handler(self, msg):
        timer = time.time() - self.time_ref
        accel = np.array([[timer, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]])
        gyro  = np.array([[timer, msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]])
        phi, theta, psi = (msg.orientation.x, msg.orientation.y, msg.orientation.z)
        euler_angles = np.array([[timer, phi, theta, psi]])
        with open(self.acceleration_dir, 'a') as f:
            np.savetxt(f, accel, delimiter=",")
        with open(self.gyro_dir, 'a') as f:
            np.savetxt(f, gyro, delimiter=",")
        with open(self.euler_angles_dir, 'a') as f:
            np.savetxt(f, euler_angles, delimiter=",")


    def __magnetometer_data_handler(self, msg):
        timer = time.time() - self.time_ref
        data = np.array([[timer, msg.data]])
        with open(self.mag_dir, 'a') as f:
            np.savetxt(f, data, delimiter=",")



def main(args=None):
    rclpy.init(args=args)

    log = LogNode()

    rclpy.spin(log)

    log.destroy_node()

    rclpy.shutdown()


