import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
from sensor_msgs.msg import Imu
from .Sensor_Fusion.Kalman_Filter import State_Estimation
import numpy as np
from .utils import *


class KalmanFilter(Node):
    def __init__(self):
        super().__init__('kalman_node')
        self.dt = 0.02
        self.subscription_mag = self.create_subscription(Float32, '/akillipaket/IMU/Magnetometer', self.__mag_callback, 4)
        self.subscription_imu = self.create_subscription(Imu, '/akillipaket/IMU/Sensor_Data', self.__imu_callback, 4)
        self.subscription_gps = self.create_subscription(Float64MultiArray, '/akillipaket/GPS/fix', self.__gps_callback, 4)
        self.subscription_aru = self.create_subscription(Float64MultiArray, '/akillipaket/Aruco/Marker/relativePose', self.__aru_callback, 4)
        self.state_estimator_publisher_ = self.create_publisher(Float64MultiArray, '/akillipaket/Kalman/States', 4)

        self.gps_variance = 2.5
        self.imu_accel_variance = 0.1
        self.imu_gyro_variance = 0.0001
        self.mag_variance = 0.01
        self.aru_variance = 0.001
        self.aru_yaw_var = 0.001
        gamma = 0.05

        Qt_gps = np.array([[self.gps_variance, 0.0, 0.0],
                           [0.0, self.gps_variance, 0.0],
                           [0.0, 0.0, self.gps_variance]])

        Qt_aru = np.array([[self.aru_variance, 0.0, 0.0, 0.0],
                           [0.0, self.aru_variance, 0.0, 0.0],
                           [0.0, 0.0, self.aru_variance, 0.0],
                           [0.0, 0.0, 0.0, self.aru_yaw_var]])

        Qt_imu = np.array([[self.imu_accel_variance, 0.0, 0.0, 0.0],
                           [0.0, self.imu_accel_variance, 0.0, 0.0],
                           [0.0, 0.0, self.imu_accel_variance, 0.0],
                           [0.0, 0.0, 0.0, self.imu_gyro_variance]])

        Qt_mag = np.array([[self.mag_variance]])

        P0 = np.array([[10,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  5,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  5,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1]])

        self.kalman_ = State_Estimation(P0, gamma, Qt_gps, Qt_imu, Qt_mag, Qt_aru)
        self.state_msg = Float64MultiArray()
        self.timer = self.create_timer(self.dt, self.__timer_callback)
        self.r = 0.0
        self.k = 0
        self.prev_heading = None
        self.marker_yaw_offset = None


    def __timer_callback(self):
        #self.get_logger().info('Prediction')
        xt = self.kalman_.xt
        self.state_msg.data  = [xt[0,0], xt[1,0], xt[2,0], xt[3,0], xt[4,0], xt[5,0], xt[6,0], xt[7,0], xt[8,0], xt[9,0]*180/np.pi, xt[10,0], xt[11,0], xt[12,0], xt[13,0], self.r]
        self.state_estimator_publisher_.publish(self.state_msg)
        self.kalman_.KF_Predict(self.dt)


    def __heading_handler(self, heading):
        if self.prev_heading == None:
            self.prev_heading = heading
            return heading
        if not self.kalman_.xt[9,0] == 0.0:
            error = heading - self.prev_heading
            if error > np.pi and self.kalman_.xt[9,0] < -np.pi/2 + 2 * self.k * np.pi:
                self.k = self.k - 1
            elif error < -np.pi and self.kalman_.xt[9,0] > np.pi/2 + 2 * self.k * np.pi:
                self.k = self.k + 1
            #self.get_logger().info(str(180/np.pi*heading) + ' ' + str(180/np.pi*self.prev_heading) + ' ' + str(180/np.pi*self.kalman_.xt[9,0]) + ' ' + str(self.k))
            self.prev_heading = heading
            return 2 * self.k *  np.pi + heading
        else:
            return heading


    def __mag_callback(self, msg):
        #self.get_logger().info('Magnetometer Correction')
        heading = self.__heading_handler(msg.data*np.pi/180)
        zt = np.array([[heading]])
        self.kalman_.MAG_Update(zt)


    def __imu_callback(self, msg):
        #self.get_logger().info('IMU Correction')
        accel = np.array([[msg.linear_acceleration.x], [msg.linear_acceleration.y], [msg.linear_acceleration.z]])
        angular_rates  = np.array([[msg.angular_velocity.x], [msg.angular_velocity.y], [msg.angular_velocity.z]])
        euler_angles = np.array([[msg.orientation.x], [msg.orientation.y], [self.kalman_.xt[9,0]]])
        self.r = angular_rates[2,0]

        accel = body2earth_transformation(euler_angles, accel)
        _,_,yaw_rate = body2earth_rate_transformation(euler_angles, angular_rates)
        zt = np.array([[accel[0,0]],[accel[1,0]],[accel[2,0]],[angular_rates[2,0]]])
        self.kalman_.IMU_Update(zt)


    def __gps_callback(self, msg):
        #self.get_logger().info('GPS Correction')
        z_gps = np.array([[msg.data[0]], [msg.data[1]], [-msg.data[2]]])
        self.kalman_.GPS_Update(z_gps)


    def __aru_callback(self, msg):
        heading = msg.data[3]*np.pi/180
        if self.marker_yaw_offset == None and self.kalman_.xt[9,0] != 0.0:
            self.marker_yaw_offset = self.kalman_.xt[9,0]*np.pi/180 - heading
            return
        elif self.marker_yaw_offset != None:
            heading = self.__heading_handler(heading)
            z_aru = np.array([[msg.data[0]], [msg.data[1]], [-msg.data[2]], [heading + self.marker_yaw_offset]])
            self.kalman_.ARU_Update(z_aru)


def main():
    rclpy.init()
    sf = KalmanFilter()

    rclpy.spin(sf)

    sf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
