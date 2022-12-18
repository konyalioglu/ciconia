#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Float32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
#from .Sensor_Fusion.Kalman_Filter import State_Estimation
from ciconia_msgs.msg import estimatedStates
import numpy as np
from utils import *


class State_Estimator():

    def __init__(self):

        self.phi = 0
        self.theta = 0
        self.psi = 0

        self.u = 0
        self.v = 0
        self.w = 0

        self.x = 0
        self.y = 0
        self.z = 0

        self.p = 0
        self.q = 0
        self.r = 0

        self.gps_flag = 0

        rospy.init_node('state_estimator')

        self.dt = 0.01

        rospy.Subscriber("/xsens/filter/positionlla", Vector3Stamped, self._position_handler)
        rospy.Subscriber("/xsens/filter/velocity", Vector3Stamped, self._velocity_handler)
        rospy.Subscriber("/xsens/sensor/imu", Imu, self._imu_handler)

        self.state_estimator_publisher_ = rospy.Publisher('/ciconia/filter/states', estimatedStates, queue_size=4)
        self._prediction_timer = rospy.Timer(rospy.Duration(self.dt), self.__timer_callback)

        self.estimated_states = estimatedStates() 

        self.gps_variance = 2.0
        self.imu_accel_variance = 0.01
        self.imu_gyro_variance = 0.0015
        self.mag_variance = 0.015
        self.aru_variance = 0.0001
        self.aru_yaw_var = 0.1
        gammat = 10
        gammar = 50

        Qt_gps = np.array([[self.gps_variance, 0.0, 0.0],
                           [0.0, self.gps_variance, 0.0],
                           [0.0, 0.0, self.gps_variance]])

        Qt_aru = np.array([[self.aru_variance, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, self.aru_variance, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, self.aru_variance, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, self.aru_variance, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, self.aru_variance, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, self.aru_variance, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.aru_yaw_var]])

        Qt_aru2 = np.array([[self.aru_variance, 0.0, 0.0, 0.0],
                           [0.0, self.aru_variance, 0.0, 0.0],
                           [0.0, 0.0, self.aru_variance, 0.0],
                           [0.0, 0.0, 0.0, self.aru_yaw_var]])

        Qt_imu = np.array([[self.imu_accel_variance, 0.0, 0.0, 0.0],
                           [0.0, self.imu_accel_variance, 0.0, 0.0],
                           [0.0, 0.0, self.imu_accel_variance, 0.0],
                           [0.0, 0.0, 0.0, self.imu_gyro_variance]])

        Qt_mag = np.array([[self.mag_variance]])

        P0 = np.array([[50,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0, 50,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0, 50,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0, 10,  0,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0, 10,  0,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0],
                       [ 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1]])

        #self.kalman_ = State_Estimation(P0, gammat, gammar, Qt_gps, Qt_imu, Qt_mag, Qt_aru, Qt_aru2)
        self.state_msg = Float64MultiArray()
        self.r = 0.0
        self.k = 0
        self.prev_heading = None
        self.marker_yaw_offset = 0.0
        self.yaw_offset_counter = 0
        self.x_aru_prev = 0
        self.y_aru_prev = 0
        self.z_aru_prev = 0


    def __timer_callback(self, event):
        #self.get_logger().info('Prediction')
        #xt = self.kalman_.xt
        #self.state_msg.data  = [xt[0,0], xt[1,0], xt[2,0], xt[3,0], xt[4,0], xt[5,0], xt[6,0], xt[7,0], xt[8,0], xt[9,0]*180/np.pi, xt[10,0], xt[11,0], xt[12,0], xt[13,0], self.r]
        #self.state_estimator_publisher_.publish(self.state_msg)
        self.estimated_states.position.x = self.x
        self.estimated_states.position.y = self.y
        self.estimated_states.position.z = self.z
        self.estimated_states.velocity.x = self.u
        self.estimated_states.velocity.y = self.v
        self.estimated_states.velocity.z = self.w
        self.estimated_states.angular_rate.x = self.p
        self.estimated_states.angular_rate.y = self.q
        self.estimated_states.angular_rate.z = self.r
        self.estimated_states.euler_angle.x = self.phi
        self.estimated_states.euler_angle.y = self.theta
        self.estimated_states.euler_angle.z = self.psi
        self.state_estimator_publisher_.publish(self.estimated_states)


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
            self.prev_heading = heading
            return 2 * self.k *  np.pi + heading
        else:
            return heading


    def _position_handler(self, msg):
        lat = msg.vector.x
        lon = msg.vector.y
        alt = msg.vector.z
        r_earth = 6371000
        self.z = -alt

        if self.gps_flag == 0:
            self.init_lat  = lat - 45
            self.init_long = lon - 12
            self.gps_flag = 1
        else:
            self.latitude  = lat - 45
            self.longitude = lon - 12

            a  = 6378137.0
            b  = 6356752.3

            self.x = a * np.sin(np.radians(self.latitude))
            self.y = b * np.sin(np.radians(self.longitude)) * np.cos(np.radians(45))
            
    
    def _velocity_handler(self, msg):
        self.u = msg.vector.x
        self.v = msg.vector.y
        self.w = msg.vector.z

    def _imu_handler(self, msg):
        self.phi, self.theta, self.psi = quaternion_to_euler_angle(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z

        return

def main():
    sf = State_Estimator()

    rospy.spin()


if __name__ == '__main__':
    main()
