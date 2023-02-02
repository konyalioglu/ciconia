#! /usr/bin/python3

import numpy as np
import rospy

from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64MultiArray

from utils import *
from ekf import EKF


class AHRS:

    def __init__(self):
    
        self.u_ned_w = 0
        self.v_ned_w = 0
        self.w_ned_w = 0

        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.quat_w = 0   

        self.euler_angles = np.array([[0],[0],[0]])

        #Default Parameters
        self.ahrs_rate = 100

        self.acc_topic = "/ahrs/imu/accelerometer"
        self.mag_topic = "/ahrs/imu/magnetometer"
        self.gyr_topic = "/ahrs/imu/angular_rates"

        self.accelerometer_cov = [0.01, 0.01, 0.01]
        self.magnetometer_cov = [0.01, 0.01, 0.01]
        self.gyroscope_cov = [0.01, 0.01, 0.01]
        self.process_cov = [1.0, 1.0, 1.0]

        self.states = Float64MultiArray()

        

        ##Ros 
        self._node_name = 'ahrs'       
        self.initialize_model()

        self.ekf = EKF(1/self.ahrs_rate, self.accelerometer_cov, self.magnetometer_cov, self.gyroscope_cov)
        
        #Initialization
        rospy.init_node(self._node_name)
        rospy.Subscriber(self.acc_topic, Vector3Stamped, self.accelerometer_handler_)
        rospy.Subscriber(self.mag_topic, Vector3Stamped, self.magnetometer_handler_)
        rospy.Subscriber(self.gyr_topic, Vector3Stamped, self.gyroscope_handler_)

        self._ahrs_timer = rospy.Timer(rospy.Duration(1/self.ahrs_rate), self._ahrs_timer_handler)
        self._ahrs_pub = rospy.Publisher('/ahrs/states', Float64MultiArray, queue_size=5)


    def get_param(self, param_name, default):
        try:
            param = rospy.get_param(param_name)
            rospy.logwarn("Found parameter: %s, value: %s"%(param_name, str(param)))
        except KeyError:
            param = default
            rospy.logwarn("Cannot find value for parameter: %s, assigning "	"default: %s"%(param_name, str(param)))
        return param


    def initialize_model(self):
        #Main Node Rate
        self.ahrs_rate = self.get_param('/ahrs_rate', self.ahrs_rate)     

        self.acc_topic = self.get_param('/acc_topic', self.acc_topic)
        self.mag_topic = self.get_param('/mag_topic', self.mag_topic)
        self.gyr_topic = self.get_param('/gyr_topic', self.gyr_topic)

        self.accelerometer_cov = self.get_param('/accelerometer_cov', self.accelerometer_cov)
        self.magnetometer_cov = self.get_param('/magnetometer_cov', self.magnetometer_cov)
        self.gyroscope_cov = self.get_param('/gyroscope_cov', self.gyroscope_cov)
        self.process_cov = self.get_param('/process_cov', self.process_cov)


    def accelerometer_handler_(self, msg):
        ax = msg.vector.x
        ay = msg.vector.y
        az = msg.vector.z
        a = np.array([[ax],[ay],[az]])
        
        self.ekf.correctACC(a / np.linalg.norm(a))



    def magnetometer_handler_(self, msg):
        mx = msg.vector.x
        my = msg.vector.y
        mz = msg.vector.z
        m = np.array([[mx],[my],[mz]])
        
        self.ekf.correctMAG(m / np.linalg.norm(m))



    def gyroscope_handler_(self, msg):
        gx = msg.vector.x
        gy = msg.vector.y
        gz = msg.vector.z
        self.ekf.angular_rate_input(gx, gy, gz)


    def _ahrs_timer_handler(self, event):
        xt = self.ekf.predict()
        self.states.data = [xt[0], xt[1], xt[2], xt[3], xt[4], xt[5], xt[6], xt[7], xt[8], xt[9]]
        self._ahrs_pub.publish(self.states)


if __name__ == '__main__':

    ahrs = AHRS()

    rospy.loginfo('AHRS HAS BEEN ACTIVATED!')
    
    rospy.spin()




