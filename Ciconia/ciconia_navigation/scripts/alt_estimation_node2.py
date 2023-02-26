#! /usr/bin/python3

#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range, FluidPressure, Imu
from std_msgs.msg import Float64MultiArray, Float64

from utils import *
from alt_KF2 import *



class altEstimatorNode:

    def __init__(self):
    
        #Default Parameters
        self.filter_rate = 50

        self.P0 = 101325

        self.y_trim_offset = 0.05645018

        self.barometer_var = 0.01
        self.barometer_data = Float64()
        self.barometer_topic = '/mavros/imu/static_pressure'

        self.rangefinder_var = 0.001
        self.rangefinder_data = Float64()
        self.rangefinder_topic  = '/mavros/rangefinder/rangefinder'

        self.accelerometer_var = 0.1
        self.imu_data = Imu()
        self.imu_topic = '/mavros/imu/data'
        
        self.process_var = 1
        self.state_publisher_topic = '/alt_est/states'
        self.cov_publisher_topic = '/alt_est/cov'
        self.imu_publisher_topic = '/imu'
        self.barometer_publisher_topic = '/barometer'
        self.rangefinder_publisher_topic = '/rangefinder'

        self.states = Float64MultiArray()
        self.cov = Float64MultiArray()

        self.number_of_acc_samples = 200
        self.acc_sample_counter = 0
        self.sum_acc = 0
        self.gravity_calibration = False

        #Global Variables
        self.phi = 0
        self.theta = 0
        self.psi = 0

        ##Ros
        self._namespace = rospy.get_namespace()
        self._node_name = 'altEstimationNode'
        self.initialize_model()

        #Initialization
        P = np.array([[1.0,0.0,0.0,0.0,0.0],[0.0,1.0,0.0,0.0,0.0],[0.0,0.0,1.0,0.0,0.0],[0.0,0.0,0.0,1.0,0.0],[0.0,0.0,0.0,0.0,1.0]])
        self.kf = ALT_State_Estimation(P, self.process_var, self.rangefinder_var, self.barometer_var, self.accelerometer_var)

        rospy.init_node(self._node_name)

        rospy.Subscriber(self.barometer_topic, FluidPressure, self._barometer_handler)
        rospy.Subscriber(self.rangefinder_topic, Range, self._rangefinder_handler_)
        rospy.Subscriber(self.imu_topic, Imu, self._imu_handler)


        self._alt_est_states = rospy.Publisher(self.state_publisher_topic, Float64MultiArray, queue_size=5)
        self._alt_est_cov = rospy.Publisher(self.cov_publisher_topic, Float64MultiArray, queue_size=5)
        self._alt_raw_barometer = rospy.Publisher(self.barometer_publisher_topic, Float64, queue_size=5)
        self._alt_raw_rangefinder = rospy.Publisher(self.rangefinder_publisher_topic, Float64, queue_size=5)   
        self._imu_publisher = rospy.Publisher(self.imu_publisher_topic, Imu, queue_size=5)

        self._timer = rospy.Timer(rospy.Duration(1/self.filter_rate), self._timer)



    def get_param(self, param_name, default):
        try:
            param = rospy.get_param(param_name)
            rospy.logwarn("Found parameter: %s, value: %s"%(param_name, str(param)))
        except KeyError:
            param = default
            rospy.logwarn("Cannot find value for parameter: %s, assigning "	"default: %s"%(param_name, str(param)))
        return param


    def initialize_model(self):
        self.filter_rate = self.get_param('/filter_rate', self.filter_rate)       

        self.barometer_topic = self.get_param('/barometer_topic', self.barometer_topic)

        self.rangefinder_var = self.get_param('/rangefinder_var', self.rangefinder_var)

        self.rangefinder_topic = self.get_param('/rangefinder_topic', self.rangefinder_topic)

        self.state_publisher_topic = self.get_param('/state_publisher_topic', self.state_publisher_topic)

        self.cov_publisher_topic = self.get_param('/cov_publisher_topic', self.cov_publisher_topic)

        self.imu_publisher_topic = self.get_param('/imu_publisher_topic', self.imu_publisher_topic)

        self.imu_topic = self.get_param('/imu_topic', self.imu_topic)

        self.process_var = self.get_param('/process_var', self.process_var)

        self.barometer_var = self.get_param('/barometer_var', self.barometer_var)  

        self.accelerometer_var = self.get_param('/accelerometer_var', self.accelerometer_var)

        self.y_trim_offset = self.get_param('/y_trim_offset', self.y_trim_offset)    

        self.P0 = self.get_param('/P0', self.P0)


    def _timer(self, event):
        xt, P = self.kf.KF_Predict(1/self.filter_rate)
        self.states.data = [xt[0,0],xt[1,0],xt[2,0],xt[3,0],xt[4,0]]
        self.cov.data = [P[0,0],P[1,1],P[2,2]]
        self._alt_est_states.publish(self.states)
        self._alt_est_cov.publish(self.cov)


    def _barometer_handler(self, msg):
        pressure = msg.fluid_pressure
        altitude = np.array([[44330 * (1 - (pressure/self.P0)**(1/5.255))]])
        self.kf.BAR_Update(altitude)
        self.barometer_data.data = altitude
        self._alt_raw_barometer.publish(self.barometer_data)


    def _rangefinder_handler_(self, msg):
        altitude = np.array([[msg.range * np.cos(self.phi) * np.cos(self.theta)]])
        self.kf.RNG_Update(altitude)
        self.rangefinder_data.data = altitude
        self._alt_raw_rangefinder.publish(self.rangefinder_data)


    def _imu_handler(self, msg):
        if self.acc_sample_counter != self.number_of_acc_samples and self.gravity_calibration == False:

            if np.abs(msg.linear_acceleration.x) > 0.001 or np.abs(msg.linear_acceleration.y) > 0.001 or np.abs(msg.linear_acceleration.z) > 0.001:
                self.acc_sample_counter += 1
                self.sum_acc += np.sqrt(msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 + msg.linear_acceleration.z ** 2)
                g = np.sqrt(msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 + msg.linear_acceleration.z ** 2)
                

        elif self.acc_sample_counter  == self.number_of_acc_samples and self.gravity_calibration == False:
            self.g = self.sum_acc / (self.number_of_acc_samples)
            self.acc_sample_counter += 1
            self.gravity_calibration = True

        else:
            qx =  msg.orientation.x
            qy =  msg.orientation.y
            qz =  msg.orientation.z
            qw =  msg.orientation.w
            self.phi, self.theta, self.psi = quaternion_to_euler_angle(qw, qx, qy, qz)

            self.theta = -self.theta
            self.psi   = -self.psi

            gx = -self.g * np.sin(self.theta)
            gy =  self.g * np.cos(self.theta) * np.sin(self.phi)
            gz =  self.g * np.cos(self.theta) * np.cos(self.phi)

            accel_vector = euler_inverse_transformation_y(self.y_trim_offset, np.array([[-msg.linear_acceleration.x],[msg.linear_acceleration.y],[msg.linear_acceleration.z]]))

            self.ax = accel_vector[0,0] - gx
            self.ay = accel_vector[1,0] - gy
            self.az = accel_vector[2,0] - gz


            self.p =  msg.angular_velocity.x
            self.q = -msg.angular_velocity.y
            self.r = -msg.angular_velocity.z

            self.kf.ACC_Update(self.az)

            self.imu_data = msg
            self.imu_data.linear_acceleration.x = self.ax
            self.imu_data.linear_acceleration.y = self.ay
            self.imu_data.linear_acceleration.z = self.az

            self._imu_publisher.publish(self.imu_data)





if __name__ == '__main__':

    altEst = altEstimatorNode()

    rospy.loginfo('ALTITUDE ESTIMATOR HAS BEEN ACTIVATED')

    rospy.spin()




