#! /usr/bin/python3

#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Range, FluidPressure
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

from utils import *
from alt_KF import *



class altEstimatorNode:

    def __init__(self):
    
        #Default Parameters
        self.filter_rate = 50

        self.P0 = 101325

        self.barometer_var = 0.01
        self.barometer_data = FluidPressure()
        self.barometer_topic = '/mavros/imu/static_pressure'

        self.rangefinder_var = 0.001
        self.rangefinder_data = Range()
        self.rangefinder_topic  = '/mavros/rangefinder/rangefinder'

        self.pose_topic = '/mavros/global_position/local'
        
        self.process_var = 1
        self.state_publisher_topic = '/alt_est/states'
        self.cov_publisher_topic = '/alt_est/cov'
        self.states = Float64MultiArray()
        self.cov = Float64MultiArray()

        #Global Variables
        self.phi = 0
        self.theta = 0
        self.psi = 0

        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'altEstimationNode'       
        self.initialize_model()

        #Initialization
        P = np.array([[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]])
        self.kf = ALT_State_Estimation(P, self.process_var, self.rangefinder_var, self.barometer_var)

        rospy.init_node(self._node_name)

        rospy.Subscriber(self.barometer_topic, FluidPressure, self._barometer_handler)
        rospy.Subscriber(self.rangefinder_topic, Range, self._rangefinder_handler_)        
        rospy.Subscriber(self.pose_topic, Odometry, self._pose_handler)  

        self._alt_est_states = rospy.Publisher(self.state_publisher_topic, Float64MultiArray, queue_size=5)
        self._alt_est_cov = rospy.Publisher(self.cov_publisher_topic, Float64MultiArray, queue_size=5)
        self._alt_raw_barometer = rospy.Publisher(self.state_publisher_topic, Vector3, queue_size=5)
        self._alt_raw_rangefinder = rospy.Publisher(self.state_publisher_topic, Vector3, queue_size=5)          
        
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

        self.pose_topic = self.get_param('/pose_topic', self.pose_topic)
        
        self.process_var = self.get_param('/process_var', self.process_var)

        self.barometer_var = self.get_param('/barometer_var', self.barometer_var)  

        self.P0 = self.get_param('/P0', self.P0)


    def _timer(self, event):
        xt, P = self.kf.KF_Predict(1/self.filter_rate)
        self.states.data = [xt[0,0],xt[1,0],xt[2,0]]
        self.cov.data = [P[0,0],P[0,1],P[0,2],P[1,0],P[1,1],P[1,2],P[2,0],P[2,1],P[2,2]]
        self._alt_est_states.publish(self.states)
        self._alt_est_cov.publish(self.cov)

        
    def _barometer_handler(self, msg):
        pressure = msg.fluid_pressure
        altitude = np.array([[44330 * (1 - (pressure/self.P0)**(1/5.255))]])
        self.kf.BAR_Update(altitude)
        #print('barometer altitude: '  + str(altitude))


    def _rangefinder_handler_(self, msg):
        altitude = np.array([[msg.range * np.cos(self.phi) * np.cos(self.theta)]])
        self.kf.RNG_Update(altitude)
        #print('rngfinder altitude: '  + str(altitude))


    def _pose_handler(self, msg):
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.phi, self.theta, self.psi = quaternion_to_euler_angle(qw, qx, qy, qz)


if __name__ == '__main__':

    altEst = altEstimatorNode()

    rospy.loginfo('ALTITUDE ESTIMATOR HAS BEEN ACTIVATED')
    
    rospy.spin()




