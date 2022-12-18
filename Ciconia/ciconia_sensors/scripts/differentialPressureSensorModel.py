#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy, os, sys, time
import math

from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64

from utils import *


class diffPressure:

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
        self.wind_velocity_var = 0.01
        self.rate = 25
        
        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'diffPressureSensor'       
        self.initialize_model()
        self.wind_velocity = Float64()
        
        #Initialization
        rospy.init_node(self._node_name)
        rospy.Subscriber("/gazebo/states/pitot/quaternion", Quaternion, self.quaternion_handler_)
        rospy.Subscriber("/gazebo/states/pitot/linear_velocity", Vector3, self.linear_velocity_handler_)
        self._wind_velocity_timer = rospy.Timer(rospy.Duration(1/self.rate), self._filtered_vel_handler)
        self._wind_velocity_pub = rospy.Publisher('/pitot/wind_velocity', Float64, queue_size=5)


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
        self.rate = self.get_param('/pitot_tube_sensor_rate', self.rate)        
        self.wind_velocity_var = self.get_param('/wind_velocity_var', self.wind_velocity_var)


    def quaternion_handler_(self, msg):
        x, y, z = quaternion_to_euler_angle(msg.x, msg.y, msg.z, msg.w)
        self.euler_angles[0,0], self.euler_angles[1,0], self.euler_angles[2,0] = x, -y, -z


    def linear_velocity_handler_(self, msg):
        self.u_ned_w =  msg.x
        self.v_ned_w = -msg.y
        self.w_ned_w = -msg.z  


    def _filtered_vel_handler(self, event):
        linear_vel = earth2body_transformation(self.euler_angles, np.array([[self.u_ned_w],[self.v_ned_w],[self.w_ned_w]]))

        Vt    = np.sqrt(linear_vel[0,0]**2 + linear_vel[1,0]**2 + linear_vel[2,0]**2)
        alpha = np.arctan(linear_vel[2,0]/linear_vel[0,0])
        beta  = np.arcsin(linear_vel[1,0]/Vt)  
        V_pitot = Vt * np.cos(alpha) * np.cos(beta)

        self.wind_velocity.data = V_pitot + np.random.normal(0,self.wind_velocity_var)
        self._wind_velocity_pub.publish(self.wind_velocity)



if __name__ == '__main__':

    pitot = diffPressure()

    rospy.loginfo('PITOT TUBE SENSOR MODEL HAS BEEN ACTIVATED!')
    
    rospy.spin()




