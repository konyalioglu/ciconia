#! /usr/bin/python3

#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy, os, sys, time, math
from geometry_msgs.msg import Pose, Vector3Stamped, Wrench
from sensor_msgs.msg import JointState, Imu, NavSatFix
from gazebo_msgs.msg import ModelStates, LinkStates
from std_msgs.msg import Float64MultiArray, Float32

from utils import *

from xsens_msgs.msg import sensorSample, baroSample, gnssSample
from xsens_msgs.msg import positionEstimate, velocityEstimate, orientationEstimate



class xsensModel:

    def __init__(self):


        self.phi_ned = 0
        self.theta_ned = 0
        self.psi_ned = 0

        self.u_ned = 0
        self.v_ned = 0
        self.w_ned = 0

        self.p_ned = 0
        self.q_ned = 0
        self.r_ned = 0


        #Default Parameters
        self.lattitude_ref = 45
        self.longitude_ref = 15

        self.lattitude_cov = 1
        self.longitude_cov = 1

        self.position_x_cov = 1
        self.position_y_cov = 1
        self.position_z_cov = 0.5

        self.velocity_x_cov = 0.2
        self.velocity_y_cov = 0.2
        self.velocity_z_cov = 0.1

        self.euler_x_cov = 0.01
        self.euler_y_cov = 0.01
        self.euler_z_cov = 0.01

        self.accel_x_cov = 0.01
        self.accel_y_cov = 0.01
        self.accel_z_cov = 0.01

        self.rate_x_cov = 0.01
        self.rate_y_cov = 0.01
        self.rate_z_cov = 0.01
        
        self._namespace = rospy.get_namespace()
        self._node_name = 'xsensSensorModel'
        
        self.initialize_model()


        ##Ros Initialization
        rospy.init_node(self._node_name)
        self.pub_force_moment = rospy.Publisher('/ciconia/ForcesAndMoments', Wrench, queue_size=5)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_handler)
        self._timer = rospy.Timer(rospy.Duration(0.05), self._accelerometer_handler)


    def get_param(self, param_name, default):
        try:
            param = rospy.get_param(param_name)
            rospy.logwarn("Found parameter: %s, value: %s"%(param_name, str(param)))
        except KeyError:
	        param = default
	        rospy.logwarn("Cannot find value for parameter: %s, assigning "
			"default: %s"%(param_name, str(param)))
        return param


    def initialize_model(self):
        #
        self.lattitude_ref = self.get_param('/lattitude_ref', self.lattitude_ref)
        self.longitude_ref = self.get_param('/longitude_ref', self.longitude_ref)
        
        #
        self.lattitude_cov = self.get_param('/lattitude_cov', self.lattitude_cov)
        self.longitude_cov = self.get_param('/longitude_cov', self.longitude_cov)


        # Position Config
        self.position_x_cov = self.get_param('/position_x_cov', self.position_x_cov)
        self.position_y_cov = self.get_param('/position_y_cov', self.position_y_cov)
        self.position_z_cov = self.get_param('/position_z_cov', self.position_z_cov)
        
        # Velocity Config
        self.velocity_x_cov = self.get_param('/velocity_x_cov', self.velocity_x_cov)
        self.velocity_y_cov = self.get_param('/velocity_y_cov', self.velocity_y_cov)
        self.velocity_z_cov = self.get_param('/velocity_z_cov', self.velocity_z_cov)

        # Accel Config
        self.accel_x_cov = self.get_param('/accel_x_cov', self.accel_x_cov)
        self.accel_y_cov = self.get_param('/accel_y_cov', self.accel_y_cov)
        self.accel_z_cov = self.get_param('/accel_z_cov', self.accel_z_cov)

        # Gyro Config
        self.rate_x_cov = self.get_param('/rate_x_cov', self.rate_x_cov)
        self.rate_y_cov = self.get_param('/rate_y_cov', self.rate_y_cov)
        self.rate_z_cov = self.get_param('/rate_z_cov', self.rate_z_cov)



    def pose_handler(self, data):

        body_index = data.name.index('ciconia::body')

        r_body = data.pose[body_index].position

        orientation = data.pose[body_index].orientation
        tmp_phi, tmp_theta, tmp_psi = quaternion_to_euler_angle(orientation.w, orientation.x, orientation.y, orientation.z)
        self.phi_ned = tmp_phi
        self.theta_ned = -tmp_theta
        self.psi_ned = -tmp_psi

        ori_vector_ned = np.array([[self.phi_ned], [self.theta_ned], [self.psi_ned]])
        vel_vector_ned = np.array([[data.twist[body_index].linear.x],[-data.twist[body_index].linear.y],[-data.twist[body_index].linear.z]])
        vel_vector = earth2body_transformation(ori_vector, vel_vector)

        self.u_ned = vel_vector[0]
        self.v_ned = -vel_vector[1]
        self.w_ned = -vel_vector[2]

        rates = np.array([[data.twist[body_index].angular.x],[data.twist[body_index].angular.y],[data.twist[body_index].angular.z]])
        rates = earth2body_transformation(ori_vector, rates)

        self.p_ned = rates[0,0]
        self.q_ned = -rates[1,0]
        self.r_ned = -rates[2,0]

        #wind axis parameters, Wind Velocity, Angle of Attack, Side slip angle
        self.Vt    = math.sqrt(self.u_ned**2 + self.v_ned**2 + self.w_ned**2)
        self.alpha = math.atan(self.w_ned/self.u_ned)
        self.beta  = math.asin(self.v_ned/self.Vt)

        self.mach = self.Vt / self.speedofsound
        return
        
        
    def _accelerometer_handler(self, data):
        return


    def loop(self):
        r = rospy.Rate(250)
        rospy.loginfo('XSENS SENSOR MODEL HAD BEEN ACTIVATED')
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':

    vtol = xsensModel()

    vtol.loop()




