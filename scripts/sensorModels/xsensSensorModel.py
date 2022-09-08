#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy, os, sys, time, math
import math

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
        self.rate = 200
        
        self.gps_rate = 3
        self.lattitude_ref = 45
        self.longitude_ref = 15
        self.lattitude_cov = 1
        self.longitude_cov = 1

        self.position_x_cov = 1
        self.position_y_cov = 1
        self.position_z_cov = 0.5
        self.pos_est_rate   = 10

        self.velocity_x_cov = 0.2
        self.velocity_y_cov = 0.2
        self.velocity_z_cov = 0.1
        self.vel_est_rate   = 10


        self.filtered_euler_x_cov = 0.01
        self.filtered_euler_y_cov = 0.01
        self.filtered_euler_z_cov = 0.01
        self.filtered_accel_x_cov = 0.01
        self.filtered_accel_y_cov = 0.01
        self.filtered_accel_z_cov = 0.01
        self.filtered_gyro_x_cov  = 0.01
        self.filtered_gyro_y_cov  = 0.01
        self.filtered_gyro_z_cov  = 0.01
        self.attitude_filter_rate = 100

        self.accel_x_cov = 0.01
        self.accel_y_cov = 0.01
        self.accel_z_cov = 0.01
        self.accel_rate  = 200

        self.gyro_x_cov = 0.01
        self.gyro_y_cov = 0.01
        self.gyro_z_cov = 0.01
        self.gyro_rate = 200
        
        # Magnetometer Config
        self.mag_x_cov = 0.01
        self.mag_y_cov = 0.01
        self.mag_z_cov = 0.01
        self.magnetometer_raw_rate = 20

        # Barometer Config
        self.pressure_cov = 0.01
        self.barometer_raw_rate = 20
        
        self._namespace = rospy.get_namespace()
        self._node_name = 'xsensSensorModel'       
        self.initialize_model()


        ##Ros Initialization
        rospy.init_node(self._node_name)

        rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_handler)
        
        self._accelerometer_time = rospy.Timer(rospy.Duration(1/self.accel_rate), self._accelerometer_handler)
        self._gyroscopte_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._gyroscope_handler)
        self._magnetometer_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._magnetometer_handler)
        self._barometer_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._barometer_handler)
        self._gps_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._gps_handler)
        
        self._filtered_vel_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._filtered_vel_handler)
        self._filtered_pos_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._filtered_pos_handler)
        self._filtered_attitude_timer = rospy.Timer(rospy.Duration(1/self.accel_rate), self._filtered_attitude_handler)
        
        self._accelerometer_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        self._gyroscopte_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        self._magnetometer_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        self._barometer_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        self._gps_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        
        self._filtered_vel_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        self._filtered_pos_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)
        self._filtered_attitude_pub = rospy.Publisher(self._node_name + '/ForcesAndMoments', Wrench, queue_size=5)




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
        #Main Node Rate
        self.rate = self.get_param('/x_sens_node_rate', self.rate)       
        
        # GPS Config
        self.lattitude_ref = self.get_param('/lattitude_ref', self.lattitude_ref)
        self.longitude_ref = self.get_param('/longitude_ref', self.longitude_ref)
        self.lattitude_cov = self.get_param('/lattitude_cov', self.lattitude_cov)
        self.longitude_cov = self.get_param('/longitude_cov', self.longitude_cov)       
        self.gps_rate = self.get_param('/gps_rate', self.gps_rate)

        # Filtered Position Config
        self.position_x_cov = self.get_param('/position_x_cov', self.position_x_cov)
        self.position_y_cov = self.get_param('/position_y_cov', self.position_y_cov)
        self.position_z_cov = self.get_param('/position_z_cov', self.position_z_cov)
        self.pos_est_rate = self.get_param('/position_z_cov', self.pos_est_rate)
             
        # Filtered Velocity Config
        self.velocity_x_cov = self.get_param('/velocity_x_cov', self.velocity_x_cov)
        self.velocity_y_cov = self.get_param('/velocity_y_cov', self.velocity_y_cov)
        self.velocity_z_cov = self.get_param('/velocity_z_cov', self.velocity_z_cov)
        self.vel_est_rate   = self.get_param('/velocity_rate', self.vel_est_rate)

        # Filtered Attitude Data
        self.filtered_euler_x_cov = self.get_param('/filtered_euler_x_cov', self.filtered_euler_x_cov)
        self.filtered_euler_y_cov = self.get_param('/filtered_euler_y_cov', self.filtered_euler_y_cov)
        self.filtered_euler_z_cov = self.get_param('/filtered_euler_z_cov', self.filtered_euler_z_cov)
        self.filtered_accel_x_cov = self.get_param('/filtered_accel_z_cov', self.filtered_accel_x_cov)
        self.filtered_accel_y_cov = self.get_param('/filtered_accel_y_cov', self.filtered_accel_y_cov)
        self.filtered_accel_z_cov = self.get_param('/filtered_accel_z_cov', self.filtered_accel_z_cov)
        self.filtered_gyro_x_cov  = self.get_param('/filtered_gyro_x_cov', self.filtered_gyro_x_cov)
        self.filtered_gyro_y_cov  = self.get_param('/filtered_gyro_y_cov', self.filtered_gyro_y_cov)
        self.filtered_gyro_z_cov  = self.get_param('/filtered_gyro_z_cov', self.filtered_gyro_z_cov)
        self.attitude_filter_rate = self.get_param('/attitude_filter_rate', self.attitude_filter_rate)

        # Accel Config
        self.accel_x_cov = self.get_param('/accel_x_cov', self.accel_x_cov)
        self.accel_y_cov = self.get_param('/accel_y_cov', self.accel_y_cov)
        self.accel_z_cov = self.get_param('/accel_z_cov', self.accel_z_cov)
        self.accel_rate = self.get_param('/accelerometer_raw_rate', self.accel_rate)

        # Gyro Config
        self.gyro_x_cov = self.get_param('/gyro_x_cov', self.gyro_x_cov)
        self.gyro_y_cov = self.get_param('/gyro_y_cov', self.gyro_y_cov)
        self.gyro_z_cov = self.get_param('/gyro_z_cov', self.gyro_z_cov)
        self.gyro_rate = self.get_param('/gyroscope_raw_rate', self.gyro_rate)
        
        # Magnetometer Config
        self.mag_x_cov = self.get_param('/mag_x_cov', self.mag_x_cov)
        self.mag_y_cov = self.get_param('/mag_y_cov', self.mag_y_cov)
        self.mag_z_cov = self.get_param('/mag_z_cov', self.mag_z_cov)
        self.magnetometer_rate = self.get_param('/accelerometer_raw_rate', self.accel_rate)
        
        # Barometer Config
        self.pressure_cov = self.get_param('/pressure_cov', self.pressure_cov)
        self.magnetometer_rate = self.get_param('/accelerometer_raw_rate', self.accel_rate)



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
        vel_vector = earth2body_transformation(ori_vector_ned, vel_vector_ned)

        self.u_ned = vel_vector[0]
        self.v_ned = vel_vector[1]
        self.w_ned = vel_vector[2]

        euler_rates = np.array([[data.twist[body_index].angular.x],[-data.twist[body_index].angular.y],[-data.twist[body_index].angular.z]])
        rates = earth2body_transformation(ori_vector_ned, euler_rates)

        self.p_ned = rates[0,0]
        self.q_ned = rates[1,0]
        self.r_ned = rates[2,0]

        return
        
        
    def _accelerometer_handler(self):
        return
        
        
    def _gyroscope_handler(self):
        return
        
        
    def _magnetometer_handler(self):
        return
        
        
    def _gps_handler(self):
        return
        
        
    def _filtered_vel_handler(self):
        return
        
        
    def _filtered_pos_handler(self):
        return
        
        
    def _filtered_attitude_handler(self):
        return
        
        
    def _barometer_handler(self):
        return


    def loop(self):
        r = rospy.Rate(self.rate)
        rospy.loginfo('XSENS SENSOR MODEL HAD BEEN ACTIVATED')
        while not rospy.is_shutdown():
            r.sleep()



if __name__ == '__main__':

    vtol = xsensModel()
    rospy.loginfo('XSENS SENSOR MODEL HAD BEEN ACTIVATED')
    
    rospy.spin()




