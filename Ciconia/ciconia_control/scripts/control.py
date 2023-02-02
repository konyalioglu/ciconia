#! /usr/bin/python3


import numpy as np
import rospy, math, time
from geometry_msgs.msg import Pose, Vector3Stamped
from sensor_msgs.msg import JointState, Imu, NavSatFix, Image
from gazebo_msgs.msg import ModelStates, LinkStates
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, Float32, String 
from ciconia_msgs.msg import quadrotorControl, transitionControl, flightControl, estimatedStates

from model_predictive_controller import Model_Predictive_Control
from lqr import LinearQuadraticRegulator
import quadprog
from numpy import savetxt

import scipy.linalg
from scipy import sparse
import osqp
from scipy import signal
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
#from processImage import readImage


class controller:


    def __init__(self):
        self.phi=0
        self.theta=0
        self.psi=0
        self.sim_time = 0
        self.robot_name = 'ciconia'

        self.phi_ned = 0
        self.theta_ned = 0
        self.psi_ned = 0

        self.u = 0
        self.v = 0
        self.w = 0

        self.u_ned = 0
        self.v_ned = 0
        self.w_ned = 0

        self.p = 0
        self.q = 0
        self.r = 0

        self.p_ned = 0
        self.q_ned = 0
        self.r_ned = 0

        self.dt = 0.01

        self.kp_roll = 15
        self.ki_roll = 2
        self.kd_roll = 5
        self.kp_pitch = 10
        self.ki_pitch = 2
        self.kd_pitch = 8
        self.kp_yaw = 20
        self.ki_yaw = 0.0
        self.kd_yaw = 0.0

        self.roll  = 0
        self.pitch = 0
        self.yaw   = 0

        self.roll_rate = 0
        self.pitch_rate = 0
        self.yaw_rate = 0

        self.linear_acceleration_x = 0
        self.linear_acceleration_y = 0
        self.linear_acceleration_z = 0

        self.setpoint_roll = 0.0
        self.setpoint_pitch = 0.0
        self.setpoint_yaw = 0.0

        self.nominal_vel = 50

        self.error_pitch = 0
        self.error_roll = 0
        self.error_yaw = 0
        self.prev_time = 0
        self.current_time = 0
        self.error_sum_roll = 0
        self.error_sum_pitch = 0
        self.error_sum_yaw = 0
        self.prev_error_roll = 0
        self.prev_error_pitch = 0
        self.prev_error_yaw = 0
        self.error_sum_yaw = 0
        self.error_rate_roll = 0
        self.error_rate_pitch = 0
        self.error_rate_yaw = 0
        self.output_roll = 0
        self.output_pitch = 0
        self.output_yaw = 0

        self.flag = 0

        self.alt_flag = 0
        self.prev_alt_time = 0
        self.current_alt_time = 0
        self.kp_alt = 5
        self.ki_alt = 0.0
        self.kd_alt = 2.0

        self.alt_error_sum = 0
        self.prev_alt_error = 0

        self.gps_flag  = 0
        self.init_long = 0
        self.init_lat  = 0
        self.latitude  = 0
        self.longitude = 0

        self.x = 0
        self.y = 0
        self.z = 0

        self.x_vel_global = 0
        self.y_vel_global = 0
        self.z_vel_global = 0

        self.vel_flag = 0
        self.prev_vel_time = 0
        self.current_vel_time = 0
        self.kp_vel_x = 0.3
        self.ki_vel_x = 0.05
        self.kd_vel_x = 0.00
        self.kp_vel_y = 0.3
        self.ki_vel_y = 0.05
        self.kd_vel_y = 0.00
        self.kp_vel_z = 20.0
        self.ki_vel_z = 2.0
        self.kd_vel_z = 1.0

        self.vel_x_error_sum = 0
        self.vel_y_error_sum = 0
        self.vel_z_error_sum = 0
        self.prev_vel_x_error = 0
        self.prev_vel_y_error = 0
        self.prev_vel_z_error = 0

        self.pos_flag = 0
        self.prev_pos_time = 0
        self.current_pos_time = 0
        self.kp_pos_x = 0.2
        self.ki_pos_x = 0.0
        self.kd_pos_x = 0.2
        self.kp_pos_y = 0.2
        self.ki_pos_y = 0.0
        self.kd_pos_y = 0.2
        self.kp_pos_z = 0.35
        self.ki_pos_z = 0.0
        self.kd_pos_z = 0.0

        self.pos_x_error_sum = 0
        self.pos_y_error_sum = 0
        self.pos_z_error_sum = 0
        self.prev_pos_x_error = 0
        self.prev_pos_y_error = 0
        self.prev_pos_z_error = 0

        ##safety limits
        #motor
        self.max_motor_vel_1 = 100
        self.max_motor_vel_2 = 100
        self.max_motor_vel_3 = 100
        self.max_motor_vel_4 = 100

        #attitude control
        self.max_pitch = math.radians(10)
        self.max_roll  = math.radians(10)

        #velocity limit
        self.max_vel_x = 8
        self.max_vel_y = 8
        self.max_vel_z = 3

        self.thrust_coefficient = 0.0245
        self.thrust_coefficientFW = 0.102

        self.mass = 25
        self.gravity_acc = 9.8065

        self.sref = 1.55
        self.cbar = 0.385
        self.bref = 4.4
        self.density = 1.225

        self.tr_cl_trim = 0.695
        self.CL_tr = 0.669
        self.CLa_tr = 6.079
        self.Clq_tr = 15.858
        self.Clde = 0.69952
        self.CD0_tr = 0.0288
        self.CDa_tr = 0.4688
        self.CDq_tr = 0.26
        self.Cm0_tr = 0.1091 
        self.Cma_tr = -3.4657
        self.Cmq_tr = -49.4246

        self.trim_states_transition = np.array([[9],[0],[0],[0],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
        self.trim_states_flight = np.array([[20],[0],[0],[-0.019],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
        self.state_store = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0],[0]],dtype='float32').reshape(1,13)
        self.input_store = np.array([[0],[0],[0],[0],[0],[0],[0],[0],[0]],dtype='float32').reshape(1,9)
        
        self.a = 635
        self.b = 530
        
        self.transition_bool = False
        self.transition_counter = 0
        
        self.backtransition_bool = False
        self.backtransition_counter = 0
        
        ##Object Definitions
        self.motor_input = Float64MultiArray()
        self.control_surface_input = Float64MultiArray()
        self.message = String()
        self.bridge = CvBridge()
        #self.markerDetection = readImage(5, 10)

        ##Ros Initialization
        rospy.init_node('control')
        self.pub_motor = rospy.Publisher('/ciconia/vel_cmd', Float64MultiArray, queue_size=4)
        self.pub_control_surface = rospy.Publisher('/ciconia/joint_controlDeflection_controller/command', Float64MultiArray, queue_size=4)

        #rospy.Subscriber("/ciconia/imu", Imu, self.imu_data_handler)
        #rospy.Subscriber("/xsens/filter/positionlla", Vector3Stamped, self.gps_pos_data_handler)
        #rospy.Subscriber("/xsens/filter/velocity", Vector3Stamped, self.gps_vel_data_handler)

        rospy.Subscriber("/ciconia/filter/states", estimatedStates, self._estimator_handler)
        rospy.Subscriber("/clock", Clock, self.sim_time_handler)


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


    def rotation2D(self, x, y):

        X = np.matrix([[x],[y]])
        R = np.matrix([[np.cos(self.yaw), -np.sin(self.yaw)],
                        [np.sin(self.yaw), np.cos(self.yaw)]])

        result = R * X
        xr = result[0,0]
        yr = result[1,0]

        return float(xr), float(yr)


    def inverse_rotation2D(self, x, y):

        X = np.matrix([[x],[y]])
        R = np.matrix([[np.cos(self.yaw), -np.sin(self.yaw)],
                        [np.sin(self.yaw), np.cos(self.yaw)]])

        tR = np.matrix.transpose(R)
        result = tR * X
        xr = result[0,0]
        yr = result[1,0]

        return float(xr), float(yr)


    def inverse_rotation3D(self, x, y, z):

        X = np.matrix([[x],[y],[z]])

        croll  = np.cos(self.roll)
        sroll  = np.sin(self.roll)
        cpitch = np.cos(self.pitch)
        spitch = np.sin(self.pitch)
        cyaw   = np.cos(self.yaw)
        syaw   = np.sin(self.yaw)

        '''
        R = np.matrix([[cyaw * cpitch, cyaw * spitch * sroll - syaw * croll, cyaw * spitch * croll + syaw * sroll],
                        [syaw * cpitch, syaw * spitch * sroll + cyaw * croll, syaw * spitch * croll - cyaw * sroll],
                        [-spitch, cpitch * sroll, cpitch * croll]])'''

        R = np.matrix([[cyaw, -syaw, 0],
                        [syaw, cyaw, 0],
                        [0, 0, 1]])

        tR = np.matrix.transpose(R)
        result = tR * X

        xr = result[0,0]
        yr = result[1,0]
        zr = result[2,0]

        return float(xr), float(yr), float(zr)


    def rotation3D(self, x, y, z):

        X = np.matrix([[x],[y],[z]])

        croll  = math.cos(self.roll)
        sroll  = math.sin(self.roll)
        cpitch = math.cos(self.pitch)
        spitch = math.sin(self.pitch)
        syaw   = math.sin(self.yaw)
        cyaw   = math.cos(self.yaw)

        '''
        R = np.matrix([[cyaw * cpitch, cyaw * spitch * sroll - syaw * croll, cyaw * spitch * croll],
                        [syaw * cpitch, syaw * spitch * sroll + cyaw * croll, syaw * spitch * croll - cyaw * sroll],
                        [-spitch, cpitch * sroll, cpitch * croll]])'''

        R = np.matrix([[cyaw, -syaw, 0],
                        [syaw, cyaw, 0],
                        [0, 0, 1]])

        result = R * X

        xr = result[0,0]
        yr = result[1,0]
        zr = result[2,0]

        return float(xr), float(yr), float(zr)


    def _estimator_handler(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.u = msg.velocity.x
        self.v = msg.velocity.y
        self.w = msg.velocity.z
        self.p = msg.angular_rate.x
        self.q = msg.angular_rate.y
        self.r = msg.angular_rate.z
        self.phi = msg.euler_angle.x
        self.theta = msg.euler_angle.y
        self.psi = msg.euler_angle.z


    def gps_pos_data_handler(self, msg):

        r_earth = 6371000
        self.z = -msg.altitude

        if self.gps_flag == 0:
            self.init_lat  = msg.latitude - 49.89999999974
            self.init_long = msg.longitude - 8.90000000309
            self.gps_flag = 1
        else:
            self.latitude  = msg.latitude - 49.89999999974
            self.longitude = msg.longitude - 8.90000000309

            a  = 6378137.0
            b  = 6356752.3

            self.x = a * math.sin(math.radians(self.latitude))
            self.y = b * math.sin(math.radians(self.longitude)) * math.cos(math.radians(49.89999999974))

        return


    def body2earth_transformation(self, angles, vector):
        rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]])
        roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]])
        rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
        return np.matrix.transpose(rotx @ roty @ rotz) @ vector


    def earth2body_transformation(self, angles, vector):
        rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]])
        roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]])
        rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
        return rotx @ roty @ rotz @ vector


    def gazebo2earth_transformation(self, psi, vector):
        rotz = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
        return  rotz @ vector


    def earth2gazebo_transformation(self, psi, vector):
        rotz = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
        return  rotz.T @ vector
    
    
    def body2earth_rate_transformation(self, angles, rates):
        vec1 = np.array([[1, \
                          np.tan(angles[1,0]) * np.sin(angles[0,0]), \
                          np.cos(angles[0,0]) * np.tan(angles[1,0])]])
    
        vec2 = np.array([[0, \
                          np.cos(angles[0,0]), \
                          -np.sin(angles[0,0])]])
    
        vec3 = np.array([[0, \
                          1/np.cos(angles[1,0]) * np.sin(angles[0,0]), \
                          1/np.cos(angles[1,0]) * np.cos(angles[0,0])]])
    
        transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)
        
        vec = transformation_matrix @ rates
        
        phi_dot = vec[0,0]
        theta_dot = vec[1,0]
        psi_dot = vec[2,0]
        
        return phi_dot, theta_dot, psi_dot
    
    
    def euler_rate2body_rate(self, angles, rates):
        vec1 = np.array([[1, 0,  -np.sin(angles[1,0])]])
    
        vec2 = np.array([[0, np.cos(angles[0,0]), np.sin(angles[0,0]) * np.cos(angles[1,0])]])
    
        vec3 = np.array([[0, -np.sin(angles[0,0]), np.cos(angles[0,0]) * np.cos(angles[1,0])]])
    
        transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)
        
        vec = transformation_matrix @ rates
        
        p = vec[0,0]
        q = vec[1,0]
        r = vec[2,0]
        
        return np.array([[p],[q],[r]])


    def sim_time_handler(self, data):
        seconds = data.clock.secs
        nano_seconds = data.clock.nsecs

        self.sim_time = seconds + nano_seconds / 1000000000
        return


    def lidar_data_handler(self, msg):
        return 0


    def get_stored_data(self):
        return self.state_store, self.input_store


    def get_name(self):
        return self.robot_name 
               
        
    def get_transition_dynamics(self, option='full'):
    
        self.tr_trimmed_states = np.array([[10],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]).reshape(10,1)
        self.tr_trimmed_control_inputs = np.array([[0],[2.776],[181.6],[-4.03348834],[0],[0],[0],[0]]).reshape(8,1)
        
        A_tran = np.array([[-0.016454,	0.187203,	0,	-9.8065,	0,	0.0046127,	0,	-1.62038e-06,	0,	0],
                            [-0.508427,	-2.31099,	8.82873,	0,	0,	-0.0808179,	1.62038e-06,	0,	0,	0],
                            [0.0881203,	-1.40677,	-3.93851,	0,	0,	0.00708723,	-7.60669e-08,	-1.42194e-07,	0,	0],
                            [0,	0,	1,	0,	0,	0,	0,	0,	1.22792e-07,	0],
                            [0,	-1,	0,	10,	0,	0,	0,	0,	1.62038e-06,	0],
                            [1.3613e-07,	-9.76549e-08,	0,	0,	0,	-0.0942647,	-0.00964732,	-9.83297,	9.8065,	0],
                            [8.6732e-09,	-1.12744e-07,	9.48618e-08,	0,	0,	0.273087,	-6.28216,	1.77522,	0,	0],
                            [-2.77507e-08,	8.23678e-09,	1.95739e-08,	0,	0,	0.267054,	-0.990437,	-0.2658,	0,	0],
                            [0,	0,	0,	-1.22792e-07,	0,	0,	1,	0,	0,	0],
                            [0,	0,	0,	0,	0,	0,	0,	1,	-5.84652e-05,	0]])
                         
        B_tran = np.array([[0,	0.04,	0,	0,	0,	0,	0,	0],
                            [-2.65643,	0,	-0.04,	0,	0,	0,	0,	0],
                            [-4.7999,	0,	0,	0.110939,	0,	0,	0,	0],
                            [0,	0,	0,	0,	0,	0,	0,	0],
                            [0,	0,	0,	0,	0,	0,	0,	0],
                            [0,	0,	0,	0,	0,	-0.700145,	0,	0],
                            [0,	0,	0,	0,	-10.6816,	0.196707,	0.117137,	0.00909773],
                            [0,	0,	0,	0,	-0.909796,	1.87101,	0.00909773,	0.0604509],
                            [0,	0,	0,	0,	0,	0,	0,	0],
                            [0,	0,	0,	0,	0,	0,	0,	0]])

        C_tran = np.eye(A_tran.shape[0])
        D_tran = np.zeros((C_tran.shape[0], B_tran.shape[1]))  
        if option == 'full':     
            return A_tran, B_tran, C_tran, D_tran
        if option == 'longitudinal':     
                return A_tran[:5,:5], B_tran[:5,:4], np.eye(5), np.zeros((5,4))
        if option == 'lateral':     
                return A_tran[5:,5:], B_tran[5:,4:], np.eye(5), np.zeros((5,4))
        if option == 'longitudinal_attitude':     
                return A_tran[:4,:4],B_tran[:4,:2], np.eye(4), np.zeros((4,2))
        if option == 'lateral_attitude':     
                return A_tran[5:9,5:9], B_tran[5:9,2:], np.eye(4), np.zeros((4,2))
    
    
    def get_flight_dynamics(self, option='full'):
        
        self.ff_trimmed_states = np.array([[18],[0.3468],[0],[1.9266e-02],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
        self.ff_trimmed_control_inputs = np.array([[3.57596876e-02],[4.0],[0],[0]]).reshape(4,1)
        
        self.ff_trimmed_states = np.array([[18],[0.281],[0],[1.567e-02],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
        self.ff_trimmed_control_inputs = np.array([[4.638e-02],[9.0],[0],[0]]).reshape(4,1)
        
        A_flight = np.array([[  -0.0249373,	0.434151,	-0.306821,	-9.80468,	0,	0,	0,	0,	0,	0],
                             [   -1.0119,	-4.03576,	15.9232,	-0.188924,	0,	0.0,	0,	0,	0,	0],
                             [   0.0480231,	-2.5199,	-6.97358,	0,	0,	0,	0,	0,	0,	0],
                             [   0,	0,	1,	0,	0,	0,	0,	0,	0,	0],
                             [   0.0192652,	-0.999814,	0,	18,	0,	0,	0,	0,	-1.49012e-07,	0],
                             [   1.35817e-09,	-3.16569e-09,	0,	0,	0,	-0.172437,	0.329692,	-17.7009,	9.80468,	0],
                             [   -4.43797e-09,	9.09345e-09,	5.28334e-11,	0,	0,	0.505768,	-11.1232,	3.14313,	0,	0],
                             [   -3.85613e-09,	-2.59574e-09,	1.09466e-11,	0,	0,	0.465515,	-1.75368,	-0.470645,	0,	0],
                             [   0,	0,	0,	-6.86224e-11,	0,	0,	1,	0.0192688,	1.24375e-12,	0],
                             [   0,	0,	0,	-1.32203e-12,	0,	0,	0,	1.00019,	6.45594e-11,	0]]) 
        B_flight = np.array([[  0.163105,	0.04,	0,	0],
                             [  -8.46473,	0,	0,	0],
                             [  -15.2978,	0,	0,	0],
                             [  0,	0,	0,	0],
                             [  0,	0,	0,	0],
                             [  0,	0,	0,	-2.23143],
                             [  0,	0,	-34.0432,	0.626925],
                             [  0,	0,	-2.89961,	5.96312],
                             [  0,	0,	0,	0],
                             [  0,	0,	0,	0]])
        C_flight = np.eye(A_flight.shape[0])
        D_flight = np.zeros((C_flight.shape[0], B_flight.shape[1]))  
        if option == 'full':     
            return A_flight, B_flight, C_flight, D_flight
        if option == 'longitudinal':     
            return A_flight[:5,:5], B_flight[:5,:2], np.eye(5), np.zeros((5,2))
        if option == 'lateral':     
            return A_flight[5:,5:], B_flight[5:,2:], np.eye(5), np.zeros((5,2))
        if option == 'longitudinal_attitude':     
            return A_flight[:4,:4],B_flight[:4,:2], np.eye(4), np.zeros((4,2))
        if option == 'lateral_attitude':     
            return A_flight[5:9,5:9], B_flight[5:9,2:], np.eye(4), np.zeros((4,2))
        
        
    def get_quadrotor_dynamics(self, option='full'):
        g = 9.81
        a = 0.530
        b = 0.635
        m = 25

        Ix = 8.638
        Iy = 9.014
        Iz = 16.738

        A_quad = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -g, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype='float32')

        B_quad = np.array([[0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [-1/m, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, b/Ix, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, a/Iy, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 1/Iz]], dtype='float32')
        
        A_att = np.array([[0, 1, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 0, 0]])
        B_att = np.array([[0, 0, 0],
                          [b/Ix, 0, 0],
                          [0, 0, 0],
                          [0, a/Iy, 0],
                          [0, 0, 0],
                          [0, 0, 1/Iz]])
        C_att = np.eye(A_att.shape[0])
        D_att = 0
        
        A_pos = np.array([[0, 1, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 1, 0, 0],
                          [0, 0, 0, 0, 0, 0],
                          [0, 0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 0, 0]])
        B_pos = np.array([[0, 0, 0],
                          [0, 0, -g],
                          [0, 0, 0],
                          [0, g, 0],
                          [0, 0, 0],
                          [-1/m, 0, 0]])
        C_pos = np.eye(A_pos.shape[0])
        D_pos = 0
        
        C_quad = np.eye(A_quad.shape[0])
        D_quad = 0
        if option == 'full':  
            return A_quad, B_quad, C_quad, D_quad
        if option == 'attitude-altitude':  
            return A_quad[4:,4:], B_quad[4:,:], C_quad[4:,4:], D_quad
        if option == 'position':
            return A_pos, B_pos, C_pos, D_pos
        if option == 'attitude':
            return A_att, B_att, C_att, D_att

    """ -------------------------------------------------- """
    """ PID PART - Position, Velocity and Attitude Control """       
    """ -------------------------------------------------- """

    def pid_control(self, nom_input):

        self.current_time = time.time()
        motor_vel = nom_input / math.cos(self.roll) / math.cos(self.pitch)


        if self.flag == 0:
            self.prev_time = self.current_time
            self.flag = 1
            output_roll  = 0
            output_pitch = 0
            output_yaw   = 0

        else:
            time_change = self.current_time - self.prev_time
            self.prev_time = self.current_time


            if self.setpoint_roll > self.max_roll:
                self.setpoint_roll = self.max_roll
            elif self.setpoint_roll < -self.max_roll:
                self.setpoint_roll = -self.max_roll

            if self.setpoint_pitch > self.max_pitch:
                self.setpoint_pitch = self.max_pitch
            elif self.setpoint_pitch < -self.max_pitch:
                self.setpoint_pitch = -self.max_pitch


            error_roll  =  self.setpoint_roll - self.roll
            error_pitch =  self.setpoint_pitch - self.pitch

            if self.setpoint_yaw > math.pi / 2 and self.yaw < - math.pi / 2:
                self.yaw = self.yaw + 2 * math.pi

            elif self.setpoint_yaw < - math.pi / 2 and self.yaw > math.pi / 2:
                self.yaw = self.yaw - 2 * math.pi

            error_yaw   = self.setpoint_yaw - self.yaw

            self.error_sum_roll  += (error_roll + self.prev_error_roll) / 2 * time_change
            self.error_sum_pitch += (error_pitch + self.prev_error_pitch) / 2 * time_change
            self.error_sum_yaw   += (error_yaw + self.prev_error_yaw) / 2 * time_change

            error_rate_roll  = (error_roll - self.prev_error_roll) / time_change
            error_rate_pitch = (error_pitch - self.prev_error_pitch) / time_change
            error_rate_yaw   = (error_yaw - self.prev_error_yaw) / time_change

            output_roll =  self.kp_roll * error_roll + self.ki_roll * self.error_sum_roll + self.kd_roll * error_rate_roll
            output_pitch = self.kp_pitch * error_pitch + self.ki_pitch * self.error_sum_pitch + self.kd_pitch * error_rate_pitch
            output_yaw = self.kp_yaw * error_yaw + self.ki_yaw * self.error_sum_yaw + self.kd_yaw * error_rate_yaw

            self.prev_error_roll  = error_roll
            self.prev_error_pitch = error_pitch
            self.prev_error_yaw   = error_yaw


        motor_1 = motor_vel + output_roll + output_pitch - output_yaw   #motor 1 front-left
        motor_2 = motor_vel - output_roll - output_pitch - output_yaw   #motor 2 back-right
        motor_3 = motor_vel - output_roll + output_pitch + output_yaw   #motor 3 front-right
        motor_4 = motor_vel + output_roll - output_pitch + output_yaw   #motor 4 back-left

        if motor_1 > self.max_motor_vel_1:
            motor_1 = self.max_motor_vel_1

        if motor_2 > self.max_motor_vel_2:
            motor_2 = self.max_motor_vel_2

        if motor_3 > self.max_motor_vel_3:
            motor_3 = self.max_motor_vel_3

        if motor_4 > self.max_motor_vel_4:
            motor_4 = self.max_motor_vel_4

        self.motor_input.data = [motor_1, motor_2, -motor_3, -motor_4, 0]
        self.pub_motor.publish(self.motor_input)

        return self.motor_input


    def quad_pid_altitude(self, set_z):

        self.current_alt_time = time.time()

        if self.alt_flag == 0:
            self.prev_alt_time = self.current_alt_time
            self.alt_flag = 1

        else:
            time_change = self.current_alt_time - self.prev_alt_time
            self.prev_alt_time = self.current_alt_time
            alt_error = set_z - self.z
            self.alt_error_sum  += (alt_error + self.prev_alt_error) / 2 * time_change
            alt_error_rate = (alt_error - self.prev_alt_error) / time_change

            self.prev_alt_error = alt_error

            nom_vel = (self.nominal_vel - self.kp_alt * alt_error - self.ki_alt * self.alt_error_sum - self.kd_alt * alt_error_rate)

            self.pid_control(nom_vel)

            return nom_vel


    def quad_pid_velocity_controller(self, vel_x, vel_y, vel_z):

        vel_x, vel_y, vel_z = self.inverse_rotation3D(vel_x, vel_y, vel_z)
        self.current_vel_time = time.time()

        if self.vel_flag == 0:
            self.prev_vel_time = self.current_vel_time
            self.vel_flag = 1

        else:
            time_change = self.current_vel_time - self.prev_vel_time
            self.prev_vel_time = self.current_vel_time

            if vel_x > self.max_vel_x:
                vel_x = self.max_vel_x
            elif vel_x < -self.max_vel_x:
                vel_x = -self.max_vel_x

            if vel_y > self.max_vel_y:
                vel_y = self.max_vel_y
            elif vel_y < -self.max_vel_y:
                vel_y = -self.max_vel_y

            if vel_z > self.max_vel_z:
                vel_z = self.max_vel_z
            elif vel_z < -self.max_vel_z:
                vel_z = -self.max_vel_z

            x_vel_local, y_vel_local, z_vel_local = self.inverse_rotation3D(self.x_vel_global, self.y_vel_global, self.z_vel_global)

            vel_x_error = vel_x - x_vel_local
            vel_y_error = vel_y - y_vel_local
            vel_z_error = vel_z - z_vel_local

            self.vel_x_error_sum  += (vel_x_error + self.prev_vel_x_error) / 2 * time_change
            self.vel_y_error_sum  += (vel_y_error + self.prev_vel_y_error) / 2 * time_change
            self.vel_z_error_sum  += (vel_z_error + self.prev_vel_z_error) / 2 * time_change

            vel_x_error_rate = (vel_x_error - self.prev_vel_x_error) / time_change
            vel_y_error_rate = (vel_y_error - self.prev_vel_y_error) / time_change
            vel_z_error_rate = (vel_z_error - self.prev_vel_z_error) / time_change

            self.prev_vel_x_error = vel_x_error
            self.prev_vel_y_error = vel_y_error
            self.prev_vel_z_error = vel_y_error

            self.setpoint_pitch = -(self.kp_vel_x * vel_x_error + self.ki_vel_x * self.vel_x_error_sum + self.kd_vel_x * vel_x_error_rate)
            self.setpoint_roll = self.kp_vel_y * vel_y_error + self.ki_vel_y * self.vel_y_error_sum + self.kd_vel_y * vel_y_error_rate
            nom_vel = self.nominal_vel - self.kp_vel_z * vel_z_error - self.ki_vel_z * self.vel_z_error_sum - self.kd_vel_z * vel_z_error_rate
            self.pid_control(nom_vel)



    def quad_pid_position_controller(self, pos_x, pos_y, pos_z):

        self.current_pos_time = time.time()

        if self.pos_flag == 0:
            self.prev_pos_time = self.current_pos_time
            self.pos_flag = 1
            set_vel_x = 0
            set_vel_y = 0
            set_vel_z = 0

        else:
            time_change = self.current_pos_time - self.prev_pos_time
            self.prev_pos_time = self.current_pos_time

            pos_x_error = pos_x - self.x
            pos_y_error = pos_y - self.y
            pos_z_error = pos_z - self.z

            self.pos_x_error_sum  += (pos_x_error + self.prev_pos_x_error) / 2 * time_change
            self.pos_y_error_sum  += (pos_y_error + self.prev_pos_y_error) / 2 * time_change
            self.pos_z_error_sum  += (pos_z_error + self.prev_pos_z_error) / 2 * time_change

            pos_x_error_rate = (pos_x_error - self.prev_pos_x_error) / time_change
            pos_y_error_rate = (pos_y_error - self.prev_pos_y_error) / time_change
            pos_z_error_rate = (pos_z_error - self.prev_pos_z_error) / time_change

            self.prev_pos_x_error = pos_x_error
            self.prev_pos_y_error = pos_y_error
            self.prev_pos_z_error = pos_y_error

            set_vel_x = self.kp_pos_x * pos_x_error + self.ki_pos_x * self.pos_x_error_sum + self.kd_pos_x * pos_x_error_rate
            set_vel_y = self.kp_pos_y * pos_y_error + self.ki_pos_y * self.pos_y_error_sum + self.kd_pos_y * pos_y_error_rate
            set_vel_z = self.kp_pos_z * pos_z_error + self.ki_pos_z * self.pos_z_error_sum + self.kd_pos_z * pos_z_error_rate

        self.quad_pid_velocity_controller(set_vel_x, set_vel_y, set_vel_z)

        return
      
 
    """ ------------------------------------------------------------------------- """ 
    """ LINEAR QUADRATIC REGULATOR PART - Position, Velocity and Attitude Control """            
    """ ------------------------------------------------------------------------- """    
    def quadrotor_actuator_input(self, u, motor_5 = 0):
    
        motor_1 = 10.2041*u[0,0] + 10.2041*u[1,0] + 10.2041*u[2,0] - 25.0*u[3,0] + 50**2 / math.cos(self.roll) / math.cos(self.pitch)
        motor_2 = 10.2041*u[0,0] - 10.2041*u[1,0] - 10.2041*u[2,0] - 25.0*u[3,0] + 50**2 / math.cos(self.roll) / math.cos(self.pitch)
        motor_3 = 10.2041*u[0,0] - 10.2041*u[1,0] + 10.2041*u[2,0] + 25.0*u[3,0] + 50**2 / math.cos(self.roll) / math.cos(self.pitch)
        motor_4 = 10.2041*u[0,0] + 10.2041*u[1,0] - 10.2041*u[2,0] + 25.0*u[3,0] + 50**2 / math.cos(self.roll) / math.cos(self.pitch)

        u[0,0] = u[0,0] + 4 * 50**2 * self.thrust_coefficient / math.cos(self.roll) / math.cos(self.pitch)
        if motor_1 < 0:
            motor_1 = 0
        else:
            motor_1 = math.sqrt(motor_1)

        if motor_2 < 0:
            motor_2 = 0
        else:
            motor_2 = math.sqrt(motor_2)

        if motor_3 < 0:
            motor_3 = 0
        else:
            motor_3 = math.sqrt(motor_3)

        if motor_4 < 0:
            motor_4 = 0
        else:
            motor_4 = math.sqrt(motor_4)

        dt = motor_5**2 * self.thrust_coefficientFW
        self.motor_input.data = [float(motor_1), float(motor_2), float(-motor_3), float(-motor_4), float(motor_5)]
        self.pub_motor.publish(self.motor_input)
        self.input_store = np.concatenate((self.input_store, np.array([[self.sim_time],[0],[dt],[u[0,0]],[u[1,0]],[0],[0],[u[2,0]],[u[3,0]]]).reshape(1,9)), axis=0)
        return
        
        
    def transition_actuator_input(self, u, states):
        de_trimmed = self.tr_trimmed_control_inputs[0,0]
        dt_trimmed = self.tr_trimmed_control_inputs[1,0]
        up_trimmed = self.tr_trimmed_control_inputs[3,0]
        uz = self.tr_trimmed_control_inputs[2,0] / self.thrust_coefficient / 4
        
        u_ned = states[0,0]
        v_ned = states[5,0]
        w_ned = states[1,0]
        q_ned = states[2,0]

        Vt    = math.sqrt(u_ned**2 + v_ned**2 + w_ned**2)
        alpha = math.atan(w_ned/u_ned)
        beta  = math.asin(v_ned/Vt)
        
        Fg = (self.mass * self.gravity_acc) / math.cos(self.roll) / math.cos(self.pitch) 
        qbar = Vt**2 * self.density / 2
        if Vt >= 1.5:
            Faz = (self.CL_tr + self.CLa_tr * alpha + self.Clq_tr * q_ned * self.cbar / 2 / Vt + self.Clde * u[0,0]) * qbar * self.sref
            #Fax = (self.CD0_tr + self.CDa_tr * alpha + self.CDq_tr * q_ned * self.cbar / 2 / Vt + self.Clde * u[0,0]) * qbar * self.sref
            #Mam = (self.Cm0_tr + self.Cma_tr * alpha + self.Cmq_tr * q_ned * self.cbar / 2 / Vt) * qbar * self.sref * self.cbar
            Fax = (self.CD0_tr + self.CDa_tr * abs(alpha)) * qbar * self.sref
            Mam = (self.Cm0_tr + self.Cma_tr * alpha) * qbar * self.sref * self.cbar
            #print(Fax)
        else:
            Faz = 0
            Fax = 0
            Mam = 0
        uz = (Fg - Faz) / self.thrust_coefficient / 4
        
        #u[3,0] = u[3,0] + up_trimmed / self.b
        u[3,0] = u[3,0] - Mam / self.b
        
        motor_1 = 10.2041*u[2,0] + 10.2041*u[6,0] + 10.2041*u[3,0] - 12.5*u[7,0] + uz #prop_nom
        motor_2 = 10.2041*u[2,0] - 10.2041*u[6,0] - 10.2041*u[3,0] - 12.5*u[7,0] + uz #prop_nom
        motor_3 = 10.2041*u[2,0] - 10.2041*u[6,0] + 10.2041*u[3,0] + 12.5*u[7,0] + uz #prop_nom
        motor_4 = 10.2041*u[2,0] + 10.2041*u[6,0] - 10.2041*u[3,0] + 12.5*u[7,0] + uz #prop_nom
        
        u[2,0] = u[2,0] + Fg - Faz
        if motor_1 < 0:
            motor_1 = 0
        else:
            motor_1 = math.sqrt(motor_1)

        if motor_2 < 0:
            motor_2 = 0
        else:
            motor_2 = math.sqrt(motor_2)

        if motor_3 < 0:
            motor_3 = 0
        else:
            motor_3 = math.sqrt(motor_3)

        if motor_4 < 0:
            motor_4 = 0
        else:
            motor_4 = math.sqrt(motor_4)
        
        if u[1,0] < 0 :
            motor_5 = 0
        else:
            #motor_5 = math.sqrt((u[1,0] + dt_trimmed) / self.thrust_coefficientFW)
            motor_5 = math.sqrt((u[1,0] + Fax) / self.thrust_coefficientFW)
        
        u[0,0] = u[0,0] + de_trimmed
        self.motor_input.data = [float(motor_1), float(motor_2), float(-motor_3), float(-motor_4), float(motor_5)]
        self.control_surface_input.data = [float(u[4,0]/2), float(-u[4,0]/2), float(u[5,0]),  float(u[5,0]), float(u[0,0])]
        self.pub_motor.publish(self.motor_input)
        self.pub_control_surface.publish(self.control_surface_input)
        self.input_store = np.concatenate((self.input_store, np.array([[self.sim_time],[u[0,0]],[u[1,0]],[u[2,0]],[u[3,0]],[u[4,0]],[u[5,0]],[u[6,0]],[u[7,0]]]).reshape(1,9)), axis=0)
        return
        
        
    def flight_actuator_input(self, u):
    
        ''' Forward Flight Mode '''
        ''' u1 -> delev, u2 -> thrust, u3 -> dail, u4 -> drud '''
        
        dt_trimmed = self.ff_trimmed_control_inputs[1,0]
        de_trimmed = self.ff_trimmed_control_inputs[0,0]
        u[1,0] = u[1,0] + dt_trimmed
        if u[1,0] < 0 :
            motor_5 = 0
        else:
            motor_5 = math.sqrt((u[1,0]) / self.thrust_coefficientFW)

        u[0,0] = u[0,0] + de_trimmed
        self.motor_input.data = [0, 0, 0, 0, float(motor_5)]
        self.control_surface_input.data = [float(u[2,0]/2), float(-u[2,0]/2), float(u[3,0]),  float(u[3,0]), (float(u[0,0]))]
        self.pub_motor.publish(self.motor_input)
        self.pub_control_surface.publish(self.control_surface_input)
        self.input_store = np.concatenate((self.input_store, np.array([[self.sim_time],[u[0,0]],[u[1,0]],[0],[0],[u[2,0]],[u[3,0]],[0],[0]]).reshape(1,9)), axis=0)
        return
        
    
    def lqr_quad_mode_initialization(self, dt):
        A_quad, B_quad, C_quad, D_quad = self.get_quadrotor_dynamics()
        self.lqr_quad = LinearQuadraticRegulator(A_quad, B_quad, C_quad)
        
        #Obtain LQR Gain
        Q = np.diag(np.array([20, 100, 20, 100, 20, 50, 100, 200, 100, 200, 200, 100]))
        R = np.diag(np.array([1, 1, 1, 1]))
        self.K_quad, P, eigVals = self.lqr_quad.lqr(Q,R)

        print('__________________________________________________________ \n','Quadcopter Mode Initialization: \n')
        print('Eigenvalues of the regulated system are given as:  ', eigVals)
        self.Kd_quad, P, eigVals = self.lqr_quad.lqrd(Q,R,dt)
        
        print('Continuous Kalman Gain is computed as','\n', self.K_quad, '\n')
        print('Discrete Kalman Gain is computed as','\n', self.Kd_quad, '\n')
        print('LQR Quadcopter Mode Initialization is completed!', '\n')                  
        return
               
        
    def lqr_flight_mode_initialization(self, dt):
        
        A_flight, B_flight, C_flight, D_flight = self.get_flight_dynamics()
        
        self.lqr_flight = LinearQuadraticRegulator(A_flight, B_flight, C_flight)
        
        #Obtain LQR Gain
        Q = np.diag(np.array([5, 2, 10, 5, 5, 1, 5, 2, 2, 5]))
        R = np.diag(np.array([100, 1, 100, 100]))
        self.K_flight, P, eigVals = self.lqr_flight.lqr(Q,R)

        print('__________________________________________________________ \n','Flight Mode Initialization: \n')
        print('Eigenvalues of the regulated system are given as:  ', eigVals)
        self.Kd_flight, P, eigVals = self.lqr_flight.lqrd(Q,R,dt)
        
        print('Continuous Kalman Gain is computed as','\n', self.K_flight, '\n')
        print('Discrete Kalman Gain is computed as','\n', self.Kd_flight, '\n')
        print('LQR Flight Mode Initialization is completed!', '\n')
                             
        return
        
        
    def lqr_transition_mode_initialization(self, dt):
        
        A_tran, B_tran, C_tran, D_tran = self.get_transition_dynamics()
        
        self.lqr_transition = LinearQuadraticRegulator(A_tran, B_tran, C_tran)
        
        #Obtain LQR Gain
        Q = np.diag(np.array([2, 5, 2, 50, 20, 1, 1, 1, 50, 10]))
        R = np.diag(np.array([20, 0.1, 0.01, 0.01, 20, 20, 0.01, 0.1]))
        self.K_tran, P, eigVals = self.lqr_transition.lqr(Q,R)

        print('__________________________________________________________ \n','Flight Mode Initialization: \n')
        print('Eigenvalues of the regulated system are given as:  ', eigVals)
        self.Kd_tran, P, eigVals = self.lqr_transition.lqrd(Q,R,dt)
        
        print('Continuous Kalman Gain is computed as','\n', self.K_flight, '\n')
        print('Discrete Kalman Gain is computed as','\n', self.Kd_flight, '\n')
        print('LQR Flight Mode Initialization is completed!', '\n')
                             
        return
        
        
    def lqr_quad_mode(self, ref_s_vector, states):
        u = -self.Kd_quad @ (states - ref_s_vector)
        self.quadrotor_actuator_input(u)
        return


    def lqr_transition_mode(self, ref_s_vector, states):
        xk = states - self.tr_trimmed_states
        u = -self.Kd_tran @ (xk - ref_s_vector)
        self.transition_actuator_input(u, states)
        return


    def lqr_flight_mode(self, ref_s_vector, states):
        x = states - self.ff_trimmed_states
        u = -self.Kd_flight @ (x - ref_s_vector)  
        self.flight_actuator_input(u)
        return
    
    
    def uMPC_Quadrotor_Initialization(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_quadrotor_dynamics()
        Q = np.diag(np.array([20, 100, 20, 100, 20, 50, 100, 200, 100, 200, 200, 100]))
        R = np.diag(np.array([1, 5, 5, 1]))
        Np = 90
        Nc = 2
        self.umpc_quadrotor = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, dt, 'Regulator')
        self.umpc_quadrotor.initialize_mpc_controller()
        Kd_flight = self.umpc_quadrotor.unconstrained_mpc_gain()
        print('__________________________________________________________ \n','Transition Flight Mode Initialization: \n')
        return
    
    
    def uMPC_Quadrotor_mode(self, ref_s_vector, xk):
        u = self.umpc_quadrotor.calculate_mpc_unconstraint_input(xk - ref_s_vector)
        print(u)
        self.quadrotor_actuator_input(u)
         
        return
    
    
    def uMPC_Quadrotor_Initialization2(self, dt):
        A_quad, B_quad, C_quad, D_quad = self.get_quadrotor_dynamics('attitude-altitude')

        Q = np.diag(np.array([1, 1, 500, 400, 600, 350, 10, 20]))
        R = np.diag(np.array([0.1, 1, 1, 0.1]))
        
        Np = 120
        Nc = 2
        
        self.umpc_quadrotor2 = Model_Predictive_Control(A_quad, B_quad, C_quad, Np, Nc, Q, R, dt, 'Regulator')
        self.umpc_quadrotor2.initialize_mpc_controller()
        Kd_flight = self.umpc_quadrotor2.unconstrained_mpc_gain()
        print('__________________________________________________________ \n','Quadrotor Altitude-Attitude Flight Mode Initialization: \n')
        return
    
    
    def uMPC_Quadrotor_mode2(self, ref_s_vector, states, dt = 0):
        xk = states[4:,0].reshape(8,1)
        u = self.umpc_quadrotor2.calculate_mpc_unconstraint_input(xk - ref_s_vector)
        self.quadrotor_actuator_input(u, dt)
        return
    
    
    def uMPC_Transition_Initialization(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_transition_dynamics()
        Q = np.diag(np.array([5, 2, 10, 5, 5, 1, 10, 1, 5, 1]))
        R = np.diag(np.array([100, 1, 1, 10, 100, 100, 10, 100]))
        Np = 90
        Nc = 2
        self.umpc_transition = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, dt, 'Regulator')
        self.umpc_transition.initialize_mpc_controller()
        self.umpc_transition.initialize_infinite_horizon()
        Kd_flight = self.umpc_transition.unconstrained_mpc_gain()
        print('__________________________________________________________ \n','Transition Flight Mode Initialization: \n')
        return
    
    
    def uMPC_Transition_mode(self, ref_s_vector, states):
        x = states - self.tr_trimmed_states
        u = self.umpc_transition.calculate_mpc_unconstraint_input(x - ref_s_vector)
        self.transition_actuator_input(u, states)
        return
    
    
    def uMPC_Flight_Initialization(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_flight_dynamics()
        Q = np.diag(np.array([5, 2, 10, 5, 5, 1, 5, 2, 2, 5]))
        R = np.diag(np.array([100, 1, 100, 100]))
        Np = 45
        Nc = 2
        self.umpc_flight = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, dt, 'Regulator')
        self.umpc_flight.initialize_mpc_controller()
        Kd_flight = self.umpc_flight.unconstrained_mpc_gain()
        print('__________________________________________________________ \n','Flight Mode Initialization: \n')
        return
    
    
    def uMPC_flight_mode(self, ref_s_vector, states):
        x = states - self.ff_trimmed_states
        u = self.umpc_flight.calculate_mpc_unconstraint_input(x - ref_s_vector)
        self.flight_actuator_input(u)
        return
    
    
    def cMPC_Flight_Initialization(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_flight_dynamics()        
        
        x_mins = -np.array([[30],[30],[5],[5],[30],[10],[5],[5],[5],[5]])*10
        x_maxs = np.array([[30],[30],[5],[5],[30],[10],[5],[5],[5],[5]])*10
        x_cons =  np.concatenate((x_mins, x_maxs), axis=0)
        u_mins = -np.array([[0.5],[200],[0.5],[0.5]])
        u_maxs = np.array([[0.5],[200],[0.5],[0.5]])
        u_cons =  np.concatenate((u_mins, u_maxs), axis=0)
        deltau_mins = -np.array([[0.01],[10],[0.1],[0.1]])
        deltau_maxs = np.array([[0.01],[10],[0.1],[0.1]])
        deltau_cons =  np.concatenate((deltau_mins, deltau_maxs), axis=0)
        
        Q = np.diag(np.array([5, 2, 10, 5, 5, 1, 5, 2, 2, 5]))
        R = np.diag(np.array([100, 1, 100, 100]))
        P = np.diag(np.array([5, 2, 10, 5, 5, 1, 5, 2, 2, 5]))*100
        
        Np = 45
        Nc = 2
        self.cmpc_flight = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, dt, 'Regulator')
        self.cmpc_flight.define_terminal_cost(P)
        self.cmpc_flight.initialize_model_contraints(x_cons, u_cons, deltau_cons)
        self.cmpc_flight.initialize_mpc_controller()
        print('__________________________________________________________ \n','Flight Mode Initialization: \n')
        return
    
    
    def cMPC_flight_mode(self, ref_s_vector, states):
        xk = states - self.ff_trimmed_states
        u = self.cmpc_flight.calculate_control_input(xk - ref_s_vector, option = 'CVXOPT')
        self.flight_actuator_input(u)
        return
    
    
    def cMPC_Transition_Initialization(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_transition_dynamics()        
        
        x_mins = -np.array([[30],[30],[1],[0.8],[30],[10],[5],[5],[0.5],[5]])*100
        x_maxs = np.array([[30],[30],[1],[0.8],[30],[10],[5],[5],[0.5],[5]])*100
        x_cons =  np.concatenate((x_mins, x_maxs), axis=0)
        u_mins = -np.array([[0.5],[200],[1000],[100],[0.5],[0.5],[100],[100]])
        u_maxs = np.array([[0.5],[200],[1000],[100],[0.5],[0.5],[100],[100]])
        u_cons =  np.concatenate((u_mins, u_maxs), axis=0)
        deltau_mins = -np.array([[0.5],[20],[100],[20],[0.5],[0.5],[20],[20]]).reshape(8,1)
        deltau_maxs = np.array([[0.5],[20],[100],[20],[0.5],[0.5],[20],[20]]).reshape(8,1)
        deltau_cons =  np.concatenate((deltau_mins, deltau_maxs), axis=0)
        
        Q = np.diag(np.array([5, 2, 10, 5, 5, 1, 10, 10, 5, 1]))
        R = np.diag(np.array([200, 0.1, 0.1, 10, 200, 200, 0.1, 100]))
        Np = 90
        Nc = 2
        self.cmpc_transition = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, dt, 'Regulator')
        self.cmpc_transition.initialize_model_contraints(x_cons, u_cons, deltau_cons)
        self.cmpc_transition.initialize_mpc_controller()
        print('__________________________________________________________ \n','Transition Mode Initialization: \n')
        return
    
    
    def cMPC_Transition_mode(self, ref_s_vector, states):
        xk = states - self.tr_trimmed_states
        u = self.cmpc_transition.calculate_control_input(xk - ref_s_vector, option = 'CVXOPT')
        self.transition_actuator_input(u, states)
        return
    
    
    def cMPC_Quadrotor_Initialization(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_quadrotor_dynamics()       
        
        x_mins = -np.array([[50],[5],[50],[5],[50],[5],[0.5],[2],[0.5],[2],[5],[10]])*1000000
        x_maxs = np.array([[50],[5],[50],[5],[50],[5],[0.5],[2],[0.5],[2],[5],[10]])*1000000
        x_cons =  np.concatenate((x_mins, x_maxs), axis=0)
        u_mins = -np.array([[1000],[100],[100],[100]])
        u_maxs = np.array([[1000],[100],[100],[100]])
        u_cons =  np.concatenate((u_mins, u_maxs), axis=0)
        deltau_mins = -np.array([[100],[30],[30],[30]])
        deltau_maxs = np.array([[100],[30],[30],[30]])
        deltau_cons =  np.concatenate((deltau_mins, deltau_maxs), axis=0)
        
        Q = np.diag(np.array([20, 100, 20, 100, 20, 50, 100, 400, 100, 400, 200, 100]))
        R = np.diag(np.array([1, 5, 5, 1]))
        Np = 90
        Nc = 2
        self.cmpc_quadrotor = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, dt, 'Regulator')
        self.cmpc_quadrotor.initialize_model_contraints(x_cons, u_cons, deltau_cons)
        self.cmpc_quadrotor.initialize_mpc_controller()
        print('__________________________________________________________ \n','Flight Mode Initialization: \n')
        return
    
    
    def cMPC_Quadrotor_mode(self, ref_s_vector, xk):
        u = self.cmpc_quadrotor.calculate_control_input(xk - ref_s_vector, option = 'CVXOPT')
        self.quadrotor_actuator_input(u)
        return
    
    
    def initialize_osqp_linear_constraint_mpc(self, dt):
        A_flight, B_flight, C_flight, D_flight = self.get_flight_dynamics() 

        sys  = signal.StateSpace(A_flight, B_flight, C_flight, D_flight)
        sysd = sys.to_discrete(dt)
        Ad = sysd.A
        Bd = sysd.B
        Cd = sysd.C
        Dd = sysd.D
        
        Ad = sparse.csc_matrix(Ad)
        Bd = sparse.csc_matrix(Bd)
        Cd = sparse.csc_matrix(Cd)
        Dd = sparse.csc_matrix(Dd)
        
        [nx, nu] = Bd.shape
        [self.nx, self.nu] = Bd.shape
        
        umin = -np.array([0.5, 200, 0.5, 0.5])
        umax = np.array([0.5, 200, 0.5, 0.5])
        xmin = -np.array([30, 30, 1, 0.8, 30, 10, 5, 5, 0.5, 5])
        xmax = np.array([30, 30, 1, 0.8, 30, 10, 5, 5, 0.5, 5])
        
        Q1 = np.diag(np.array([5, 2, 10, 5, 5, 1, 5, 2, 2, 5]))
        R1 = np.diag(np.array([100, 1, 100, 100]))
        
        Q  = sparse.csc_matrix(Q1)
        QN = Q
        R  = sparse.csc_matrix(R1)
        
        x0 = np.zeros(10)
        xr = np.array([0.,0.,0.,0.,15.,0.,0.,0.,0.,0.8])
        
        N = 45
        self.N = N
        K = 2
        self.K = K
        
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R)], format='csc')
        q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
                       np.zeros(N*nu)])
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-x0, np.zeros(N*nx)])
        ueq = leq
        Aineq = sparse.eye((N+1)*nx + N*nu)
        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
        A = sparse.vstack([Aeq, Aineq], format='csc')
        self.lc = np.hstack([leq, lineq])
        self.uc = np.hstack([ueq, uineq])
        self.prob = osqp.OSQP()
        
        self.prob.setup(P, q, A, self.lc, self.uc, warm_start=True)
        return
    
    
    def osqp_linear_constraint_mpc(self, xk):
        res = self.prob.solve()

        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')
    
        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]
        self.flight_actuator_input(ctrl.reshape(4,1))
        print(ctrl)
        xk = xk - self.ff_trimmed_states
        x0 = xk.reshape(10,)
        self.lc[:self.nx] = -x0
        self.uc[:self.nx] = -x0
        self.prob.update(l=self.lc, u=self.uc)
        
        return ctrl


    def cMPC_Quadrotor_Initialization2(self, dt):
        A_quad, B_quad, C_quad, D_quad = self.get_quadrotor_dynamics('attitude-altitude')

        Q = np.diag(np.array([1, 1, 500, 1000, 600, 1000, 10, 20]))
        R = np.diag(np.array([0.1, 1, 1, 0.1]))
        
        Np = 90
        Nc = 2
        
        x_mins = -np.array([[10],[5],[0.5],[2],[0.5],[2],[5],[10]])*1000
        x_maxs = np.array([[10],[5],[0.5],[2],[0.5],[2],[5],[10]])*1000
        x_cons =  np.concatenate((x_mins, x_maxs), axis=0)
        u_mins = -np.array([[1000],[100],[100],[100]])
        u_maxs = np.array([[1000],[100],[100],[100]])
        u_cons =  np.concatenate((u_mins, u_maxs), axis=0)
        deltau_mins = -np.array([[100],[30],[30],[30]])
        deltau_maxs = np.array([[100],[30],[30],[30]])
        deltau_cons =  np.concatenate((deltau_mins, deltau_maxs), axis=0)
        
        self.cmpc_quadrotor2 = Model_Predictive_Control(A_quad, B_quad, C_quad, Np, Nc, Q, R, dt, 'Regulator')
        self.cmpc_quadrotor2.initialize_model_contraints(x_cons, u_cons, deltau_cons)
        self.cmpc_quadrotor2.initialize_mpc_controller()
        Kd_flight = self.cmpc_quadrotor2.unconstrained_mpc_gain()
        print('__________________________________________________________ \n','Quadrotor Altitude-Attitude Flight Mode Initialization: \n')
        return
    
    
    def cMPC_Quadrotor_mode2(self, ref_s_vector, states, dt):
        xk = states[4:,0].reshape(8,1)
        u = self.cmpc_quadrotor2.calculate_control_input(xk - ref_s_vector, option = 'CVXOPT')
        self.quadrotor_actuator_input(u, dt)
        return
        
    def trimmed_case(self):
        self.control_surface_input.data = [0, 0, 0, 0, 0.036]
        self.pub_control_surface.publish(self.control_surface_input)
        return


    def doublet_input_case(self):
        return


    def define_mission(self, dict_input):
        self.mission_dict = dict_input
        return 0
    
    
    def perform_transition(self, tol):
        '''
        1. Quadrotor Mode Altitude: 10 meters
        2. Quadrotor2 Mode Altitude: 10 meters with dt -> 20
        3. Tranistion Mode Altitude-> 10 m, u -> 10 m/s
        4. Flight Mode Altitude -> 10 m, u -> 18 m/s
        -------

        Returns
        -------
        None.
        '''
        
        if self.transition_bool == True:
            return
        
        print(self.transition_counter)
        
        modes = {'quad': np.array([[0],[0],[0],[0],[-10],[0],[0],[0],[0],[0],[0],[0]]).reshape(12,1), \
                   'quad2': np.array([[-10],[0],[0],[0],[0],[0],[0],[0]]).reshape(8,1), \
                   'transition': np.array([[0],[0],[0],[0],[10],[0],[0],[0],[0],[0]]).reshape(10,1), \
                   'transition2': np.array([[5],[0],[0],[0],[10],[0],[0],[0],[0],[0]]).reshape(10,1), \
                   'flight': np.array([[0],[0],[0],[0],[10],[0],[0],[0],[0],[0]]).reshape(10,1)}
        
        if self.transition_counter == 0:
            state_error = modes.get('quad') - self.state_vector
            print(abs(np.transpose(state_error) @ state_error), self.transition_counter)
            if abs(np.transpose(state_error) @ state_error) > tol:
                self.cMPC_Quadrotor_mode(modes.get('quad'), self.state_vector)
            else:
                self.transition_counter = 1
        
        if self.transition_counter == 1:
            state_error = modes.get('quad2') - self.state_vector[4:,0].reshape(8,1)
            print(state_error, self.transition_counter)
            if self.states_flight[0,0] < 7:
                self.cMPC_Quadrotor_mode2(modes.get('quad2'), self.state_vector, 25)
            else:
                self.transition_counter = 2
                
        if self.transition_counter == 2:
            state_error = modes.get('transition') - self.states_flight + self.tr_trimmed_states
            print(state_error, self.transition_counter)
            if abs(np.transpose(state_error) @ state_error) > tol:
                self.cMPC_Transition_mode(modes.get('transition'), self.states_flight)
            else:
                self.transition_counter = 3
                
        if self.transition_counter == 3:
            state_error = modes.get('transition2') - self.states_flight + self.tr_trimmed_states
            print(state_error, self.transition_counter)
            if abs(np.transpose(state_error) @ state_error) > tol:
                self.cMPC_Transition_mode(modes.get('transition2'), self.states_flight)
            else:
                self.transition_counter = 4
        
        if self.transition_counter == 4:
            state_error = modes.get('transition') - self.states_flight + self.ff_trimmed_states
            print(state_error, self.transition_counter)
            if abs(np.transpose(state_error) @ state_error) > tol:
                self.cMPC_flight_mode(modes.get('flight'), self.states_flight)
            else:
                self.transition_bool = True
                
        return
    
    
    def perform_backtransition(self):
        return
    
    
    def abort_mission(self):
        return
    
    
    def mission_planner(self):
        ref_flight = np.array([[0],[0],[0],[0],[10],[0],[0],[0],[0],[0]]).reshape(10,1)
        if self.transition_bool == False:
            self.perform_transition(1)
        else:
            self.cMPC_flight_mode(ref_flight, self.states_flight)
        return


    def trajectory_generator(self):
        return 0


    def control_loop(self):
            
        freq = 30  
        self.lqr_quad_mode_initialization(1/freq)   
        self.lqr_flight_mode_initialization(1/freq)
        self.lqr_transition_mode_initialization(1/freq) 
        self.uMPC_Flight_Initialization(1/freq)
        self.uMPC_Transition_Initialization(1/freq)
        self.uMPC_Quadrotor_Initialization(1/freq)
        self.uMPC_Quadrotor_Initialization2(1/freq)
        self.cMPC_Flight_Initialization(1/freq)
        self.cMPC_Transition_Initialization(1/freq)
        self.cMPC_Quadrotor_Initialization(1/freq)
        self.cMPC_Quadrotor_Initialization2(1/freq)
        self.initialize_osqp_linear_constraint_mpc(1/freq)
        r = rospy.Rate(freq)
        
        
        
        x   =   5
        y   =   1
        z   =  -3
        psi =   np.pi/2+0.1
        ori_set_vector = np.array([[0], [0], [psi]])
        pos_set_earth = np.array([[x],[y],[z]])
        pos_set_body = self.earth2body_transformation(ori_set_vector, pos_set_earth)
        ref_quad   = np.array([[pos_set_body[0,0]],[0],[pos_set_body[1,0]],[0],[pos_set_body[2,0]],[0],[0],[0],[0],[0],[psi],[0]]).reshape(12,1)
        ref_quad2  = np.array([[-10],[0],[0],[0],[0],[0],[0],[0]]).reshape(8,1)
        ref_tran   = np.array([[0],[0],[0],[0],[10],[0],[0],[0],[0],[0]]).reshape(10,1)
        ref_tran2  = np.array([[5],[0],[0],[0],[10],[0],[0],[0],[0],[0]]).reshape(10,1)
        ref_flight = np.array([[0],[0],[0],[0],[15],[0],[0],[0],[0],[0.8]]).reshape(10,1)
        
        while not rospy.is_shutdown():

            self.state_vector = np.array([[self.x],[self.u],[self.y],[self.v],[self.z],[self.w],[self.phi],[self.p],[self.theta],[self.q],[self.psi],[self.r]], dtype= 'float32')
            self.states_flight = np.array([[self.u],[self.w],[self.q],[self.theta],[self.z],[self.v],[self.p],[self.r],[self.phi],[self.psi]],dtype= 'float32')
            #print(self.states_flight)
            states = np.array([[self.sim_time],[self.x],[self.u],[self.y],[self.v],[self.z],[self.w],[self.phi],[self.p],[self.theta],[self.q],[self.psi],[self.r]]).reshape(1,13)
            self.state_store = np.concatenate((self.state_store, states), axis=0)
            #self.mission_planner()
            #self.message.data = "hello"
            #self.pub_a_topic.publish(self.message)
            #self.pid_control(self.nominal_vel)
            #self.quad_pid_altitude(-10)
            #a = self.quad_pid_velocity_controller(1, 1, -1)
            #a = self.quad_pid_position_controller(10.0, 10.0, -5.0)

            #print(states_flight)
            #self.lqr_quad_mode(ref_quad,self.state_vector)
            #self.lqr_transition_mode(10, 0, 0, 0, states_flight)
            self.uMPC_Quadrotor_mode(ref_quad, self.state_vector)
            #self.uMPC_Quadrotor_mode2(-10, 0, self.state_vector, 20)
            #self.uMPC_Transition_mode(12, 5, 0.0, 0.0, self.states_flight)
            #self.cMPC_flight_mode(10, 0, 0.0, 0, self.states_flight)
            #self.cMPC_Transition_mode(10, 0, 0.0, 0.0, self.states_flight)
            #self.cMPC_Quadrotor_mode(ref_quad, self.state_vector)
            #self.lqr_flight_mode(ref_flight, self.states_flight)
            #self.cMPC_flight_mode(ref_flight, self.states_flight)
            #self.osqp_linear_constraint_mpc(self.states_flight)
            #self.uMPC_flight_mode(ref_flight, self.states_flight)
            #self.trimmed_case()
            r.sleep()
     




if __name__ == '__main__':
    vtol = controller()
    try:
        vtol.control_loop()

    except KeyboardInterrupt:
        state_data, input_data = vtol.get_stored_data()
        savetxt('src/' + vtol.get_name() + '/sim_data/data_s.csv', state_data, delimiter=',')
        savetxt('src/' + vtol.get_name() + '/sim_data/data_i.csv', input_data, delimiter=',')
        print(' Data has been saved! ')


