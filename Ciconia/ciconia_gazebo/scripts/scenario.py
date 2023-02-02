#! /usr/bin/python3

import numpy as np
import rospy 
import rospkg 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState, SetJointProperties
from std_msgs.msg import Float64MultiArray, Float32
import math
import sys


class Scenario:

    def __init__(self):
		
        self.motor_input = Float64MultiArray() 
        self.control_surface_input = Float64MultiArray()

        rospy.init_node('init_scenario')
        self.pub_motor = rospy.Publisher('/ciconia/joint_motor_controller/command', Float64MultiArray, queue_size=4)
        self.pub_control_surface = rospy.Publisher('/ciconia/joint_controlDeflection_controller/command', Float64MultiArray, queue_size=4)
		

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
    
    
    def body2earth_transformation(self, angles, vector):
        rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]])
        roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]])
        rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
        return np.matrix.transpose(rotz @ roty @ rotx) @ vector


    def earth2body_transformation(self, angles, vector):
        rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]])
        roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]])
        rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
        return rotz @ roty @ rotx @ vector
		

    def flight_trim_scenario(self, alt):
        
        ff_trimmed_states = np.array([[18],[0.281],[0],[1.567e-02],[0],[0],[0],[0],[0],[0]]).reshape(10,1)
        ff_trimmed_control_inputs = np.array([[4.638e-02],[9.0],[0],[0]]).reshape(4,1)
        
        uvw = np.array([[ff_trimmed_states[0,0]],[-ff_trimmed_states[5,0]],[-ff_trimmed_states[1,0]]])
        pqr = np.array([[ff_trimmed_states[6,0]],[-ff_trimmed_states[2,0]],[-ff_trimmed_states[7,0]]])
        eulera =  np.array([[ff_trimmed_states[8,0]],[-ff_trimmed_states[3,0]],[-ff_trimmed_states[9,0]]])
        
        uvw_earth = self.body2earth_transformation(eulera, uvw)
        pqr_earth = self.body2earth_transformation(eulera, pqr)
        w, x, y, z = self.euler_angles_to_quaternions(eulera[2,0], eulera[1,0], eulera[0,0])
        
        joint_msg = SetJointProperties()

        state_msg = ModelState()
        state_msg.model_name = 'ciconia'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = alt

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        state_msg.twist.linear.x = uvw_earth[0,0]
        state_msg.twist.linear.y = uvw_earth[1,0]
        state_msg.twist.linear.z = uvw_earth[2,0]

        state_msg.twist.angular.x = pqr_earth[0,0]
        state_msg.twist.angular.y = pqr_earth[1,0]
        state_msg.twist.angular.z = pqr_earth[2,0]


        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException:
            print ("Service call failed: %s")

        return 0


    def flight_transition_scenario(self, alt):
		

        state_msg = ModelState()
        state_msg.model_name = 'ciconia'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = alt

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        state_msg.twist.linear.x = 10
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0


        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException:
            print ("Service call failed: %s")

        return 0
    
    
    def flight_quadrotor_scenario(self, alt):
		

        state_msg = ModelState()
        state_msg.model_name = 'ciconia'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = alt

        state_msg.pose.orientation.x = 0
        state_msg.pose.orientation.y = 0
        state_msg.pose.orientation.z = 0
        state_msg.pose.orientation.w = 0

        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0

        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException:
            print ("Service call failed: %s")

        return 0
    
    
    
    def runway(self, psi):
        
        w, x, y, z = self.euler_angles_to_quaternions(psi, 0, 0)
        angles = np.array([[0],[0],[psi]])
        
        joint_msg = SetJointProperties()

        state_msg = ModelState()
        state_msg.model_name = 'ciconia'
        state_msg.pose.position.x = 0
        state_msg.pose.position.y = 0
        state_msg.pose.position.z = 0

        state_msg.pose.orientation.x = x
        state_msg.pose.orientation.y = y
        state_msg.pose.orientation.z = z
        state_msg.pose.orientation.w = w

        state_msg.twist.linear.x = 0
        state_msg.twist.linear.y = 0
        state_msg.twist.linear.z = 0

        state_msg.twist.angular.x = 0
        state_msg.twist.angular.y = 0
        state_msg.twist.angular.z = 0


        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg )

        except rospy.ServiceException:
            print ("Service call failed: %s")

        return 0
    


if __name__ == '__main__':
    	
        sc = Scenario()
        
        if str(sys.argv[1]) == 'flight':
            try:
                alt = float(sys.argv[2])
                sc.flight_trim_scenario(alt)
                print(sys.argv[1], sys.argv[2])
            except IndexError:
                sc.flight_trim_scenario(10)
                print(sys.argv[1], 10)
                
        elif sys.argv[1] == 'transition':
            if sys.argv[2] is not None:
                alt = float(sys.argv[2])
                sc.flight_transition_scenario(alt)
            else:
                sc.flight_transition_scenario(10)
                      
        elif sys.argv[1] == 'quadrotor':
            if sys.argv[2] is not None:
                alt = float(sys.argv[2])
                sc.flight_quadrotor_scenario(alt)
            else:
                sc.flight_quadrotor_scenario(10)
                
        elif sys.argv[1] == 'runway':
            if sys.argv[2] is not None:
                heading = float(sys.argv[2])
                heading = heading * np.pi / 180
                sc.runway(heading)
            else:
                heading = 0
                sc.runway(heading)
                
        print('Gazebo is Ready')

        #sc.flight_transition_scenario()
        #sc.flight_trim_scenario(5)
        #sc.runway(0, 0.0,0.0)
        #sc.runway(2.8, 0.0,0.00)
        #sc.runway2(1.5708, 0.0,0.00)
	
