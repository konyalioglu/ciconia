#! /usr/bin/python3

import time
import numpy as np
from numpy import nan
import rospy
from geometry_msgs.msg import Pose, Vector3Stamped, Wrench
from sensor_msgs.msg import JointState, Imu, NavSatFix
from gazebo_msgs.msg import ModelStates, LinkStates
from std_msgs.msg import Float64MultiArray, Float32
import math
import time
from gazebo_msgs.srv import ApplyBodyWrench
from scipy.io import loadmat
from scipy.interpolate import LinearNDInterpolator
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import interpn
import os



class Physics:

    def __init__(self):

        self.aerodynamic_data_initialization()

        self.phi=0
        self.theta=0
        self.psi=0

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

        self.alpha = 0
        self.Vt = 0
        self.beta = 0
        self.mach = 0

        self.delevator = 0
        self.drudder = 0
        self.daileron = 0

        self.prop1_vel = 0
        self.prop2_vel = 0
        self.prop3_vel = 0
        self.prop4_vel = 0
        self.prop5_vel = 0
        self.propFW_vel = 0

        self.thrust_coefficient = 0.0245
        self.thrust_coefficientFW = 0.102
        self.hub_coefficient = 0.01
        self.hub_coefficient_FW = 0.001
        self.CG = np.array([[0],[0],[0.016389]])

        self.density = 1.225
        self.speedofsound = 343
        self.m = 25
        self.g = 9.8065

        self.r1 = np.array([[0.530], [0.635], [0.0]]) - self.CG
        self.r2 = np.array([[-0.530], [-0.635], [0.0]]) - self.CG
        self.r3 = np.array([[0.530], [-0.635], [0.0]]) - self.CG
        self.r4 = np.array([[-0.530], [0.635], [0.0]]) - self.CG

        self.wrench = Wrench()

        ##Ros Initialization
        rospy.init_node('physics')
        self.pub_force_moment = rospy.Publisher('/ciconia/ForcesAndMoments', Wrench, queue_size=5)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_handler)
        rospy.Subscriber("/ciconia/joint_states", JointState, self.joint_states_handler)
        rospy.Subscriber("/ciconia/joint_vel", Float64MultiArray, self.motor_speed_handler)


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
        #Z = math.atan(t3/t4)

        return X, Y, Z


    def joint_states_handler(self, data):
        self.daileron = data.position[3] - data.position[4]
        self.delevator = data.position[0]
        self.drudder = data.position[2]
        return
    
    
    def motor_speed_handler(self, data):
        self.prop1_vel = data.data[0]
        self.prop2_vel = data.data[1]
        self.prop3_vel = data.data[2]
        self.prop4_vel = data.data[3]
        self.prop5_vel = data.data[4]
        return


    def pose_handler(self, data):

        body_index = data.name.index('ciconia::body')

        r_body = data.pose[body_index].position

        orientation = data.pose[body_index].orientation
        self.phi, self.theta, self.psi = self.quaternion_to_euler_angle(orientation.w, orientation.x, orientation.y, orientation.z)
        self.phi_ned = self.phi
        self.theta_ned = -self.theta
        self.psi_ned = -self.psi


        ori_vector = np.array([[self.phi], [self.theta], [self.psi]])
        ori_vector_ned = np.array([[self.phi_ned], [self.theta_ned], [self.psi_ned]])

        vel_vector = np.array([[data.twist[body_index].linear.x],[data.twist[body_index].linear.y],[data.twist[body_index].linear.z]])
        vel_vector_ned = np.array([[data.twist[body_index].linear.x],[-data.twist[body_index].linear.y],[-data.twist[body_index].linear.z]])
        #print(vel_vector)
        #vel_vector = self.earth2gazebo_transformation(self.psi, vel_vector)
        vel_vector = self.earth2body_transformation(ori_vector, vel_vector)
        #vel_vector_ned = self.body2earth_transformation(ori_vector_ned, vel_vector_ned)

        self.u = vel_vector[0]
        self.v = vel_vector[1]
        self.w = vel_vector[2]
        

        self.u_ned = self.u
        self.v_ned = -self.v
        self.w_ned = -self.w
        

        rates = np.array([[data.twist[body_index].angular.x],[data.twist[body_index].angular.y],[data.twist[body_index].angular.z]])
        rates = self.earth2body_transformation(ori_vector, rates)
        #rates = self.euler_rate2body_rate(ori_vector, rates1)
        self.p = rates[0,0]
        self.q = rates[1,0]
        self.r = rates[2,0]

        self.p_ned = rates[0,0]
        self.q_ned = -rates[1,0]
        self.r_ned = -rates[2,0]

        #wind axis parameters, Wind Velocity, Angle of Attack, Side slip angle
        self.Vt    = math.sqrt(self.u_ned**2 + self.v_ned**2 + self.w_ned**2)
        self.alpha = math.atan(self.w_ned/self.u_ned)
        self.beta  = math.asin(self.v_ned/self.Vt)
        
        #print('Gazebo Rates: ', data.twist[body_index].angular.x, data.twist[body_index].angular.y, data.twist[body_index].angular.z)
        #print('Rotated Rates: ', rates1)
        #print('Body Rates: ', rates)
        #print('Euler Angles: ',self.phi,self.theta,self.psi)
        #print('Body Velocity Vector: ',self.u_ned,self.v_ned,self.w_ned)
        #print('Wind Parameters: ', self.alpha, self.w_ned, self.u_ned)

        self.mach = self.Vt / self.speedofsound

        if self.alpha > self.alpha_max:
            self.alpha = self.alpha_max
        elif self.alpha < self.alpha_min:
            self.alpha = self.alpha_min

        if self.beta > self.beta_max:
            self.beta = self.beta_max
        elif self.beta < self.beta_min:
            self.beta = self.beta_min

        if self.mach > self.mach_max:
            self.mach = self.mach_max
        elif self.mach < self.mach_min:
            self.mach = self.mach_min
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


    def propulsive_forces_and_moments(self):

        ori_vector = np.array([[self.phi], [self.theta], [self.psi]])

        Force1 = self.thrust_coefficient * self.prop1_vel**2
        Force2 = self.thrust_coefficient * self.prop2_vel**2
        Force3 = self.thrust_coefficient * self.prop3_vel**2
        Force4 = self.thrust_coefficient * self.prop4_vel**2
        Force5 = self.thrust_coefficientFW * self.prop5_vel**2

        prop1_force_body = np.array([[0],[0],[Force1]])
        prop2_force_body = np.array([[0],[0],[Force2]])
        prop3_force_body = np.array([[0],[0],[Force3]])
        prop4_force_body = np.array([[0],[0],[Force4]])
        prop5_force_body = np.array([[Force5],[0],[0]])

        prop1_moment_body = np.cross(self.r1.reshape((1,3)), prop1_force_body.reshape((1,3))).reshape((3,1))
        prop2_moment_body = np.cross(self.r2.reshape((1,3)), prop2_force_body.reshape((1,3))).reshape((3,1))
        prop3_moment_body = np.cross(self.r3.reshape((1,3)), prop3_force_body.reshape((1,3))).reshape((3,1))
        prop4_moment_body = np.cross(self.r4.reshape((1,3)), prop4_force_body.reshape((1,3))).reshape((3,1))
        
        hub_quad_moment_body = self.hub_coefficient * ( \
            math.copysign(1, self.prop1_vel) * self.prop1_vel**2 + \
            math.copysign(1, self.prop2_vel) * self.prop2_vel**2 + \
            math.copysign(1, self.prop3_vel) * self.prop3_vel**2 + \
            math.copysign(1, self.prop4_vel) * self.prop4_vel**2)
            
        hub_fw_moment_body = self.hub_coefficient_FW * self.prop5_vel**2 * math.copysign(1, self.prop5_vel)
        hub_torque_moment  = np.array([[hub_fw_moment_body],[0],[hub_quad_moment_body]])
        
        prop_force_body = prop1_force_body + prop2_force_body + prop3_force_body + prop4_force_body + prop5_force_body
        prop_moment_body = prop1_moment_body + prop2_moment_body + prop3_moment_body + prop4_moment_body + hub_torque_moment

        forces_earth = self.body2earth_transformation(ori_vector, prop_force_body)
        moments_earth = self.body2earth_transformation(ori_vector, prop_moment_body)

        return forces_earth, moments_earth


    def aerodynamic_data_initialization(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        aero_data = loadmat(dir_path + '/aerodynamic_database.mat')

        ## Aerodynamic Parameters
        self.sref = float(aero_data['data']['sref'][0])
        self.cbar = float(aero_data['data']['cbar'][0])
        self.bref = float(aero_data['data']['bref'][0])


        ## Breakpoints
        alpha = np.array(aero_data['data']['alpha'][0][0][0], dtype='float32') * np.pi / 180
        self.alpha_min = alpha[0]
        self.alpha_max = alpha[-1]
        nalpha = float(aero_data['data']['nalpha'][0])

        beta = np.array(aero_data['data']['beta'][0][0][0], dtype='float32') * np.pi / 180
        self.beta_min = beta[0]
        self.beta_max = beta[-1]
        nbeta = float(aero_data['data']['nbeta'][0])

        mach = np.array(aero_data['data']['mach'][0][0][0], dtype='float32')
        nmach = float(aero_data['data']['nmach'][0])
        self.mach_min = mach[0]
        self.mach_max = mach[-1]

        ## Drag Force Coefficients
        CD = np.array(aero_data['data']['CD'][0][0], dtype='float32')

        self.CD_interpn = RegularGridInterpolator((alpha, beta, mach), CD, method='linear', bounds_error=False, fill_value=nan)

        ## Lift Force Coefficients
        CL = np.array(aero_data['data']['CL'][0][0], dtype='float32')
        self.CLq = float(aero_data['data']['CLq'][0][0][0])
        self.CLde = -float(aero_data['data']['Czde'][0][0][0])

        self.CL_interpn = RegularGridInterpolator((alpha,beta, mach), CL, method='linear', bounds_error=False, fill_value=nan)

        ## Force Coefficient on y-axis
        Cy = np.array(aero_data['data']['Cy'][0][0], dtype='float32')

        self.Cyp = float(aero_data['data']['Cyp'][0][0][0])
        self.Cyr = float(aero_data['data']['Cyr'][0][0][0])
        self.Cydr = float(aero_data['data']['Cydr'][0][0][0])

        self.Cy_interpn = RegularGridInterpolator((alpha, beta, mach), Cy, method='linear', bounds_error=False, fill_value=nan)

        ## Rolling Moment Coefficients
        Cl = np.array(aero_data['data']['Cl'][0][0], dtype='float32')

        self.Clp = float(aero_data['data']['Clp'][0][0][0])
        self.Clr = float(aero_data['data']['Clr'][0][0][0])
        self.Clda = float(aero_data['data']['Clda'][0][0][0])
        self.Cldr = float(aero_data['data']['Cldr'][0][0][0])

        self.Cl_interpn = RegularGridInterpolator((alpha, beta, mach), Cl, method='linear', bounds_error=False, fill_value=nan)

        ## Pitching Moment Coefficients
        Cm = np.array(aero_data['data']['Cm'][0][0], dtype='float32')

        self.Cmq = float(aero_data['data']['Cmq'][0][0][0])
        self.Cmde = float(aero_data['data']['Cmde'][0][0][0])

        self.Cm_interpn = RegularGridInterpolator((alpha, beta, mach), Cm, method='linear', bounds_error=False, fill_value=nan)

        ## Yawing Moment Coefficients
        Cn = np.array(aero_data['data']['Cn'][0][0], dtype='float32')

        self.Cnp = float(aero_data['data']['Cnp'][0][0][0])
        self.Cnr = float(aero_data['data']['Cnr'][0][0][0])
        self.Cnda = float(aero_data['data']['Cnda'][0][0][0])
        self.Cndr = float(aero_data['data']['Cndr'][0][0][0])

        self.Cn_interpn = RegularGridInterpolator((alpha, beta, mach), Cn, method='linear', bounds_error=False, fill_value=nan)

        return
        
        
    def gravitational_model(self):
        "Euler Angles must have the format of [phi, theta, psi]' "
        euler_angles = np.array([[self.phi], [self.theta], [self.psi]])
        
        Fg1 = -self.m * self.g * np.sin(self.theta_ned)
        Fg2 =  self.m * self.g * np.cos(self.theta_ned) * np.sin(self.phi_ned)
        Fg3 =  self.m * self.g * np.cos(self.theta_ned) * np.cos(self.phi_ned)
        
        return self.body2earth_transformation(euler_angles, np.array([[Fg1],[-Fg2],[-Fg3]]))


    def aerodynamic_forces_and_moments(self):

        if self.u_ned <= 1.5:
            return np.array([[0],[0],[0]]), np.array([[0],[0],[0]])

        #print('Wind Parameters:  ', self.alpha, self.beta, self.mach, '\n')
        CD = self.CD_interpn((self.alpha, self.beta, self.mach))

        CL = self.CL_interpn((self.alpha, self.beta, self.mach))

        Cy = self.Cy_interpn((self.alpha, self.beta, self.mach))

        Cl = self.Cl_interpn((self.alpha, self.beta, self.mach))

        Cm = self.Cm_interpn((self.alpha, self.beta, self.mach))

        Cn = self.Cn_interpn((self.alpha, self.beta, self.mach))

        '''
        ###

        CD = self.CD_interpn((self.theta_ned, -self.psi_ned, self.mach))

        CL = self.CL_interpn((self.theta_ned, -self.psi_ned, self.mach))

        Cy = self.Cy_interpn((self.theta_ned, -self.psi_ned, self.mach))

        Cl = self.Cl_interpn((self.theta_ned, -self.psi_ned, self.mach))

        Cm = self.Cm_interpn((self.theta_ned, -self.psi_ned, self.mach))

        Cn = self.Cn_interpn((self.theta_ned, -self.psi_ned, self.mach))
        '''

        dyn_press = self.Vt**2 * self.density / 2

        Lift = (CL + self.q_ned * self.cbar / (2 * self.Vt) * self.CLq + self.CLde * self.delevator) * dyn_press * self.sref
        Drag = CD * dyn_press * self.sref

        X = -Drag * np.cos(self.alpha) + Lift * np.sin(self.alpha)
        Z = -Drag * np.sin(self.alpha) - Lift * np.cos(self.alpha)

        #???
        Y = (Cy + self.bref * self.p_ned / (2 * self.Vt) * self.Cyp + self.bref * self.r_ned / (2 * self.Vt) * self.Cyr + self.Cydr * self.drudder) * dyn_press * self.sref

        L = (Cl + self.bref * self.p_ned / (2 * self.Vt) * self.Clp + self.bref * self.r_ned / (2 * self.Vt) * self.Clr + self.Clda * self.daileron + self.Cldr * self.drudder) * self.sref * self.bref * dyn_press

        M = (Cm + self.Cmde * self.delevator * self.cbar + self.cbar * self.q_ned / (2 * self.Vt) * self.Cmq) * self.sref * self.cbar * dyn_press

        N = (Cn + self.bref * self.p_ned / (2 * self.Vt) * self.Cnp + self.bref * self.r_ned / (2 * self.Vt) * self.Cnr + self.Cnda * self.daileron + self.Cndr * self.drudder) * self.sref * self.bref * dyn_press

        ori_vector = np.array([[self.phi], [self.theta], [self.psi]])
        #print(np.array([[L],[M],[N]]))

        aero_forces_earth = self.body2earth_transformation(ori_vector, np.array([[X],[-Y],[-Z]]))
        aero_moments_earth = self.body2earth_transformation(ori_vector, np.array([[L],[-M],[-N]]))
        #print(aero_moments_earth)
        #print(np.array([[self.p_ned],[self.q_ned],[self.r_ned]]))
        return aero_forces_earth, aero_moments_earth


    def publish_forces_and_moments(self):

        prop_forces, prop_moments = self.propulsive_forces_and_moments()
        #print(prop_forces, prop_moments)
        aero_forces, aero_moments = self.aerodynamic_forces_and_moments()
        grav_forces = self.gravitational_model()

        #print('Aerodynamic Forces: \n' , aero_forces, 'Aerodynamic Moments: \n', aero_moments)



        if  np.isnan(aero_forces[0,0]):
            aero_forces[0,0] = 0


        if  np.isnan(aero_forces[1,0]):
            aero_forces[1,0] = 0


        if  np.isnan(aero_forces[2,0]):
            aero_forces[2,0] = 0


        if np.isnan(aero_moments[0,0]):
            aero_moments[0,0] = 0


        if np.isnan(aero_moments[1,0]):
            aero_moments[1,0] = 0


        if np.isnan(aero_moments[2,0]):
            aero_moments[2,0] = 0


        forces_earth = prop_forces + aero_forces + grav_forces
        moments_earth = prop_moments + aero_moments
        #print('\nTotal Forces: \n', forces_earth, '\nTotal Moments: \n', moments_earth)
        
        self.wrench.force.x = forces_earth[0,0]
        self.wrench.force.y = forces_earth[1,0]
        self.wrench.force.z = forces_earth[2,0]

        self.wrench.torque.x = moments_earth[0,0]
        self.wrench.torque.y = moments_earth[1,0]
        self.wrench.torque.z = moments_earth[2,0]

        self.pub_force_moment.publish(self.wrench)
        #print('Wind Parameters: ', self.alpha, self.beta, self.Vt)
        return


    def physics_loop(self):
        r = rospy.Rate(250)
        print('Aerodynamic and Propulsive Forces and Moments are being published with Gazebo!')
        while not rospy.is_shutdown():
            self.publish_forces_and_moments()

            r.sleep()





if __name__ == '__main__':

    vtol = Physics()

    vtol.physics_loop()
    #vtol.constraint_mpc_control_loop()
    #try:
        #vtol.control_loop()
    #except rospy.ROSInterruptException:
        #pass
    #rospy.spin()
