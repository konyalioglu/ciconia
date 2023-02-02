#! /usr/bin/python3


import numpy as np
from numpy import nan
import math, rospy

from geometry_msgs.msg import Wrench
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Float64MultiArray

from scipy.io import loadmat
from scipy.interpolate import RegularGridInterpolator, interp1d
import os
from utils import *





class aerodynamics:
    
    def __init__(self):
        
        dir_path = os.path.dirname(os.path.realpath(__file__))
        aero_data = loadmat(dir_path + '/database/aerodynamic_database.mat')
        
        "Aerodynamic Parameters"
        self.sref = float(aero_data['data']['sref'][0])
        self.cbar = float(aero_data['data']['cbar'][0])
        self.bref = float(aero_data['data']['bref'][0])
        self.c    = 343
        self.density = 1.225

        "Breakpoints"
        alpha = np.array(aero_data['data']['alpha'][0][0][0], dtype='float32') * np.pi / 180
        self.alpha_min = alpha[0]
        self.alpha_max = alpha[-1]
        nalpha = alpha.size

        beta = np.array(aero_data['data']['beta'][0][0][0], dtype='float32') * np.pi / 180
        self.beta_min = beta[0]
        self.beta_max = beta[-1]
        nbeta = beta.size
		
        mach = np.array(aero_data['data']['mach'][0][0][0], dtype='float32')
        nmach = mach.size
        self.mach_min = mach[0]
        self.mach_max = mach[-1]
		
        "Drag Force Coefficients"
        CD = np.array(aero_data['data']['CD'][0][0], dtype='float32')
        self.CD_interpn = RegularGridInterpolator((alpha, beta, mach), CD, method='linear', bounds_error=False, fill_value=None)


        "Lift Force Coefficients"
        CL = np.array(aero_data['data']['CL'][0][0], dtype='float32')
        self.CLq = float(aero_data['data']['CLq'][0][0][0])
        self.Czde = float(aero_data['data']['Czde'][0][0][0])
        self.CL_interpn = RegularGridInterpolator((alpha,beta, mach), CL, method='linear', bounds_error=False, fill_value=None)


        "Force Coefficient on y-axis"
        Cy = np.array(aero_data['data']['Cy'][0][0], dtype='float32')
        self.Cyp = float(aero_data['data']['Cyp'][0][0][0])
        self.Cyr = float(aero_data['data']['Cyr'][0][0][0])
        self.Cydr = float(aero_data['data']['Cydr'][0][0][0])
        self.Cy_interpn = RegularGridInterpolator((alpha, beta, mach), Cy, method='linear', bounds_error=False, fill_value=None)
				

        "Rolling Moment Coefficients"
        Cl = np.array(aero_data['data']['Cl'][0][0], dtype='float32')
        self.Clp = float(aero_data['data']['Clp'][0][0][0])
        self.Clr = float(aero_data['data']['Clr'][0][0][0])
        self.Clda = float(aero_data['data']['Clda'][0][0][0])
        self.Cldr = float(aero_data['data']['Cldr'][0][0][0])
        self.Cl_interpn = RegularGridInterpolator((alpha, beta, mach), Cl, method='linear', bounds_error=False, fill_value=None)


        "Pitching Moment Coefficients"
        Cm = np.array(aero_data['data']['Cm'][0][0], dtype='float32')
        self.Cmq = float(aero_data['data']['Cmq'][0][0][0])
        self.Cmde = float(aero_data['data']['Cmde'][0][0][0])
        self.Cm_interpn = RegularGridInterpolator((alpha, beta, mach), Cm, method='linear', bounds_error=False, fill_value=None)		
		

        "Yawing Moment Coefficients"
        Cn = np.array(aero_data['data']['Cn'][0][0], dtype='float32')
        self.Cnp = float(aero_data['data']['Cnp'][0][0][0])
        self.Cnr = float(aero_data['data']['Cnr'][0][0][0])
        self.Cnda = float(aero_data['data']['Cnda'][0][0][0])
        self.Cndr = float(aero_data['data']['Cndr'][0][0][0])
        self.Cn_interpn = RegularGridInterpolator((alpha, beta, mach), Cn, method='linear', bounds_error=False, fill_value=None)   
        
    
    def aerodynamic_model(self, wind_param, rates_body, cont_surf):
        Vt = wind_param[0,0]
        if Vt == 0:
            return np.array([[0],[0],[0]]), np.array([[0],[0],[0]])
        alpha = wind_param[1,0]
        beta = wind_param[2,0]
        
        p = rates_body[0,0]
        q = rates_body[1,0]
        r = rates_body[2,0]
        
        delev = cont_surf[0,0]
        dail = cont_surf[1,0]
        drud = cont_surf[2,0]
        
        mach = Vt / self.c
        
        if alpha > self.alpha_max:
            alpha = self.alpha_max
        elif alpha < self.alpha_min:
            alpha = self.alpha_min
        if beta > self.beta_max:
            beta = self.beta_max
        elif beta < self.beta_min:
            beta = self.beta_min
        if mach > self.mach_max:
            mach = self.mach_max
        elif mach < self.mach_min:
            mach = self.mach_min
            
        dyn_press = Vt**2 * self.density / 2
        
        CD = self.CD_interpn((alpha, beta, mach))
        CL = self.CL_interpn((alpha, beta, mach)) + q * self.cbar / (2 * Vt) * self.CLq
        Cy = self.Cy_interpn((alpha, beta, mach))
        Cl = self.Cl_interpn((alpha, beta, mach))
        Cm = self.Cm_interpn((alpha, beta, mach))
        Cn = self.Cn_interpn((alpha, beta, mach))
        
        Cx = -CD * np.cos(alpha) + CL * np.sin(alpha)
        Cz = -CD * np.sin(alpha) - CL * np.cos(alpha)
        
        X = Cx * dyn_press * self.sref
        Z = (Cz + self.Czde * delev)  * dyn_press * self.sref
        Y = (Cy + self.bref * p / (2 * Vt) * self.Cyp + self.bref * r / (2 * Vt) * self.Cyr + self.Cydr * drud) * dyn_press * self.sref
        L = (Cl + self.bref * p / (2 * Vt) * self.Clp + self.bref * r / (2 * Vt) * self.Clr + self.Clda * dail + self.Cldr * drud) * self.sref * self.bref * dyn_press
        M = (Cm + self.Cmde * delev * self.cbar + self.cbar * q / (2 * Vt) * self.Cmq) * self.sref * self.cbar * dyn_press
        N = (Cn + self.bref * p / (2 * Vt) * self.Cnp + self.bref * r / (2 * Vt) * self.Cnr + self.Cnda * dail + self.Cndr * drud) * self.sref * self.bref * dyn_press

        return np.array([[X],[Y],[Z]]), np.array([[L],[M],[N]])



class gravitational:
    
    def __init__(self, m, g = 9.8065, r = None):
        self.m = m
        self.g = g
        self.r = r
        
        
    def gravitational_forces(self, euler_angles):
        Fg1 = -self.m * self.g * np.sin(euler_angles[1,0])
        Fg2 =  self.m * self.g * np.cos(euler_angles[1,0]) * np.sin(euler_angles[0,0])
        Fg3 =  self.m * self.g * np.cos(euler_angles[1,0]) * np.cos(euler_angles[0,0])
        
        return np.array([[Fg1],[Fg2],[Fg3]])
    
    
    def gravitational_moments(self, euler_angles):
        if self.r == None:
            assert False, "Position vector hasn't selected!"
        else:
            Fg = self.gravitation_forces(euler_angles)
            Mg = np.cross(self.r.reshape((1,3)), Fg.reshape((1,3))).reshape((3,1))
            return Mg

        

class gyroscopic:
    
    def __init__(self, rotation_matrix=np.eye(3)):
        
        self.P_2685_R = 26 * 0.0254 * 2
        self.P_2110_R = 21 * 0.0254 * 2
        
        self.P_2685_M = 0.041
        self.P_2110_M = 7.94 * 28.35 / 1000
        
        self.P_2685_J = 1 / 2 * self.P_2685_M * self.P_2685_R**2
        self.P_2110_J = 1 / 2 * self.P_2110_M * self.P_2110_R**2
        
        self.TBP = rotation_matrix
        Hp = np.array([[0.0],[0.0],[0.0]])
        
        
    def gyroscopic_moment(self, angular_rates, *args):
        Hp = np.array([[0.0],[0.0],[0.0]])
        for arg in args[0]:
            if arg.type == 0:
                J = self.P_2685_J
            elif arg.type == 1:
                J = self.P_2110_J
            Hp += self.TBP @ arg.orientation * J * arg.RPM * 2 * np.pi / 60 * arg.rotation
        Mg = -np.cross(angular_rates.reshape((1,3)), Hp.reshape((1,3))).reshape((3,1))
        return Mg



class pair(object):
    
    def __init__(self, propulsion_database, type, r, orientation, rotation):
        """
        Parameters
        ----------
        pair_type : 0 for MN801S - 26x8.5 prop
                    1 for AT7212 - 21x10 prop
        r         : 
                    position vector in type of numpy array
        orientation:
            orientation vector that pair has, relative to body
            
        rotation:  1 for CW
                  -1 for CCW

        Returns
        -------
        None.

        """
        self.object = propulsion_database
        self.type = type
        self.position = r
        self.orientation = orientation
        self.rotation = rotation
        
        self.RPM = 0.0
        self.dt = 0.0
        self.thrust = 0
        self.torque = 0
        self.thrust_vector = np.array([[0],[0],[0]])
        self.torque_vector = np.array([[0],[0],[0]])


    def calc_thrust_vector(self):
        self.thrust_vector =  self.thrust * self.orientation


    def calc_torque_vector(self):
        self.torque_vector =  self.torque * self.orientation * -self.rotation


    def get_forces_moments(self):
        self.calc_thrust_vector()
        self.calc_torque_vector()
        return self.thrust_vector, self.torque_vector


    def set_rpm(self, mach, rpm):
        if self.type == 0:
            self.RPM = rpm
            self.thrust = self.object.rpm_to_thrust_intrpl(self.RPM)
            self.torque = self.object.rpm_to_torque_intrpl(self.RPM)
        elif self.type == 1:
            self.RPM = rpm
            self.thrust = self.object.thrust_interpn2((mach, self.RPM))
            self.torque = self.object.torque_interpn2((mach, self.RPM))
        return
        
        
        
class propulsion:
    
    def __init__(self, g):
        
        self.g = g
        
        dir_path = os.path.dirname(os.path.realpath(__file__))
        
        
        prop_data = loadmat(dir_path + '/database/MN801S_26x85_PropulsionData_py.mat')  
        
        throttle_BP = np.array(prop_data['MN801S_26x85_PropulsionData']['throttle'][0][0]).flatten()
        RPM = np.array(prop_data['MN801S_26x85_PropulsionData']['RPM'][0][0]).flatten()
        thrust = np.array(prop_data['MN801S_26x85_PropulsionData']['thrust'][0][0]).flatten() / 1000 * self.g
        torque = np.array(prop_data['MN801S_26x85_PropulsionData']['torque'][0][0]).flatten()
        
        self.thrust_interpn1 = interp1d(throttle_BP, thrust, kind='linear', bounds_error=False, fill_value=None)
        self.torque_interpn1 = interp1d(throttle_BP, torque, kind='linear', bounds_error=False, fill_value=None)
        self.rpm_interpn1 = interp1d(throttle_BP, RPM, kind='linear', bounds_error=False, fill_value=None)

        self.throttle_interpn1 = interp1d(thrust, throttle_BP, kind='linear', bounds_error=False, fill_value=None)

        self.rpm_to_thrust_intrpl = interp1d(RPM, thrust, kind='linear', bounds_error=False, fill_value=None)
        self.rpm_to_torque_intrpl = interp1d(RPM, torque, kind='linear', bounds_error=False, fill_value=None)
        
        
        prop_data = loadmat(dir_path + '/database/AT7217_21x10_PropulsionData_py.mat')   
        
        throttle_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['throttle'][0][0]).flatten()
        RPM_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM_BP'][0][0]).flatten()
        RPM = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM'][0][0]).flatten()
        Mach_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['Mach_BP'][0][0]).flatten()
        thrust = np.array(prop_data['AT7217_21x10_PropulsionData']['thrust'][0][0])
        torque = np.array(prop_data['AT7217_21x10_PropulsionData']['torque'][0][0])
        
        self.rpm_interpn2 = interp1d(throttle_BP, RPM, kind='linear', bounds_error=False, fill_value=None)        
        self.thrust_interpn2 = RegularGridInterpolator((Mach_BP, RPM_BP), thrust, method='linear', bounds_error=False, fill_value=None)   
        self.torque_interpn2 = RegularGridInterpolator((Mach_BP, RPM_BP), torque, method='linear', bounds_error=False, fill_value=None)
        

    def propulsive_forces_moments(self, mach, *pairs):
        Fp = 0
        Mp = 0
        a = pairs
        
        if isinstance(pairs[0], Iterable):
            for pair in pairs[0]:
                b =pair
                self.calculate_forces_and_moments(mach, pair)
                Fp += pair.thrust_vector
                Mp += pair.torque_vector + np.cross(pair.position.reshape((1,3)), pair.thrust.reshape((1,3))).reshape((3,1))
        else:
            pair = pairs[0]
            self.calculate_forces_and_moments(mach, pair)
            Fp += pair.thrust_vector
            Mp += pair.torque + np.cross(pair.position.reshape((1,3)), pair.thrust.reshape((1,3))).reshape((3,1))

        return Fp, Mp
            
    
    def calculate_forces_and_moments(self, mach, pair):
        if pair.type == 0:
            pair.RPM = self.rpm_interpn1(pair.dt)
            pair.thrust_vector = self.thrust_interpn1(pair.dt) * pair.orientation
            pair.torque_vector = self.torque_interpn1(pair.dt) * pair.orientation * -pair.rotation
        elif pair.type == 1:
            pair.RPM = self.rpm_interpn2(pair.dt)
            pair.torque_vector = self.torque_interpn2((mach, pair.RPM)) * pair.orientation * -pair.rotation
            pair.thrust_vector = self.thrust_interpn2((mach, pair.RPM)) * pair.orientation





class Physics:

    def __init__(self):

        self.m = 30
        self.density = 1.225
        self.c = 343
        self.g = 9.8065
    
        Ixx = 8.638
        Iyy = 9.014
        Izz = 16.738
        Ixy = 0
        Ixz = 1.3
        Iyz = 0
        
        self.a = 635
        self.b = 530
        
        self.inertia_tensor = np.array([[Ixx,-Ixy,-Ixz],\
                                        [-Ixy, Iyy, -Iyz],\
                                        [-Ixz, -Iyz, Izz]], dtype = "float32")
            
            
        self.grav = gravitational(self.m, g=self.g)
        self.aero = aerodynamics()
        self.prop = propulsion(self.g)
        self.gyro = gyroscopic()
        
        r1 = np.array([[ 0.530, 0.635, 0]])
        r2 = np.array([[-0.530,-0.635, 0]])
        r3 = np.array([[ 0.530,-0.635, 0]])
        r4 = np.array([[-0.530, 0.635, 0]])
        rF = np.array([[-0.490, 0.0, 0.0]])

        orientation = np.array([[0],[0],[1]])

        self.motor1 = pair(self.prop, 0, r1, orientation, rotation =  1)
        self.motor2 = pair(self.prop, 0, r2, orientation, rotation =  1)
        self.motor3 = pair(self.prop, 0, r3, orientation, rotation = -1)
        self.motor4 = pair(self.prop, 0, r4, orientation, rotation = -1)
        
        orientation = np.array([[1],[0],[0]])

        self.motor5 = pair(self.prop, 1, rF, orientation, rotation = 1)
        
        self.motorset = self.motor1, self.motor2, self.motor3, self.motor4, self.motor5
        

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


        self.wind_param = np.array([[0],[0],[0]])
        self.body_rates = np.array([[0],[0],[0]])
        self.cont_surf = np.array([[0],[0],[0]])
        self.euler_angles = np.array([[0],[0],[0]])

        self.delevator = 0
        self.drudder = 0
        self.daileron = 0

        self.thrust_coefficient = 0.0245
        self.thrust_coefficientFW = 0.102
        self.hub_coefficient = 0.01
        self.hub_coefficient_FW = 0.001
        self.CG = np.array([[0],[0],[0.016389]])

        self.wrench = Wrench()

        ##Ros Initialization
        rospy.init_node('physics')
        self.pub_force_moment = rospy.Publisher('/ciconia/ForcesAndMoments', Wrench, queue_size=5)
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_handler)
        rospy.Subscriber("/ciconia/joint_states", JointState, self.joint_states_handler)
        rospy.Subscriber("/ciconia/joint_vel", Float64MultiArray, self.motor_speed_handler)



    def joint_states_handler(self, data):
        self.cont_surf[0,0] = data.position[0]                         #elevator deflection 
        self.cont_surf[0,0] = data.position[3] - data.position[4]      #aileron deflection
        self.cont_surf[0,0] = data.position[2]                         #rudder deflection
        return
    
    
    def motor_speed_handler(self, data):
        self.motor1.set_rpm(self.mach, data.data[0] * 30 / np.pi)
        self.motor2.set_rpm(self.mach, data.data[1] * 30 / np.pi)
        self.motor3.set_rpm(self.mach, data.data[2] * 30 / np.pi)
        self.motor4.set_rpm(self.mach, data.data[3] * 30 / np.pi)
        self.motor5.set_rpm(self.mach, data.data[4] * 30 / np.pi)
        return


    def pose_handler(self, data):

        body_index = data.name.index('ciconia::body')

        r_body = data.pose[body_index].position

        orientation = data.pose[body_index].orientation
        self.phi, self.theta, self.psi = quaternion_to_euler_angle(orientation.w, orientation.x, orientation.y, orientation.z)
        self.phi_ned = self.phi
        self.theta_ned = -self.theta
        self.psi_ned = -self.psi


        ori_vector = np.array([[self.phi], [self.theta], [self.psi]])
        self.euler_angles = np.array([[self.phi], [-self.theta], [-self.psi]])
        vel_vector = np.array([[data.twist[body_index].linear.x],[data.twist[body_index].linear.y],[data.twist[body_index].linear.z]])
        vel_vector = earth2body_transformation(ori_vector, vel_vector)

        self.u_ned = vel_vector[0]
        self.v_ned = -vel_vector[1]
        self.w_ned = -vel_vector[2]
        
        rates = np.array([[data.twist[body_index].angular.x],[data.twist[body_index].angular.y],[data.twist[body_index].angular.z]])
        rates = earth2body_transformation(ori_vector, rates)

        self.p_ned = rates[0,0]
        self.q_ned = -rates[1,0]
        self.r_ned = -rates[2,0]

        self.body_rates[0,0] = self.p_ned
        self.body_rates[1,0] = self.p_ned
        self.body_rates[2,0] = self.r_ned

        #wind axis parameters, Wind Velocity, Angle of Attack, Side slip angle
        self.Vt    = math.sqrt(self.u_ned**2 + self.v_ned**2 + self.w_ned**2)
        self.alpha = math.atan(self.w_ned/self.u_ned)
        self.beta  = math.asin(self.v_ned/self.Vt)

        self.mach = self.Vt / self.c


        if self.Vt > 5:
            self.wind_param[0,0] = self.Vt
            self.wind_param[1,0] = self.alpha
            self.wind_param[2,0] = self.beta

        if self.alpha > self.aero.alpha_max:
            self.alpha = self.aero.alpha_max
        elif self.alpha < self.aero.alpha_min:
            self.alpha = self.aero.alpha_min

        if self.beta > self.aero.beta_max:
            self.beta = self.aero.beta_max
        elif self.beta < self.aero.beta_min:
            self.beta = self.aero.beta_min

        if self.mach > self.aero.mach_max:
            self.mach = self.mach_max
        elif self.mach < self.aero.mach_min:
            self.mach = self.aero.mach_min



    def propulsive_forces_and_moments(self):
        Fp1, Mp1 = self.motor1.get_forces_moments()
        Fp2, Mp2 = self.motor2.get_forces_moments()
        Fp3, Mp3 = self.motor3.get_forces_moments()
        Fp4, Mp4 = self.motor4.get_forces_moments()
        Fp5, Mp5 = self.motor5.get_forces_moments()
        return Fp1 + Fp2 + Fp3 + Fp4 + Fp5, Mp1 + Mp2 + Mp3 + Mp4 + Mp5


    def publish_forces_and_moments(self):
        
        prop_forces, prop_moments = self.propulsive_forces_and_moments()
        aero_forces, aero_moments = self.aero.aerodynamic_model(self.wind_param, self.body_rates, self.cont_surf)
        grav_forces = self.grav.gravitational_forces(self.euler_angles)
        gyro_moment = self.gyro.gyroscopic_moment(self.body_rates, self.motorset)


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
        moments_earth = prop_moments + aero_moments + gyro_moment
        
        self.wrench.force.x = forces_earth[0,0]
        self.wrench.force.y = forces_earth[1,0]
        self.wrench.force.z = forces_earth[2,0]

        self.wrench.torque.x = moments_earth[0,0]
        self.wrench.torque.y = moments_earth[1,0]
        self.wrench.torque.z = moments_earth[2,0]

        self.pub_force_moment.publish(self.wrench)



    def physics_loop(self):
        r = rospy.Rate(250)
        print('Aerodynamic and Propulsive Forces and Moments are being published with Gazebo!')
        while not rospy.is_shutdown():
            self.publish_forces_and_moments()

            r.sleep()



def main():
    vtol = Physics()
    vtol.physics_loop()



if __name__ == '__main__':
    main()
