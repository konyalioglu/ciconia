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
		
		self.prop1_vel = 0
		self.prop2_vel = 0
		self.prop3_vel = 0
		self.prop4_vel = 0
		self.prop5_vel = 0
		self.propFW_vel = 0

		self.thrust_coefficient = 0.0245
		self.thrust_coefficientFW = 0.0118
		self.CG = np.array([[0],[0],[0.016389]])

		self.density = 1.225
		self.speedofsound = 343

		self.r1 = np.array([[0.530], [-0.635], [0.0305]]) - self.CG
		self.r2 = np.array([[-0.530], [0.635], [0.0305]]) - self.CG
		self.r3 = np.array([[0.530], [0.635], [0.0305]]) - self.CG
		self.r4 = np.array([[-0.530], [-0.635], [0.0305]]) - self.CG

		self.wrench = Wrench()
		
		##Ros Initialization
		rospy.init_node('physics')
		self.pub_force_moment = rospy.Publisher('/vtol_v6/ForcesAndMoments', Wrench, queue_size=5)
		rospy.Subscriber("/gazebo/link_states", LinkStates, self.pose_handler)
		rospy.Subscriber("/vtol_v6/joint_states", JointState, self.joint_states_handler)
	

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


	def joint_states_handler(self, data):

		self.prop1_vel = data.velocity[2]
		self.prop2_vel = data.velocity[1]
		self.prop3_vel = data.velocity[3]
		self.prop4_vel = data.velocity[0]
		self.prop5_vel = data.velocity[5]

		self.daileron = data.position[8] - data.position[9]
		self.delevator = data.position[4]
		self.drudder = data.position[6]

		return
		
	
	def pose_handler(self, data):

		body_index = data.name.index('vtol_v6::body')		
		
		r_body = data.pose[body_index].position
		
		orientation = data.pose[body_index].orientation
		self.phi, self.theta, self.psi = self.quaternion_to_euler_angle(orientation.w, orientation.x, orientation.y, orientation.z)	
		self.phi_ned = self.phi
		self.theta_ned = -self.theta
		self.psi_ned = -self.psi

		self.u = data.twist[body_index].linear.x
		self.v = data.twist[body_index].linear.y
		self.w = data.twist[body_index].linear.z

		self.u_ned = data.twist[body_index].linear.x
		self.v_ned = -data.twist[body_index].linear.y
		self.w_ned = -data.twist[body_index].linear.z

		self.p = data.twist[body_index].angular.x
		self.q = data.twist[body_index].angular.y
		self.r = data.twist[body_index].angular.z

		self.p_ned = data.twist[body_index].angular.x
		self.q_ned = -data.twist[body_index].angular.y
		self.r_ned = -data.twist[body_index].angular.z

		#wind axis parameters, Wind Velocity, Angle of Attack, Side slip angle
		self.Vt    = math.sqrt(self.u_ned**2 + self.v_ned**2 + self.w_ned**2)
		self.alpha = math.atan2(self.w_ned, self.u_ned)
		self.beta  = math.asin(self.v_ned/self.Vt)
		
		self.mach = self.Vt / self.speedofsound

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
		
		#prop_force_body = prop1_force_body + prop2_force_body + prop3_force_body + prop4_force_body + prop5_force_body
		#prop_moment_body = prop1_moment_body + prop2_moment_body + prop3_moment_body + prop4_moment_body
		
		prop1_force_earth = self.body2earth_transformation(ori_vector, prop1_force_body)
		prop2_force_earth = self.body2earth_transformation(ori_vector, prop2_force_body)
		prop3_force_earth = self.body2earth_transformation(ori_vector, prop3_force_body)
		prop4_force_earth = self.body2earth_transformation(ori_vector, prop4_force_body)

		prop1_moment_earth = self.body2earth_transformation(ori_vector, prop1_moment_body)
		prop2_moment_earth = self.body2earth_transformation(ori_vector, prop2_moment_body)
		prop3_moment_earth = self.body2earth_transformation(ori_vector, prop3_moment_body)
		prop4_moment_earth = self.body2earth_transformation(ori_vector, prop4_moment_body)

		forces_earth  = prop1_force_earth + prop2_force_earth + prop3_force_earth + prop4_force_earth
		moments_earth = prop1_moment_earth + prop2_moment_earth + prop3_moment_earth + prop4_moment_earth
		
		return forces_earth, moments_earth

	def aerodynamic_data_initialization(self):
		dir_path = os.path.dirname(os.path.realpath(__file__))
		aero_data = loadmat(dir_path + '/data.mat')

		## Aerodynamic Parameters
		self.sref = float(aero_data['data']['sref'][0])
		self.cbar = float(aero_data['data']['cbar'][0])
		self.blref = float(aero_data['data']['blref'][0])

		## Breakpoints
		alpha = np.array(aero_data['data']['alpha'][0][0][0], dtype='float32') * np.pi / 180
		nalpha = float(aero_data['data']['nalpha'][0])

		mach = np.array(aero_data['data']['mach'][0][0][0], dtype='float32')
		nmach = float(aero_data['data']['nmach'][0])

		delta = (np.array(aero_data['data']['delta'][0][0][0], dtype='float32') * np.pi / 180)
		deltal = np.array(aero_data['data']['deltal'][0][0][0], dtype='float32') * np.pi / 180
		deltar = np.array(aero_data['data']['deltar'][0][0][0], dtype='float32') * np.pi / 180
		rdelta = np.array(aero_data['data']['rdelta'][0][0][0], dtype='float32') * np.pi / 180

		## Drag Force Coefficients
		cd = np.array(aero_data['data']['cd'][0][0], dtype='float32')
		self.cd_interpn = RegularGridInterpolator((alpha,mach), cd, method='linear', bounds_error=False, fill_value=nan)

		## Lift Force Coefficients
		cl = np.array(aero_data['data']['cl'][0][0], dtype='float32')
		cla = np.array(aero_data['data']['cla'][0][0], dtype='float32')
		clq = np.array(aero_data['data']['clq'][0][0], dtype='float32')
		clad = np.array(aero_data['data']['clad'][0][0], dtype='float32')
		dclde = np.array(aero_data['data']['dcl_sym'][0][0], dtype='float32')

		self.cl_interpn = RegularGridInterpolator((alpha,mach), cl, method='linear', bounds_error=False, fill_value=nan)
		self.clq_interpn = RegularGridInterpolator((alpha,mach), clq, method='linear', bounds_error=False, fill_value=nan)
		self.clad_interpn = RegularGridInterpolator((alpha,mach), clad, method='linear', bounds_error=False, fill_value=nan)
		self.dclde_interpn = RegularGridInterpolator((delta,mach), dclde, method='linear', bounds_error=False, fill_value=nan)
		
		xcp = np.array(aero_data['data']['xcp'][0][0], dtype='float32')

		## Force Coefficient on y-axis
		cyb = np.array(aero_data['data']['cyb'][0][0], dtype='float32')
		cyp = np.array(aero_data['data']['cyp'][0][0], dtype='float32')
		cydr = np.array(aero_data['data']['drcy'][0][0], dtype='float32')

		self.cyb_interpn = RegularGridInterpolator((alpha,mach), cyb, method='linear', bounds_error=False, fill_value=nan)
		self.cyp_interpn = RegularGridInterpolator((alpha,mach), cyp, method='linear', bounds_error=False, fill_value=nan)
		self.cydr_interpn = RegularGridInterpolator((alpha,rdelta), cydr, method='linear', bounds_error=False, fill_value=nan)
				
		## Rolling Moment Coefficients
		clb = np.array(aero_data['data']['clb'][0][0], dtype='float32')
		clp = np.array(aero_data['data']['clp'][0][0], dtype='float32')
		clr = np.array(aero_data['data']['clr'][0][0], dtype='float32')
		clda = np.array(aero_data['data']['clroll'][0][0], dtype='float32')
		cldr = np.array(aero_data['data']['drcl'][0][0], dtype='float32')

		self.clb_interpn = RegularGridInterpolator((alpha,mach), clb, method='linear', bounds_error=False, fill_value=nan)
		self.clp_interpn = RegularGridInterpolator((alpha,mach), clp, method='linear', bounds_error=False, fill_value=nan)
		self.clr_interpn = RegularGridInterpolator((alpha,mach), clr, method='linear', bounds_error=False, fill_value=nan)
		self.clda_interpn = RegularGridInterpolator((deltal-deltar,mach), clda, method='linear', bounds_error=False, fill_value=nan)
		self.cldr_interpn = RegularGridInterpolator((alpha,rdelta), cldr, method='linear', bounds_error=False, fill_value=nan)

		## Pitching Moment Coefficients
		cm = np.array(aero_data['data']['cm'][0][0], dtype='float32')
		cma = np.array(aero_data['data']['cma'][0][0], dtype='float32')
		cmq = np.array(aero_data['data']['cmq'][0][0], dtype='float32')
		dcmde = np.array(aero_data['data']['dcm_sym'][0][0], dtype='float32')

		self.cm_interpn = RegularGridInterpolator((alpha,mach), cm, method='linear', bounds_error=False, fill_value=nan)
		self.cmq_interpn = RegularGridInterpolator((alpha,mach), cmq, method='linear', bounds_error=False, fill_value=nan)
		self.dcmde_interpn = RegularGridInterpolator((delta,mach), dcmde, method='linear', bounds_error=False, fill_value=nan)
		

		## Yawing Moment Coefficients
		cnb = np.array(aero_data['data']['cnb'][0][0], dtype='float32')
		cnp = np.array(aero_data['data']['cnp'][0][0], dtype='float32')
		cnr = np.array(aero_data['data']['cnr'][0][0], dtype='float32')
		cnda = np.array(aero_data['data']['cn_asy'][0][0], dtype='float32')
		cndr = np.array(aero_data['data']['drcn'][0][0], dtype='float32')

		self.cnb_interpn = RegularGridInterpolator((alpha,mach), cnb, method='linear', bounds_error=False, fill_value=nan)
		self.cnp_interpn = RegularGridInterpolator((alpha,mach), cnp, method='linear', bounds_error=False, fill_value=nan)
		self.cnr_interpn = RegularGridInterpolator((alpha,mach), cnr, method='linear', bounds_error=False, fill_value=nan)
		self.cnda_interpn = RegularGridInterpolator((alpha,deltal-deltar,mach), cnda, method='linear', bounds_error=False, fill_value=nan)
		self.cndr_interpn = RegularGridInterpolator((alpha,rdelta), cndr, method='linear', bounds_error=False, fill_value=nan)

		return

	
	def aerodynamic_forces_and_moments(self):

		cd = self.cd_interpn((self.alpha, self.mach))

		cl = self.cl_interpn((self.alpha, self.mach))
		clq = self.clq_interpn((self.alpha, self.mach))
		dclde = self.cm_interpn((self.delevator, self.mach))

		cyb = self.cyb_interpn((self.alpha, self.mach))
		cyp = self.cyp_interpn((self.alpha, self.mach))
		cydr = self.cydr_interpn((self.alpha,self.drudder))

		clb = self.clb_interpn((self.alpha, self.mach))
		clp = self.clp_interpn((self.alpha, self.mach))
		clr = self.clr_interpn((self.alpha, self.mach))
		clda = self.clda_interpn((self.daileron, self.mach))
		cldr = self.cldr_interpn((self.alpha, self.drudder))
		
		cm = self.cm_interpn((self.alpha, self.mach))
		cmq = self.cmq_interpn((self.alpha, self.mach))
		cmde = self.dcmde_interpn((self.delevator, self.mach))

		cnb = self.cnb_interpn((self.alpha, self.mach))
		cnp = self.cnp_interpn((self.alpha, self.mach))
		cnr = self.cnr_interpn((self.alpha, self.mach))
		cnda = self.cnda_interpn((self.daileron, self.mach))
		cndr = self.cndr_interpn((self.alpha, self.drudder))

		dyn_press = self.Vt**2 * self.density / 2

		L = (cl + self.q_ned * self.cbar / (2 * self.Vt) * clq + clde) * dyn_press * self.sref
		D = cd * dyn_press * self.sref

		X = -D * np.cos(self.alpha) + L * np.sin(self.alpha)
		Z = -D * np.sin(self.alpha) + L * np.cos(self.alpha)

		Y = (self.beta * cyb + self.blref * self.p_ned / (2 * self.Vt) * cyp + 2 * cydr) * dyn_press * self.sref

		L = (self.beta * clb + self.blref * self.p_ned / (2 * self.Vt) * clp + self.blref * self.r_ned / (2 * self.Vt) * clr + clda + 2 * cldr) * self.sref * self.blref * dyn_press

		M = (cm + cmde + self.cbar * self.q_ned / (2 * self.Vt) * cmq) * self.sref * self.cbar * dyn_press

		N = (self.beta * cnb + self.blref * self.p_ned / (2 * self.Vt) * cnp + self.blref * self.r_ned / (2 * self.Vt) * cnr + cnda + 2 * cndr) * self.sref * self.blref * dyn_press

		ori_vector = np.array([[self.phi], [self.theta], [self.psi]])

		aero_forces_earth = self.body2earth_transformation(ori_vector, np.array([[X],[-Y],[-Z]]))
		aero_moments_earth = self.body2earth_transformation(ori_vector, np.array([[L],[-M],[-N]]))

		return aero_forces_earth, aero_moments_earth

	def publish_forces_and_moments(self):

		prop_forces, prop_moments = self.propulsive_forces_and_moments()
		#aero_forces, aero_moments = self.aerodynamic_forces_and_moments()

		forces_earth = prop_forces #+ aero_forces
		moments_earth = prop_moments #+ aero_moments
		
		self.wrench.force.x = forces_earth[0,0]
		self.wrench.force.y = forces_earth[1,0]
		self.wrench.force.z = forces_earth[2,0]

		self.wrench.torque.x = moments_earth[0,0]
		self.wrench.torque.y = moments_earth[1,0]
		self.wrench.torque.z = moments_earth[2,0]
		self.pub_force_moment.publish(self.wrench)

		return
			
	
	def control_loop(self):
		r = rospy.Rate(250)
		while not rospy.is_shutdown():
			self.publish_forces_and_moments()
			
			r.sleep()	
		
	
			
			

if __name__ == '__main__':
	
	vtol = Physics()

	vtol.control_loop()
	#vtol.constraint_mpc_control_loop()
	#try:
		#vtol.control_loop()
	#except rospy.ROSInterruptException:
		#pass
	#rospy.spin()
	
	

