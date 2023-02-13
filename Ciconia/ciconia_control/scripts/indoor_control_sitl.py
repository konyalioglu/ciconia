#! /usr/bin/python3

#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy

from geometry_msgs.msg import Vector3Stamped, PoseStamped, TwistStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import RCOut, State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

from utils import *
from pid import pid
from model_predictive_controller import Model_Predictive_Control



class indoorController:

    def __init__(self):
    
        #Default Parameters
        self.controller_rate = 50
        self.controller_type = 'PID'

        #MAVROS STATES
        self.is_armed = False
        self.is_connected = False
        self.is_guided = False
        self.is_manual_input = False
        self.system_status = False
        self.mode = 'STABILIZE'

        #Outputs
        self.aileron = 1000
        self.elevator = 1000
        self.throttle = 1000
        self.rudder = 1000
        self.motor1 = 1000
        self.motor2 = 1000
        self.motor3 = 1000
        self.motor4 = 1000
        self.RCIN9  = 1000

        self.set_accel = Vector3Stamped()
        self.set_point = PositionTarget()
        self.set_attitude = AttitudeTarget()

        self.set_point.coordinate_frame = self.set_point.FRAME_BODY_NED
        self.set_point.type_mask  = self.set_point.IGNORE_PX + self.set_point.IGNORE_PY + self.set_point.IGNORE_PZ 
        self.set_point.type_mask += self.set_point.IGNORE_VX + self.set_point.IGNORE_VY + self.set_point.IGNORE_VZ 
        self.set_point.type_mask += self.set_point.IGNORE_YAW
        self.set_point.yaw_rate = 0.0

        self.set_attitude.type_mask =  self.set_attitude.IGNORE_ROLL_RATE + self.set_attitude.IGNORE_PITCH_RATE + self.set_attitude.IGNORE_YAW_RATE + self.set_attitude.IGNORE_ATTITUDE

        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'altControllerNode'       
        self.initialize_model()

        #Controller Parameters
        self.position_controller = pid(1.0, 0.0, 0.0)
        self.velocity_controller = pid(3.0, 0.0, 0.0)
        self.acceleration_controller = pid(0.5, 0.8, 0.0)
        self.ref_alt = 0.1


        self.A_alt = np.array([[0, 1],[0, 0]])
        self.B_alt = np.array([[0],[1]])
        self.C_alt = np.array([[1, 0],[0, 1]])
        Np = 40
        Nc = 4
        Q  = np.array([2, 20])
        R  = np.array([0.1])

        x_mins = -np.array([[10],[10]])
        x_maxs = np.array([[10],[10]])
        x_cons =  np.concatenate((x_mins, x_maxs), axis=0)
        u_mins = -np.array([[10]])
        u_maxs = np.array([[10]])
        u_cons =  np.concatenate((u_mins, u_maxs), axis=0)
        deltau_mins = -np.array([[0.1]])
        deltau_maxs = np.array([[0.1]])
        deltau_cons =  np.concatenate((deltau_mins, deltau_maxs), axis=0)

        self.inner_acc_controller = pid(0.8, 0.1, 0.01, output_constraints = [-90, 90], anti_windup = 80)
        self.mpc_alt = Model_Predictive_Control(self.A_alt, self.B_alt, self.C_alt, Np, Nc, Q, R, 1/self.controller_rate, 'Regulator')
        self.mpc_alt.initialize_model_contraints(x_cons, u_cons, deltau_cons)
        self.mpc_alt.initialize_mpc_controller()
        self.mpc_alt.unconstrained_mpc_gain()

        #Global Variables
        self.g = 9.81

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0
        
        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.z = 0.0
        self.z_dot = 0.0

        self.vel_data_on = False
        self.pos_data_on = False


        #Initialization
        rospy.init_node(self._node_name)

        rospy.Subscriber('/mavros/rc/out', RCOut, self._rc_out_handler)
        rospy.Subscriber('/mavros/state', State, self._mavros_states_handler)        
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_handler)  
        rospy.Subscriber('/alt_est/states', Float64MultiArray, self._estimator_handler)   
        rospy.Subscriber('/mavros/setpoint_raw/target_local', PositionTarget, self._set_point_callback)    
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self._pose_handler)    
        rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self._velocity_handler)    

        self._set_accel_pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=5)             
        self._set_local_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=5)
        self._set_attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=5)  

        self._timer = rospy.Timer(rospy.Duration(1/self.controller_rate), self._timer)   
                


    def get_param(self, param_name, default):
        try:
            param = rospy.get_param(param_name)
            rospy.logwarn("Found parameter: %s, value: %s"%(param_name, str(param)))
        except KeyError:
            param = default
            rospy.logwarn("Cannot find value for parameter: %s, assigning "	"default: %s"%(param_name, str(param)))
        return param


    def initialize_model(self):
        self.controller_rate = self.get_param('/controller_rate', self.controller_rate)
        self.controller_type = self.get_param('/controller_type', self.controller_type)
        rospy.sleep(10)
        self.set_guided_nogps()
        self.arming()


    def arming(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_drone = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_drone(True)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def set_guided_nogps(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            set_guided = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            set_guided(base_mode = 0, custom_mode='GUIDED_NOGPS')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def _set_point_callback(self, msg):
        print(msg.acceleration_or_force.z)

    
    def _pose_handler(self, msg):
        if self.pos_data_on == False:
            self.pos_data_on = True
            self.home_z = -msg.pose.position.z
        self.z = -msg.pose.position.z
        

    def _velocity_handler(self, msg):
        if self.vel_data_on == False:
            self.vel_data_on = True
        self.z_dot = -msg.twist.linear.z


    def _timer(self, event):

        if self.is_armed and self.mode == 'GUIDED_NOGPS' and self.vel_data_on and self.pos_data_on:
            if self.controller_type == 'PID':
                vz_signal = self.position_controller.calculate_control_input(-self.ref_alt + self.home_z, self.z, 1/self.controller_rate)
                az_signal = self.velocity_controller.calculate_control_input(vz_signal, self.z_dot, 1/self.controller_rate)
                throttle = self.acceleration_controller.calculate_control_input(az_signal, self.az, 1/self.controller_rate)
                self.set_attitude.thrust = (-throttle + 30.0) / 100
                self._set_attitude_pub.publish(self.set_attitude)
                print('Throttle: ' + str(throttle) + '    az_signal: ' + str(az_signal) + '   az: ' + str(self.az) + '   zdot: ' + str(self.z_dot) + '   z: ' + str(self.z - self.home_z))


            elif self.controller_type == 'MPC':
                xk = np.array([[self.z],[self.z_dot]])
                ref_s_vector = np.array([[-self.ref_alt + self.home_z],[0]])

                #u = self.mpc_alt.calculate_mpc_unconstraint_input(xk - ref_s_vector)

                u = self.mpc_alt.calculate_control_input(xk - ref_s_vector, option = 'Hildreth')
                throttle = self.inner_acc_controller.calculate_control_input(-u[0,0], self.az, 1/self.controller_rate)
                print('x: ' + str(xk) + '    ref: ' + str(ref_s_vector) + '    diff: ' + str(xk - ref_s_vector))

                print('Throttle: ' + str(throttle) + '    umpc_a: ' + str(u[0,0]) + '   az: ' + str(self.az) + '   zdot: ' + str(self.z_dot) + '   z: ' + str(self.z - self.home_z))

                self.set_attitude.thrust = (throttle + 30.0) / 100
                #self.set_attitude.thrust = (throttle) / 100
                self._set_attitude_pub.publish(self.set_attitude)

        
    def _rc_out_handler(self, msg):
        self.aileron = msg.channels[0]
        self.elevator = msg.channels[1]
        self.throttle = msg.channels[2]
        self.rudder = msg.channels[3]
        self.motor1 = msg.channels[4]
        self.motor2 = msg.channels[5]
        self.motor3 = msg.channels[6]
        self.motor4 = msg.channels[7]
        self.RCIN9  = msg.channels[8]


    def _mavros_states_handler(self, msg):
        self.is_connected = msg.connected
        self.is_armed = msg.armed
        self.is_guided = msg.guided
        self.is_manual_input = msg.manual_input
        self.mode = msg.mode
        self.system_status = msg.system_status


    def _imu_handler(self, msg):
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.phi, self.theta, self.psi = quaternion_to_euler_angle(qw, qx, qy, qz)
        gx, gy, gz = quaternion_to_gravity(qx, qy, qz, qw)

        self.ax = -msg.linear_acceleration.x + gx * self.g
        self.ay = -msg.linear_acceleration.y + gy * self.g
        self.az = -msg.linear_acceleration.z + gz * self.g

        self.p = msg.angular_velocity.x
        self.q = msg.angular_velocity.y
        self.r = msg.angular_velocity.z


    def _estimator_handler(self, msg):
        # self.z = msg.data[0]
        # self.zdot = msg.data[1]
        return



if __name__ == '__main__':

    inCont = indoorController()

    rospy.loginfo('ALTITUDE ESTIMATOR HAS BEEN ACTIVATED')
    
    rospy.spin()




