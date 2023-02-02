#! /usr/bin/python3



import numpy as np
from numpy import nan
import rospy

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from mavros_msgs.msg import RCOut, State, PositionTarget, AttitudeTarget, RCIn
from mavros_msgs.srv import CommandBool, SetMode
from ciconia_msgs.msg import altPIDControl

from utils import *
from pid import pid



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
        self.pid_data = altPIDControl()

        self.set_point.coordinate_frame = self.set_point.FRAME_BODY_NED
        self.set_point.type_mask  = self.set_point.IGNORE_PX + self.set_point.IGNORE_PY + self.set_point.IGNORE_PZ 
        self.set_point.type_mask += self.set_point.IGNORE_VX + self.set_point.IGNORE_VY + self.set_point.IGNORE_VZ 
        self.set_point.type_mask += self.set_point.IGNORE_YAW
        self.set_point.yaw_rate = 0.0

        self.set_attitude.type_mask =  self.set_attitude.IGNORE_ROLL_RATE + self.set_attitude.IGNORE_PITCH_RATE + self.set_attitude.IGNORE_YAW_RATE + self.set_attitude.IGNORE_ATTITUDE


        #Global Variables
        self.g = 9.81

        self.number_of_acc_samples = 100
        self.acc_sample_counter = 0
        self.sum_acc = 0
        self.gravity_calibration = False

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

        self.init_home = False
        self.init_throttle = False
        self.throttle_ref = 0.0

        self.home_z = 0.0

        self.pos_kp = 1.5
        self.pos_ki = 0.0
        self.pos_kd = 0.0

        self.vel_kp = 15.0
        self.vel_ki = 0.002
        self.vel_kd = 0.0

        self.acc_kp = 0.8
        self.acc_ki = 0.1
        self.acc_kd = 0.01

        self.position_controller = pid(self.pos_kp, self.pos_ki, self.pos_kd)
        self.velocity_controller = pid(self.vel_kp, self.vel_ki, self.vel_kd, anti_windup = 0.2)
        self.acceleration_controller = pid(self.acc_kp, self.acc_ki, self.acc_kd, output_constraints = [-95, -5], anti_windup = 80)

        self.ref_alt = 0.3

        self.pid_data.alt_to_vel_p = self.acc_kp
        self.pid_data.alt_to_vel_i = self.acc_ki
        self.pid_data.alt_to_vel_d = self.acc_kd

        self.pid_data.vel_to_acc_p = self.acc_kp
        self.pid_data.vel_to_acc_i = self.acc_ki
        self.pid_data.vel_to_acc_d = self.acc_kd

        self.pid_data.acc_to_thrust_p = self.acc_kp
        self.pid_data.acc_to_thrust_i = self.acc_ki
        self.pid_data.acc_to_thrust_d = self.acc_kd

        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'altControllerNode'       
        self.initialize_model()

        #Initialization
        rospy.init_node(self._node_name)

        rospy.Subscriber('/mavros/rc/out', RCOut, self._rc_out_handler)
        rospy.Subscriber('/mavros/rc/in', RCIn, self._rc_in_handler)
        rospy.Subscriber('/mavros/state', State, self._mavros_states_handler)        
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_handler)  
        rospy.Subscriber('/alt_est/states', Float64MultiArray, self._estimator_handler)   
        rospy.Subscriber('/mavros/setpoint_raw/target_local', PositionTarget, self._set_point_callback)     

        self._set_accel_pub = rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=5)             
        self._set_local_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=5)
        self._set_attitude_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=5)  
        self._set_pid_data_pub = rospy.Publisher('/pid/data', altPIDControl, queue_size=5)  

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


    def _timer(self, event):

        if self.is_armed and self.mode == 'GUIDED_NOGPS' and self.init_home == False and self.init_throttle == False:
            self.init_home = True
            self.init_throttle = True
            self.throttle_ref = self.throttle
            self.home_z = self.z

        if self.is_armed and self.mode == 'GUIDED_NOGPS' and self.init_home and self.init_throttle:
            if self.controller_type == 'PID':
                vz_signal = self.position_controller.calculate_control_input(-self.ref_alt + self.home_z, self.z, 1/self.controller_rate)
                az_signal = self.velocity_controller.calculate_control_input(vz_signal, self.z_dot, 1/self.controller_rate)
                throttle = self.acceleration_controller.calculate_control_input(az_signal, self.az, 1/self.controller_rate)

                self.set_attitude.thrust = (-throttle + self.throttle_ref) / 100
                self._set_attitude_pub.publish(self.set_attitude)

                self.pid_data.reference_altitude = -self.ref_alt
                self.pid_data.altitude = self.z
                self.pid_data.home_altitude = self.home_z

                self.pid_data.velocity_signal = vz_signal
                self.pid_data.velocity = self.z_dot

                self.pid_data.acceleration_signal = az_signal
                self.pid_data.acceleration = self.az

                self.pid_data.throttle = (-throttle + self.throttle_ref) / 100
                self.pid_data.reference_throttle = self.throttle_ref

                self._set_pid_data_pub.publish(self.pid_data)
                print('Throttle: ' + str(self.set_attitude.thrust) + '   az: ' + str(self.az) + '   zdot: ' + str(self.z_dot) + '   z: ' + str(self.z - self.home_z))


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


    def _rc_in_handler(self, msg):
        self.set_point_settling = msg.channels[5]
        self.throttle = (msg.channels[2] - 1000) / 10
        if self.set_point_settling < 1300:
            self.ref_alt = 0.3
        elif self.set_point_settling > 1300 and self.set_point_settling < 1700:
            self.ref_alt = 0.0
        
        #print('Throttle: ' + str(self.throttle) + '   az: ' + str(self.ref_alt))


    def _mavros_states_handler(self, msg):
        self.is_connected = msg.connected
        self.is_armed = msg.armed
        self.is_guided = msg.guided
        self.is_manual_input = msg.manual_input
        self.mode = msg.mode
        self.system_status = msg.system_status


    def _imu_handler(self, msg):
        if self.acc_sample_counter + 1 != self.number_of_acc_samples and self.gravity_calibration == False:
            self.acc_sample_counter += 1
            self.sum_acc += np.sqrt(msg.linear_acceleration.x ** 2 + msg.linear_acceleration.y ** 2 + msg.linear_acceleration.z ** 2)

        elif self.acc_sample_counter + 1 == self.number_of_acc_samples and self.gravity_calibration == False:
            self.g = self.sum_acc / self.number_of_acc_samples
            self.acc_sample_counter += 1
            self.gravity_calibration = True

        else:
            qx = msg.orientation.x
            qy = msg.orientation.y
            qz = msg.orientation.z
            qw = msg.orientation.w
            self.phi, self.theta, self.psi = quaternion_to_euler_angle(qw, qx, qy, qz)
            gx, gy, gz = quaternion_to_gravity(qx, qy, qz, qw)

            self.ax = -msg.linear_acceleration.x + gx * self.g
            self.ay = -msg.linear_acceleration.y + gy * self.g
            self.az = -msg.linear_acceleration.z + gz * self.g - (-0.09553390312093057)

            self.p = msg.angular_velocity.x
            self.q = msg.angular_velocity.y
            self.r = msg.angular_velocity.z


    def _estimator_handler(self, msg):
        self.z = -msg.data[0]
        self.z_dot = -msg.data[1]



if __name__ == '__main__':

    inCont = indoorController()

    rospy.loginfo('ALTITUDE CONTROLLER HAS BEEN ACTIVATED')
    
    rospy.spin()




