#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy, os, sys, time
import math

from geometry_msgs.msg import Pose, Vector3, Quaternion
from sensor_msgs.msg import Imu, NavSatFix, Temperature, FluidPressure
from std_msgs.msg import Float64MultiArray, Float32

from utils import *

from xsens_msgs.msg import sensorSample, baroSample, gnssSample
from xsens_msgs.msg import positionEstimate, velocityEstimate, orientationEstimate



class xsensModel:

    def __init__(self):
    
        self.x_ned_w = 0
        self.y_ned_w = 0
        self.z_ned_w = 0

        self.phi_ned = 0
        self.theta_ned = 0
        self.psi_ned = 0

        self.euler_angles = np.array([[0],[0],[0]])

        self.gx = 0
        self.gy = 0
        self.gz = 0   

        self.gravity_vec = np.array([[0],[0],[0]])

        self.u_ned_w = 0
        self.v_ned_w = 0
        self.w_ned_w = 0

        self.du_ned_w = 0
        self.dv_ned_w = 0
        self.dw_ned_w = 0

        self.p_ned_w = 0
        self.q_ned_w = 0
        self.r_ned_w = 0

        self.x_ned_b = 0
        self.y_ned_b = 0
        self.z_ned_b = 0

        self.u_ned_b = 0
        self.v_ned_b = 0
        self.w_ned_b = 0

        self.du_ned_b = 0
        self.dv_ned_b = 0
        self.dw_ned_b = 0

        self.p_ned_b = 0
        self.q_ned_b = 0
        self.r_ned_b = 0

        self.quat_x = 0
        self.quat_y = 0
        self.quat_z = 0
        self.quat_w = 0   


        #Default Parameters
        self.rate = 200
        
        self.gnss_rate = 3
        self.lattitude_ref = 45
        self.longitude_ref = 15
        self.lattitude_cov = 1
        self.longitude_cov = 1

        self.position_x_cov  = 1
        self.position_y_cov  = 1
        self.position_z_cov  = 0.5
        self.pos_filter_rate = 10
        self.gnss = NavSatFix()

        self.velocity_x_cov  = 0.2
        self.velocity_y_cov  = 0.2
        self.velocity_z_cov  = 0.1
        self.vel_filter_rate = 10
        self.linear_velocity = Vector3()

        self.filtered_euler_x_cov = 0.01
        self.filtered_euler_y_cov = 0.01
        self.filtered_euler_z_cov = 0.01
        self.free_accel_x_cov = 0.01
        self.free_accel_y_cov = 0.01
        self.free_accel_z_cov = 0.01
        self.filtered_gyro_x_cov  = 0.01
        self.filtered_gyro_y_cov  = 0.01
        self.filtered_gyro_z_cov  = 0.01
        self.imu_filter_rate = 100
        self.imu = Imu()

        self.accel_x_cov = 0.01
        self.accel_y_cov = 0.01
        self.accel_z_cov = 0.01
        self.accel_rate  = 100
        self.gravity = 9.8065
        self.linear_acceleration = Vector3()

        self.gyro_x_cov = 0.01
        self.gyro_y_cov = 0.01
        self.gyro_z_cov = 0.01
        self.gyro_rate = 100
        self.angular_velocity = Vector3()
        
        # Magnetometer Config
        self.mag_x_cov = 0.01
        self.mag_y_cov = 0.01
        self.mag_z_cov = 0.01
        self.mag_magnitude = 25.0
        self.mag_declination = 5.959 * np.pi / 180
        self.mag_inclination = 58.908 * np.pi / 180
        self.mag_ref_heading = 0
        self.magnetometer_raw_rate = 100
        self.mag_x_w, self.mag_y_w, self.mag_z_w, = self.world_magnetic_field(self.mag_magnitude, self.mag_declination, self.mag_inclination, self.mag_ref_heading)
        self.magnetic_field = Vector3()

        # Barometer Config
        self.pressure_cov = 0.01
        self.barometer_raw_rate = 25
        
        # Barometer Config
        self.temperature_cov = 0.01
        self.temperature_raw_rate = 25
        
        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'xsensSensorModel'       
        self.initialize_model()
        self.barometer_data = FluidPressure()
        

        #Initialization
        rospy.init_node(self._node_name)

        rospy.Subscriber("/gazebo/states/xsense/angular_velocity", Vector3, self.angular_velocity_handler_)
        rospy.Subscriber("/gazebo/states/xsense/linear_acceleration", Vector3, self.linear_acceleration_handler_)
        rospy.Subscriber("/gazebo/states/xsense/linear_velocity", Vector3, self.linear_velocity_handler_)
        rospy.Subscriber("/gazebo/states/xsense/position", Vector3, self.position_handler_)
        rospy.Subscriber("/gazebo/states/xsense/quaternion", Quaternion, self.quaternion_handler_)
                

        self._accelerometer_time = rospy.Timer(rospy.Duration(1/self.accel_rate), self._accelerometer_handler)
        self._gyroscope_timer = rospy.Timer(rospy.Duration(1/self.gyro_rate), self._gyroscope_handler)
        self._magnetometer_timer = rospy.Timer(rospy.Duration(1/self.magnetometer_raw_rate), self._magnetometer_handler)
        self._barometer_timer = rospy.Timer(rospy.Duration(1/self.barometer_raw_rate), self._barometer_handler)
        self._temperature_timer = rospy.Timer(rospy.Duration(1/self.temperature_raw_rate), self._temperature_handler)
        self._gnss_timer = rospy.Timer(rospy.Duration(1/self.gnss_rate), self._gnss_handler)
        
        self._filtered_vel_timer = rospy.Timer(rospy.Duration(1/self.vel_filter_rate), self._filtered_vel_handler)
        self._filtered_pos_timer = rospy.Timer(rospy.Duration(1/self.pos_filter_rate), self._filtered_pos_handler)
        self._filtered_imu_timer = rospy.Timer(rospy.Duration(1/self.imu_filter_rate), self._filtered_imu_handler)
        

        self._accelerometer_pub = rospy.Publisher('/imu/acceleration', Vector3, queue_size=5)
        self._gyroscope_pub = rospy.Publisher('/imu/angular_velocity', Vector3, queue_size=5)
        self._magnetometer_pub = rospy.Publisher('/imu/mag', Vector3, queue_size=5)
        self._barometer_pub = rospy.Publisher('/pressure', FluidPressure, queue_size=5)
        self._temperature = rospy.Publisher('/temperature', Temperature, queue_size=5)
        self._gnss_pub = rospy.Publisher('/gnss', NavSatFix, queue_size=5)
                
        self._filtered_vel_pub = rospy.Publisher('/filter/velocity', Vector3, queue_size=5)
        self._filtered_pos_pub = rospy.Publisher('/filter/positionlla', Vector3, queue_size=5)
        self._filtered_imu_pub = rospy.Publisher('/sensor/imu', Imu, queue_size=5)
        self._filtered_acc_pub = rospy.Publisher('/filter/free_acceleration', Vector3, queue_size=5)
        self._filtered_quat_pub = rospy.Publisher('/filter/quaternion', Quaternion, queue_size=5)


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
        self.rate = self.get_param('/x_sens_node_rate', self.rate)       
        
        # GPS Config
        self.lattitude_ref = self.get_param('/lattitude_ref', self.lattitude_ref)
        self.longitude_ref = self.get_param('/longitude_ref', self.longitude_ref)
        self.lattitude_cov = self.get_param('/lattitude_cov', self.lattitude_cov)
        self.longitude_cov = self.get_param('/longitude_cov', self.longitude_cov)       
        self.gnss_rate = self.get_param('/gnss_rate', self.gnss_rate)

        # Filtered Position Config
        self.position_x_cov = self.get_param('/position_x_cov', self.position_x_cov)
        self.position_y_cov = self.get_param('/position_y_cov', self.position_y_cov)
        self.position_z_cov = self.get_param('/position_z_cov', self.position_z_cov)
        self.pos_filter_rate = self.get_param('/position_rate', self.pos_filter_rate)
             
        # Filtered Velocity Config
        self.velocity_x_cov = self.get_param('/velocity_x_cov', self.velocity_x_cov)
        self.velocity_y_cov = self.get_param('/velocity_y_cov', self.velocity_y_cov)
        self.velocity_z_cov = self.get_param('/velocity_z_cov', self.velocity_z_cov)
        self.vel_filter_rate   = self.get_param('/velocity_rate', self.vel_filter_rate)

        # Filtered Attitude Data
        self.filtered_euler_x_cov = self.get_param('/filtered_euler_x_cov', self.filtered_euler_x_cov)
        self.filtered_euler_y_cov = self.get_param('/filtered_euler_y_cov', self.filtered_euler_y_cov)
        self.filtered_euler_z_cov = self.get_param('/filtered_euler_z_cov', self.filtered_euler_z_cov)
        self.free_accel_x_cov = self.get_param('/free_accel_z_cov', self.free_accel_x_cov)
        self.free_accel_y_cov = self.get_param('/free_accel_y_cov', self.free_accel_y_cov)
        self.free_accel_z_cov = self.get_param('/free_accel_z_cov', self.free_accel_z_cov)
        self.filtered_gyro_x_cov  = self.get_param('/filtered_gyro_x_cov', self.filtered_gyro_x_cov)
        self.filtered_gyro_y_cov  = self.get_param('/filtered_gyro_y_cov', self.filtered_gyro_y_cov)
        self.filtered_gyro_z_cov  = self.get_param('/filtered_gyro_z_cov', self.filtered_gyro_z_cov)
        self.imu_filter_rate = self.get_param('/imu_filter_rate', self.imu_filter_rate)

        # Accel Config
        self.accel_x_cov = self.get_param('/accel_x_cov', self.accel_x_cov)
        self.accel_y_cov = self.get_param('/accel_y_cov', self.accel_y_cov)
        self.accel_z_cov = self.get_param('/accel_z_cov', self.accel_z_cov)
        self.accel_rate = self.get_param('/accelerometer_raw_rate', self.accel_rate)
        self.gravity = self.get_param('/gravity', self.gravity)

        # Gyro Config
        self.gyro_x_cov = self.get_param('/gyro_x_cov', self.gyro_x_cov)
        self.gyro_y_cov = self.get_param('/gyro_y_cov', self.gyro_y_cov)
        self.gyro_z_cov = self.get_param('/gyro_z_cov', self.gyro_z_cov)
        self.gyro_rate = self.get_param('/gyroscope_raw_rate', self.gyro_rate)
        
        # Magnetometer Config
        self.mag_x_cov = self.get_param('/mag_x_cov', self.mag_x_cov)
        self.mag_y_cov = self.get_param('/mag_y_cov', self.mag_y_cov)
        self.mag_z_cov = self.get_param('/mag_z_cov', self.mag_z_cov)
        self.mag_magnitude = self.get_param('/magnetometer_magnitude', self.mag_magnitude)
        self.mag_declination = self.get_param('/magnetometer_declination', self.mag_declination) * np.pi / 180
        self.mag_inclination = self.get_param('/magnetometer_inclination', self.mag_inclination) * np.pi / 180
        self.mag_ref_heading = self.get_param('/magnetometer_reference_heading', self.mag_ref_heading)
        self.magnetometer_raw_rate = self.get_param('/magnetometer_raw_rate', self.magnetometer_raw_rate)
        
        # Barometer Config
        self.pressure_cov = self.get_param('/pressure_cov', self.pressure_cov)
        self.barometer_raw_rate = self.get_param('/barometer_raw_rate', self.barometer_raw_rate)

        # Temperature Config
        self.temperature_cov = self.get_param('/temperature_cov', self.pressure_cov)
        self.temperature_raw_rate = self.get_param('/temperature_raw_rate', self.temperature_raw_rate)


    def world_magnetic_field(self, magnitude, declination, inclination, reference_heading):
        x = magnitude *  np.cos(inclination) * np.cos(reference_heading - declination)
        y = magnitude *  np.cos(inclination) * np.sin(reference_heading - declination)
        z = magnitude * -np.sin(inclination);
        return x, y, z 


    def angular_velocity_handler_(self, msg):
        self.p_ned_w = msg.x
        self.q_ned_w = -msg.y
        self.r_ned_w = -msg.z


    def linear_acceleration_handler_(self, msg):
        self.du_ned_w = msg.x
        self.dv_ned_w = -msg.y
        self.dw_ned_w = -msg.z


    def linear_velocity_handler_(self, msg):
        self.u_ned_w = msg.x
        self.v_ned_w = -msg.y
        self.w_ned_w = -msg.z     


    def quaternion_handler_(self, msg):
        self.quat_x = msg.x
        self.quat_y = msg.y
        self.quat_z = msg.z  
        self.quat_w = msg.w

        x, y, z = quaternion_to_euler_angle(self.quat_x, self.quat_y, self.quat_z, self.quat_w)
        gx, gy, gz  = quaternion_to_gravity(self.quat_x, self.quat_y, self.quat_z, self.quat_w)
        self.phi_ned = x
        self.theta_ned = -y
        self.psi_ned = -z
        self.euler_angles = np.array([[self.phi_ned],[self.theta_ned],[self.psi_ned]])
        self.gx = gx
        self.gy = -gy
        self.gz = -gz
        self.gravity_vec  = np.array([[self.gx],[self.gy],[self.gz]])


    def position_handler_(self, msg):
        self.x_ned_w = msg.x
        self.y_ned_w = -msg.y
        self.z_ned_w = -msg.z  
        
        
    def _accelerometer_handler(self, event):
        linear_accel = earth2body_transformation(self.euler_angles, np.array([[self.du_ned_w],[self.dv_ned_w],[self.dw_ned_w]]))       
        self.linear_acceleration.x = linear_accel[0,0] + self.gravity_vec[0,0] * self.gravity
        self.linear_acceleration.y = linear_accel[1,0] + self.gravity_vec[1,0] * self.gravity
        self.linear_acceleration.z = linear_accel[2,0] + self.gravity_vec[2,0] * self.gravity
        self._accelerometer_pub.publish(self.linear_acceleration)
        
        
    def _gyroscope_handler(self, event):
        angular_vel = earth2body_transformation(self.euler_angles, np.array([[self.p_ned_w],[self.q_ned_w],[self.r_ned_w]]))
        self.angular_velocity.x = angular_vel[0,0]
        self.angular_velocity.y = angular_vel[1,0]
        self.angular_velocity.z = angular_vel[2,0]
        self._gyroscope_pub.publish(self.angular_velocity)
        
        
    def _magnetometer_handler(self, event):
        mag_field = earth2body_transformation(self.euler_angles, np.array([[self.mag_x_w],[self.mag_y_w],[self.mag_z_w]]))
        self.magnetic_field.x = mag_field[0,0]
        self.magnetic_field.y = mag_field[1,0]
        self.magnetic_field.z = mag_field[2,0]
        self._magnetometer_pub.publish(self.magnetic_field)
        
        
    def _gnss_handler(self, event):
        return
        
        
    def _filtered_vel_handler(self, event):
        linear_vel = earth2body_transformation(self.euler_angles, np.array([[self.u_ned_w],[self.v_ned_w],[self.w_ned_w]]))
        self.linear_velocity.x = linear_vel[0,0]
        self.linear_velocity.y = linear_vel[1,0]
        self.linear_velocity.z = linear_vel[2,0]
        self._filtered_vel_pub.publish(self.linear_velocity)
        

    def _filtered_pos_handler(self, event):
        return
        
        
    def _filtered_imu_handler(self, event):
        qx, qy, qz, qw = euler_to_quaternion(self.phi_ned, self.theta_ned, self.psi_ned)
        angular_vel = earth2body_transformation(self.euler_angles, np.array([[self.p_ned_w],[self.q_ned_w],[self.r_ned_w]]))
        linear_accel = earth2body_transformation(self.euler_angles, np.array([[self.du_ned_w],[self.dv_ned_w],[self.dw_ned_w]]))

        self.imu.orientation.x = qx
        self.imu.orientation.y = qy
        self.imu.orientation.z = qz
        self.imu.orientation.w = qw

        self.imu.angular_velocity.x = angular_vel[0,0]
        self.imu.angular_velocity.y = angular_vel[1,0]
        self.imu.angular_velocity.z = angular_vel[2,0]

        self.imu.linear_acceleration.x = linear_accel[0,0]
        self.imu.linear_acceleration.y = linear_accel[1,0]
        self.imu.linear_acceleration.z = linear_accel[2,0]

        self._filtered_imu_pub.publish(self.imu)

        
    def _barometer_handler(self, event):
        c1 = 44330.0
        c2 = 9.869232667160128300024673081668e-6
        c3 = 0.1901975534856
        height = -self.z_ned_w + np.random.normal(0, self.pressure_cov)
        interm = height / c1        
        pressure = math.pow(1-interm, -c3) / c2
        self.barometer_data.fluid_pressure = pressure
        self.barometer_data.variance = 0
        self._barometer_pub.publish(self.barometer_data)

       
    def _temperature_handler(self, event):
        return



if __name__ == '__main__':

    xsens = xsensModel()
    rospy.loginfo('XSENS SENSOR MODEL HAD BEEN ACTIVATED')
    
    rospy.spin()




