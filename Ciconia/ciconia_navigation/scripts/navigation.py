#! /usr/bin/python3

#! /usr/bin/python3

import numpy as np
from numpy import nan
import rospy, os, sys, time
import math

from geometry_msgs.msg import Pose, Vector3, Vector3Stamped, Quaternion
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
        self.latitude_ref = 45
        self.longitude_ref = 15
        self.altitude_ref = 0
        self.gnss_pos_x_var  = 1
        self.gnss_pos_y_var  = 1
        self.gnss_pos_z_var  = 0.5
    
        self.earth_equatorial_radius = 6378137.0
        self.earth_polar_radius  = 6356752.3


        self.filter_pos_x_var  = 1
        self.filter_pos_y_var  = 1
        self.filter_pos_z_var  = 0.5
        self.pos_filter_rate = 10
        self.gnss = NavSatFix()
        self.position = Vector3Stamped()

        self.velocity_x_var  = 0.2
        self.velocity_y_var  = 0.2
        self.velocity_z_var  = 0.1
        self.vel_filter_rate = 10
        self.linear_velocity = Vector3Stamped()

        self.filtered_euler_x_var = 0.01
        self.filtered_euler_y_var = 0.01
        self.filtered_euler_z_var = 0.01
        self.free_accel_x_var = 0.01
        self.free_accel_y_var = 0.01
        self.free_accel_z_var = 0.01
        self.acc_filter_rate = 100
        self.free_acceleration = Vector3Stamped()
        self.filtered_gyro_x_var  = 0.01
        self.filtered_gyro_y_var  = 0.01
        self.filtered_gyro_z_var  = 0.01
        self.imu_filter_rate = 100
        self.imu = Imu()

        self.accel_x_var = 0.01
        self.accel_y_var = 0.01
        self.accel_z_var = 0.01
        self.accel_rate  = 100
        self.gravity = 9.8065
        self.linear_acceleration = Vector3Stamped()

        self.gyro_x_var = 0.01
        self.gyro_y_var = 0.01
        self.gyro_z_var = 0.01
        self.gyro_rate = 100
        self.angular_velocity = Vector3Stamped()
        
        # Magnetometer Config
        self.mag_x_var = 0.01
        self.mag_y_var = 0.01
        self.mag_z_var = 0.01
        self.mag_magnitude = 25.0
        self.mag_declination = 5.959 * np.pi / 180
        self.mag_inclination = 58.908 * np.pi / 180
        self.mag_ref_heading = 0
        self.magnetometer_raw_rate = 100
        self.mag_x_w, self.mag_y_w, self.mag_z_w, = self.world_magnetic_field(self.mag_magnitude, self.mag_declination, self.mag_inclination, self.mag_ref_heading)
        self.magnetic_field = Vector3Stamped()

        # Barometer Config
        self.pressure_var = 0.01
        self.barometer_raw_rate = 25
        
        # Termometer Config
        self.temperature_var = 0.01
        self.temperature_raw_rate = 25
        self.temperature_data = Temperature()
        
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
        
        self._filtered_acc_timer = rospy.Timer(rospy.Duration(1/self.acc_filter_rate), self._free_acceleration_handler)
        self._filtered_vel_timer = rospy.Timer(rospy.Duration(1/self.vel_filter_rate), self._filtered_vel_handler)
        self._filtered_pos_timer = rospy.Timer(rospy.Duration(1/self.pos_filter_rate), self._filtered_pos_handler)
        self._filtered_imu_timer = rospy.Timer(rospy.Duration(1/self.imu_filter_rate), self._filtered_imu_handler)
        

        self._accelerometer_pub = rospy.Publisher('/xsens/imu/acceleration', Vector3Stamped, queue_size=5)
        self._gyroscope_pub = rospy.Publisher('/xsens/imu/angular_velocity', Vector3Stamped, queue_size=5)
        self._magnetometer_pub = rospy.Publisher('/xsens/imu/mag', Vector3Stamped, queue_size=5)
        self._barometer_pub = rospy.Publisher('/xsens/pressure', FluidPressure, queue_size=5)
        self._temperature = rospy.Publisher('/xsens/temperature', Temperature, queue_size=5)
        self._gnss_pub = rospy.Publisher('/xsens/gnss', NavSatFix, queue_size=5)
                
        self._filtered_vel_pub = rospy.Publisher('/xsens/filter/velocity', Vector3Stamped, queue_size=5)
        self._filtered_pos_pub = rospy.Publisher('/xsens/filter/positionlla', Vector3Stamped, queue_size=5)
        self._filtered_imu_pub = rospy.Publisher('/xsens/sensor/imu', Imu, queue_size=5)
        self._filtered_acc_pub = rospy.Publisher('/xsens/filter/free_acceleration', Vector3Stamped, queue_size=5)
        self._filtered_quat_pub = rospy.Publisher('/xsens/filter/quaternion', Quaternion, queue_size=5)


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
        self.latitude_ref = self.get_param('/latitude_ref', self.latitude_ref)
        self.longitude_ref = self.get_param('/longitude_ref', self.longitude_ref)
        self.altitude_ref = self.get_param('/altitude_ref', self.altitude_ref)
        self.gnss_pos_x_var = self.get_param('/gnss_pos_x_var', self.gnss_pos_x_var)
        self.gnss_pos_y_var = self.get_param('/gnss_pos_y_var', self.gnss_pos_y_var)
        self.gnss_pos_z_var = self.get_param('/gnss_pos_z_var', self.gnss_pos_z_var)  
        self.gnss_rate = self.get_param('/gnss_rate', self.gnss_rate)

        # Filtered Position Config
        self.filter_pos_x_var = self.get_param('/filter_pos_x_var', self.filter_pos_x_var)
        self.filter_pos_y_var = self.get_param('/filter_pos_y_var', self.filter_pos_y_var)
        self.filter_pos_z_var = self.get_param('/filter_pos_z_var', self.filter_pos_z_var)
        self.pos_filter_rate = self.get_param('/position_rate', self.pos_filter_rate)
             
        # Filtered Velocity Config
        self.velocity_x_var = self.get_param('/velocity_x_var', self.velocity_x_var)
        self.velocity_y_var = self.get_param('/velocity_y_var', self.velocity_y_var)
        self.velocity_z_var = self.get_param('/velocity_z_var', self.velocity_z_var)
        self.vel_filter_rate   = self.get_param('/velocity_rate', self.vel_filter_rate)

        # Filtered Attitude Data
        self.filtered_euler_x_var = self.get_param('/filtered_euler_x_var', self.filtered_euler_x_var)
        self.filtered_euler_y_var = self.get_param('/filtered_euler_y_var', self.filtered_euler_y_var)
        self.filtered_euler_z_var = self.get_param('/filtered_euler_z_var', self.filtered_euler_z_var)
        self.free_accel_x_var = self.get_param('/free_accel_z_var', self.free_accel_x_var)
        self.free_accel_y_var = self.get_param('/free_accel_y_var', self.free_accel_y_var)
        self.free_accel_z_var = self.get_param('/free_accel_z_var', self.free_accel_z_var)
        self.filtered_gyro_x_var  = self.get_param('/filtered_gyro_x_var', self.filtered_gyro_x_var)
        self.filtered_gyro_y_var  = self.get_param('/filtered_gyro_y_var', self.filtered_gyro_y_var)
        self.filtered_gyro_z_var  = self.get_param('/filtered_gyro_z_var', self.filtered_gyro_z_var)
        self.imu_filter_rate = self.get_param('/imu_filter_rate', self.imu_filter_rate)

        # Accel Config
        self.accel_x_var = self.get_param('/accel_x_var', self.accel_x_var)
        self.accel_y_var = self.get_param('/accel_y_var', self.accel_y_var)
        self.accel_z_var = self.get_param('/accel_z_var', self.accel_z_var)
        self.accel_rate = self.get_param('/accelerometer_raw_rate', self.accel_rate)
        self.gravity = self.get_param('/gravity', self.gravity)

        # Gyro Config
        self.gyro_x_var = self.get_param('/gyro_x_var', self.gyro_x_var)
        self.gyro_y_var = self.get_param('/gyro_y_var', self.gyro_y_var)
        self.gyro_z_var = self.get_param('/gyro_z_var', self.gyro_z_var)
        self.gyro_rate = self.get_param('/gyroscope_raw_rate', self.gyro_rate)
        
        # Magnetometer Config
        self.mag_x_var = self.get_param('/mag_x_var', self.mag_x_var)
        self.mag_y_var = self.get_param('/mag_y_var', self.mag_y_var)
        self.mag_z_var = self.get_param('/mag_z_var', self.mag_z_var)
        self.mag_magnitude = self.get_param('/magnetometer_magnitude', self.mag_magnitude)
        self.mag_declination = self.get_param('/magnetometer_declination', self.mag_declination) * np.pi / 180
        self.mag_inclination = self.get_param('/magnetometer_inclination', self.mag_inclination) * np.pi / 180
        self.mag_ref_heading = self.get_param('/magnetometer_reference_heading', self.mag_ref_heading)
        self.magnetometer_raw_rate = self.get_param('/magnetometer_raw_rate', self.magnetometer_raw_rate)
        
        # Barometer Config
        self.pressure_var = self.get_param('/pressure_var', self.pressure_var)
        self.barometer_raw_rate = self.get_param('/barometer_raw_rate', self.barometer_raw_rate)

        # Temperature Config
        self.temperature_var = self.get_param('/temperature_var', self.pressure_var)
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

        x, y, z = quaternion_to_euler_angle(self.quat_w, self.quat_x, self.quat_y, self.quat_z)
        gx, gy, gz  = quaternion_to_gravity(self.quat_w, self.quat_x, self.quat_y, self.quat_z)
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
        self.linear_acceleration.vector.x = linear_accel[0,0] + self.gravity_vec[0,0] * self.gravity + np.random.normal(0,self.accel_x_var)
        self.linear_acceleration.vector.y = linear_accel[1,0] + self.gravity_vec[1,0] * self.gravity + np.random.normal(0,self.accel_y_var)
        self.linear_acceleration.vector.z = linear_accel[2,0] + self.gravity_vec[2,0] * self.gravity + np.random.normal(0,self.accel_z_var)
        self._accelerometer_pub.publish(self.linear_acceleration)
        
        
    def _gyroscope_handler(self, event):
        angular_vel = earth2body_transformation(self.euler_angles, np.array([[self.p_ned_w],[self.q_ned_w],[self.r_ned_w]]))
        self.angular_velocity.vector.x = angular_vel[0,0] + np.random.normal(0,self.gyro_x_var)
        self.angular_velocity.vector.y = angular_vel[1,0] + np.random.normal(0,self.gyro_y_var)
        self.angular_velocity.vector.z = angular_vel[2,0] + np.random.normal(0,self.gyro_z_var)
        self._gyroscope_pub.publish(self.angular_velocity)
        
        
    def _magnetometer_handler(self, event):
        mag_field = earth2body_transformation(self.euler_angles, np.array([[self.mag_x_w],[self.mag_y_w],[self.mag_z_w]]))
        self.magnetic_field.vector.x = mag_field[0,0] + np.random.normal(0,self.mag_x_var)
        self.magnetic_field.vector.y = mag_field[1,0] + np.random.normal(0,self.mag_y_var)
        self.magnetic_field.vector.z = mag_field[2,0] + np.random.normal(0,self.mag_z_var)
        self._magnetometer_pub.publish(self.magnetic_field)
        
        
    def _gnss_handler(self, event):
        X_W = self.x_ned_w + np.random.normal(0,self.gnss_pos_x_var)
        Y_W = self.y_ned_w + np.random.normal(0,self.gnss_pos_y_var)
        Z_W = self.z_ned_w + np.random.normal(0,self.gnss_pos_z_var)
        
        latitude  = self.latitude_ref + np.arcsin(X_W / self.earth_equatorial_radius) * 180 / np.pi
        longitude = self.longitude_ref + np.arcsin(Y_W / self.earth_polar_radius / np.cos(self.longitude_ref * np.pi / 180)) * 180 / np.pi
        altitude  = -Z_W + self.altitude_ref
        self.gnss.latitude = latitude
        self.gnss.longitude = longitude
        self.gnss.altitude = altitude
        self._gnss_pub.publish(self.gnss)
        
        
    def _filtered_vel_handler(self, event):
        linear_vel = earth2body_transformation(self.euler_angles, np.array([[self.u_ned_w],[self.v_ned_w],[self.w_ned_w]]))
        self.linear_velocity.vector.x = linear_vel[0,0] + np.random.normal(0,self.velocity_x_var)
        self.linear_velocity.vector.y = linear_vel[1,0] + np.random.normal(0,self.velocity_y_var)
        self.linear_velocity.vector.z = linear_vel[2,0] + np.random.normal(0,self.velocity_z_var)
        self._filtered_vel_pub.publish(self.linear_velocity)
        

    def _filtered_pos_handler(self, event):
        X_W = self.x_ned_w + np.random.normal(0,self.filter_pos_x_var)
        Y_W = self.y_ned_w + np.random.normal(0,self.filter_pos_y_var)
        Z_W = self.z_ned_w + np.random.normal(0,self.filter_pos_z_var)

        latitude  = self.latitude_ref + np.arcsin(X_W / self.earth_equatorial_radius) * 180 / np.pi
        longitude = self.longitude_ref + np.arcsin(Y_W / self.earth_polar_radius / np.cos(self.longitude_ref * np.pi / 180)) * 180 / np.pi
        altitude  = -Z_W + self.altitude_ref

        self.position.vector.x = latitude
        self.position.vector.y = longitude
        self.position.vector.z = altitude

        self._filtered_pos_pub.publish(self.position)


    def _free_acceleration_handler(self, event):
        linear_accel = earth2body_transformation(self.euler_angles, np.array([[self.du_ned_w],[self.dv_ned_w],[self.dw_ned_w]]))       
        self.free_acceleration.vector.x = linear_accel[0,0] + np.random.normal(0,self.velocity_x_var)
        self.free_acceleration.vector.y = linear_accel[1,0] + np.random.normal(0,self.velocity_x_var)
        self.free_acceleration.vector.z = linear_accel[2,0] + np.random.normal(0,self.velocity_x_var)
        self._filtered_acc_pub.publish(self.free_acceleration)
        
        
    def _filtered_imu_handler(self, event):
        #phi = self.phi_ned + np.random.normal(0,self.filtered_euler_x_var)
        #theta = self.theta_ned + np.random.normal(0,self.filtered_euler_y_var)
        #psi = self.psi_ned + np.random.normal(0,self.filtered_euler_z_var)

        #qx, qy, qz, qw = euler_to_quaternion(phi, theta, -psi)
        angular_vel = earth2body_transformation(self.euler_angles, np.array([[self.p_ned_w],[self.q_ned_w],[self.r_ned_w]]))
        linear_accel = earth2body_transformation(self.euler_angles, np.array([[self.du_ned_w],[self.dv_ned_w],[self.dw_ned_w]]))

        self.imu.orientation.x = self.quat_x + np.random.normal(0,self.filtered_euler_x_var)/2
        self.imu.orientation.y = -self.quat_y + np.random.normal(0,self.filtered_euler_x_var)/2
        self.imu.orientation.z = -self.quat_z + np.random.normal(0,self.filtered_euler_x_var)/2
        self.imu.orientation.w = self.quat_w + np.random.normal(0,self.filtered_euler_x_var)/2

        self.imu.angular_velocity.x = angular_vel[0,0] + np.random.normal(0,self.filtered_gyro_x_var)
        self.imu.angular_velocity.y = angular_vel[1,0] + np.random.normal(0,self.filtered_gyro_x_var)
        self.imu.angular_velocity.z = angular_vel[2,0] + np.random.normal(0,self.filtered_gyro_x_var)

        self.imu.linear_acceleration.x = linear_accel[0,0] + np.random.normal(0,self.free_accel_x_var)
        self.imu.linear_acceleration.y = linear_accel[1,0] + np.random.normal(0,self.free_accel_x_var)
        self.imu.linear_acceleration.z = linear_accel[2,0] + np.random.normal(0,self.free_accel_x_var)

        self._filtered_imu_pub.publish(self.imu)

        
    def _barometer_handler(self, event):
        c1 = 44330.0
        c2 = 9.869232667160128300024673081668e-6
        c3 = 0.1901975534856
        height = -self.z_ned_w + np.random.normal(0, self.pressure_var)
        interm = height / c1        
        pressure = math.pow(1-interm, -c3) / c2 + np.random.normal(0,self.pressure_var)
        self.barometer_data.fluid_pressure = pressure
        self.barometer_data.variance = 0
        self._barometer_pub.publish(self.barometer_data)


    def _temperature_handler(self, event):
        temperature = 25.0
        self.temperature_data.temperature = temperature + np.random.normal(0,self.temperature_var)
        self._temperature.publish(self.temperature_data)



if __name__ == '__main__':

    xsens = xsensModel()

    rospy.loginfo('XSENS SENSOR MODEL HAS BEEN ACTIVATED')
    
    rospy.spin()




