
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float32
from sensor_msgs.msg import Imu
import numpy as np
from .utils import *
from .pid import pid


class ControlNode(Node):

    def __init__(self):
        super().__init__('control_node')
        self.state_estimator_subs_ = self.create_subscription(Float64MultiArray, '/akillipaket/Kalman/States', self.estimator_callback_, 4)
        self.motor_input_publisher_ = self.create_publisher(Float64MultiArray, '/akillipaket/motor/input', 4)
        self.gps_get_ref_sub_ = self.create_subscription(Float64MultiArray, '/akillipaket/GPS/ref', self.position_set_point_callback_, 4)
        self.motor_input_ = Float64MultiArray()
        self.velocity_controller = pid(1.0, 0.2, 0.05)
        self.position_controller = pid(0.5, 0.0, 0.0)
        self.yaw_rate_controller = pid(8.0, 1, 0.18, output_constraints = [-30.0, 30.0])
        self.yaw_rate_controller = pid(10.0, 0.2, 0.18, output_constraints = [-30.0, 30.0])
        self.heading_controller  = pid(2.5, 0.0, 0.0, output_constraints = [-0.5, 0.5])

        self.x_ref = 0
        self.y_ref = 0

        self.arm = False
        self.arm_acc = 5
        self.arm_alt = 2

        self.dt = 0.02
        self.pwm_idle = 20

        self.arm_controller = False


    def estimator_callback_(self, msg):
        pwm1, pwm2 = self.pid_handler_(msg.data)
        self.motor_input_.data = [pwm1, pwm2]
        self.motor_input_publisher_.publish(self.motor_input_)
        if self.arm_controller == False and msg.data[0] != 0.0:
            self.arm_controller = True

    def pid_handler_(self, data):
        '''
           x   = data[0]
           y   = data[1]
           z   = data[2]
           xd  = data[3]
           yd  = data[4]
           zd  = data[5]
           xdd = data[6]
           ydd = data[7]
           zdd = data[8]
           psi = data[9]
           psid= data[10]
           r   = data[11]
        '''

        if self.arm_controller == False:
            pwm1 = 0.0
            pwm2 = 0.0
            return pwm1, pwm2

        heading_ref    = -np.arctan2((self.y_ref-data[1]),(self.x_ref-data[0]))
        x_body, y_body = rotzE2B(heading_ref) @ np.array([[data[0]],[data[1]]])
        x_body_ref     = np.sqrt(data[0]**2 + data[1]**2)
        u, v = rotzE2B(heading_ref) @ np.array([[data[3]],[data[4]]])

        data[9] = data[9] * np.pi / 180

        k = int(data[9] / 2 / np.pi)

        heading_ref = 2.70  #-165 * np.pi / 180

        heading_ref = (heading_ref + 2 * np.pi * k)

        if heading_ref - data[9] > np.pi:
            heading_ref -= 2 * np.pi
        elif heading_ref - data[9] < -np.pi:
            heading_ref += 2 * np.pi


        #u_signal = position_controller.calculate_control_input(x_body_ref, x_body, self.dt)
        #control_signal1 = velocity_controller.calculate_control_input(u_signal, u, self.dt)
        control_signal1 = 0

        psidot_signal = self.heading_controller.calculate_control_input(heading_ref, data[9], self.dt)
        control_signal2 = self.yaw_rate_controller.calculate_control_input(psidot_signal, data[14], self.dt)

        pwm1 = control_signal1 / 2 - control_signal2 / 2 + self.pwm_idle
        pwm2 = control_signal1 / 2 + control_signal2 / 2 + self.pwm_idle

        self.get_logger().info(str(heading_ref) + ' ' + str(data[9]) + ' ' + str(psidot_signal)+ ' ' + str(data[14]) + ' ' + str(pwm1) + ' '+str(pwm2))

        return pwm1, pwm2


    def position_set_point_callback_(self, msg):
        self.x_ref = msg.data[0]
        self.y_ref = msg.data[1]


def main():
    rclpy.init()
    cn = ControlNode()
    rclpy.spin(cn)

