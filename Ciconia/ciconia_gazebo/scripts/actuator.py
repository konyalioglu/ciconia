#! /usr/bin/python3

import rospy, os
from std_msgs.msg import Float64MultiArray
from ciconia_msgs.msg import motorSignal
import numpy as np
from scipy.io import loadmat
from scipy.interpolate import interp1d


class Actuator():

    def __init__(self):

        self.pwm_min = 1000
        self.pwm_max = 2000

        self.deflection_min = -800 * 2 * np.pi / 4095 / 2
        self.deflection_max =  800 * 2 * np.pi / 4095 / 2

        dir_path = os.path.dirname(os.path.realpath(__file__))
        prop_data = loadmat(dir_path + '/database/MN801S_26x85_PropulsionData_py.mat') 
        RPM = np.array(prop_data['MN801S_26x85_PropulsionData']['RPM'][0][0]).flatten()
        throttle_BP = np.array(prop_data['MN801S_26x85_PropulsionData']['throttle'][0][0]).flatten()
        self.rpm_interpn1 = interp1d(throttle_BP, RPM, kind='linear', bounds_error=False, fill_value=None)

        prop_data = loadmat(dir_path + '/database/AT7217_21x10_PropulsionData_py.mat')  
        RPM = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM'][0][0]).flatten()
        self.rpm_interpn2 = interp1d(throttle_BP, RPM, kind='linear', bounds_error=False, fill_value=None)        

        self.motor_vel = Float64MultiArray()
        self.servo_ang = Float64MultiArray()

        rospy.init_node('ciconia_actuator')
        rospy.Subscriber("/ciconia/actuator", motorSignal, self._actuator_signal_handler)

        self.pub_motor_vel = rospy.Publisher('/ciconia/vel_cmd', Float64MultiArray, queue_size=4)
        self.pub_servo_ang = rospy.Publisher('/ciconia/joint_controlDeflection_controller/command', Float64MultiArray, queue_size=4)



    def _actuator_signal_handler(self, msg):
        
        if msg.MOTOR1_PWM > self.pwm_max:
            PWM1 = self.pwm_max
        elif msg.MOTOR1_PWM < self.pwm_min:
            PWM1 = self.pwm_min
        else:
            PWM1 = msg.MOTOR1_PWM

        if msg.MOTOR2_PWM > self.pwm_max:
            PWM2 = self.pwm_max
        elif msg.MOTOR2_PWM < self.pwm_min:
            PWM2 = self.pwm_min
        else:
            PWM2 = msg.MOTOR2_PWM

        if msg.MOTOR3_PWM > self.pwm_max:
            PWM3 = self.pwm_max
        elif msg.MOTOR3_PWM < self.pwm_min:
            PWM3 = self.pwm_min
        else:
            PWM3 = msg.MOTOR3_PWM

        if msg.MOTOR4_PWM > self.pwm_max:
            PWM4 = self.pwm_max
        elif msg.MOTOR4_PWM < self.pwm_min:
            PWM4 = self.pwm_min
        else:
            PWM4 = msg.MOTOR4_PWM

        if msg.MOTOR5_PWM > self.pwm_max:
            PWM5 = self.pwm_max
        elif msg.MOTOR5_PWM < self.pwm_min:
            PWM5 = self.pwm_min
        else:
            PWM5 = msg.MOTOR5_PWM

        MOTOR1_VEL = self.rpm_interpn1((PWM1 - 1000) / 1000)
        MOTOR2_VEL = self.rpm_interpn1((PWM2 - 1000) / 1000)
        MOTOR3_VEL = self.rpm_interpn1((PWM3 - 1000) / 1000)
        MOTOR4_VEL = self.rpm_interpn1((PWM4 - 1000) / 1000)
        MOTOR5_VEL = self.rpm_interpn2((PWM5 - 1000) / 1000)


        if msg.AILERON_ANGLE > self.deflection_max:
            DAIL = self.deflection_max
        elif msg.AILERON_ANGLE < self.deflection_min:
            DAIL = self.deflection_min
        else:
            DAIL = msg.AILERON_ANGLE

        if msg.ELEVATOR_ANGLE > self.deflection_max:
            DELE = self.deflection_max
        elif msg.ELEVATOR_ANGLE < self.deflection_min:
            DELE = self.deflection_min
        else:
            DELE = msg.ELEVATOR_ANGLE

        if msg.RUDDER_ANGLE > self.deflection_max:
            DRUD = self.deflection_max
        elif msg.RUDDER_ANGLE < self.deflection_min:
            DRUD = self.deflection_min
        else:
            DRUD = msg.RUDDER_ANGLE

        self.motor_vel.data = [MOTOR1_VEL, MOTOR2_VEL, MOTOR3_VEL, MOTOR4_VEL, MOTOR5_VEL]
        self.servo_ang.data = [DAIL/2, -DAIL/2, DRUD, DRUD, DELE]
        self.pub_motor_vel.publish(self.motor_vel)
        self.pub_servo_ang.publish(self.servo_ang)


def main():
    actuator = Actuator()

    rospy.loginfo('PITOT TUBE SENSOR MODEL HAS BEEN ACTIVATED!')
    
    rospy.spin()



if __name__ == '__main__':
    main()
