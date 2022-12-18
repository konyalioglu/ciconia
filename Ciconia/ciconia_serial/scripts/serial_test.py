#! /usr/bin/python3


"""
Created on Mon Aug  2 16:42:24 2021

@author: turan
"""

import numpy as np
from numpy import nan
import rospy

import random 

from ciconia_msgs.msg import rcMessages, motorSignal



class test_serial:

    def __init__(self):

        #Default Parameters

        self.rate = 50


        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'ciconia_serial_test'       
        self.control_data = motorSignal()

        #Initialization
        rospy.init_node(self._node_name)
        self._data_publisher = rospy.Publisher('/ciconia/control_signals', motorSignal, queue_size=5)
        self._serial_test_timer = rospy.Timer(rospy.Duration(1/self.rate), self._motor_data_publisher)



    def _motor_data_publisher(self, event):
        self.control_data.MOTOR1_PWM = 1000 + random.randint(0, 50) 
        self.control_data.MOTOR2_PWM = 1100 + random.randint(0, 50) 
        self.control_data.MOTOR3_PWM = 1200 + random.randint(0, 50) 
        self.control_data.MOTOR4_PWM = 1300 + random.randint(0, 50) 
        self.control_data.MOTOR5_PWM = 1400 + random.randint(0, 50) 

        self.control_data.AILERON_ANGLE = 1500 + random.randint(0, 50) 
        self.control_data.ELEVATOR_ANGLE = 1600 + random.randint(0, 50) 
        self.control_data.RUDDER_ANGLE = 1700 + random.randint(0, 50) 

        self._data_publisher.publish(self.control_data)

        

def main():
    stm32_ser = test_serial()

    rospy.loginfo('CICONIA SERIALIZATION HAS BEEN ACTIVATED!')
    
    rospy.spin()



if __name__ == '__main__':
    main()




