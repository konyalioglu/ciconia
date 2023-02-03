#! /usr/bin/python3

import rospy

import numpy as np
from std_msgs.msg import Float64MultiArray
from ciconia_msgs.msg import altPIDControl
import time


class LogNode():

    def __init__(self):

        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'altLogNode'       

        #Initialization
        rospy.init_node(self._node_name)

        rospy.Subscriber('/pid/data', altPIDControl, self._pid_data_handler)  
        rospy.Subscriber('/alt_est/states', Float64MultiArray, self._altitude_estimator_data_handler)     

        self.time_ref = time.time()
        self.dir = '/home/turan/ciconia/src/ciconia_logging/log'
        self.alt_est_dir = self.dir + '/alt_est_data.csv'
        self.pid_data_dir = self.dir + '/pid_data.csv'

        np.savetxt(self.alt_est_dir, np.array([[0,0,0]]), delimiter=",")
        np.savetxt(self.pid_data_dir, np.array([[0,0,0,0,0,0,0]]), delimiter=",")


    def _altitude_estimator_data_handler(self, msg):
        timer = time.time() - self.time_ref
        data = np.array([[timer, msg.data[0], msg.data[1]]])
        with open(self.alt_est_dir, 'a') as f:
            np.savetxt(f, data, delimiter=",")


    def _pid_data_handler(self, msg):
        timer = time.time() - self.time_ref

        ref_alt = msg.reference_altitude
        alt = msg.altitude
        home_alt = msg.home_altitude
        velocity_signal = msg.velocity_signal
        acceleration_signal = msg.acceleration_signal
        throttle = msg.throttle

        state = np.array([[timer, ref_alt, alt, home_alt, velocity_signal, acceleration_signal, throttle]])
        with open(self.pid_data_dir, 'a') as f:
            np.savetxt(f, state, delimiter=",")



if __name__ == '__main__':

    logging = LogNode()

    rospy.loginfo('CICONIA LOGGER HAS BEEN ACTIVATED')
    
    rospy.spin()


python3-pykdl
control_msgs
angles
geographic_msgs