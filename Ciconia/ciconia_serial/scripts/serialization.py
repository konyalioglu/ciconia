#! /usr/bin/python3


"""
Created on Mon Aug  2 16:42:24 2021

@author: turan
"""


import numpy as np

import rospy

from threading import Thread, Lock
import serial
from ciconia_msgs.msg import rcMessages, motorSignal



class serialization:

    def __init__(self):

        #Default Parameters
        self.control_signal_topic = '/ciconia/control_signals'
        self.rc_signal_topic = '/ciconia/sbus_signal'
        self.device = '/dev/ttyACM0'
        self.baudrate = 57600
        self.timeout = 1

        self.isReady = False
        self.reading = False

        #MOTOR SIGNALS
        self.header1  = 0xff
        self.header2  = 0xff

        self.end_byte  = 0xfc
        self.end_byte2 = 0xfc


        self.tbuf = np.zeros((21), dtype='uint8')

        self.tbuf[0] = self.header1
        self.tbuf[1] = self.header2

        self.tbuf[-2] = self.end_byte
        self.tbuf[-1] = self.end_byte2


        #SBUS SIGNALS

        self.sbus_header = 0x0f

        self.sbus_footer1 = 0x00
        self.sbus_footer2 = 0x04

        self.rbuf = np.zeros((1,25), dtype='uint8')
        self.rxbytearray = bytearray(25)

        self.sbus_state = 0
        self.sbus_index = 0
        self.sbus_data_length = 0

        ##Ros 
        self._namespace = rospy.get_namespace()
        self._node_name = 'ciconia_serial'       
        self.initialize_model()
        self.sbus_data = rcMessages()
        self.control_data = motorSignal()

        #Initialization
        rospy.init_node(self._node_name)
        rospy.Subscriber(self.control_signal_topic, motorSignal, self._control_signal_publisher)
        self._sbus_data_publisher = rospy.Publisher(self.rc_signal_topic, rcMessages, queue_size=5)

        self.stream = serial.Serial(self.device, self.baudrate, timeout=None)


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
        self.device = self.get_param('/device', self.device)        
        self.control_signal_topic = self.get_param('/control_signal_topic', self.control_signal_topic)
        self.rc_signal_topic = self.get_param('/rc_signal_topic', self.rc_signal_topic)


    def _control_signal_publisher(self, msg):
        pwm1 = msg.MOTOR1_PWM
        pwm2 = msg.MOTOR2_PWM
        pwm3 = msg.MOTOR3_PWM
        pwm4 = msg.MOTOR4_PWM
        pwm5 = msg.MOTOR5_PWM

        dail = msg.AILERON_ANGLE
        dele = msg.ELEVATOR_ANGLE
        drud = msg.RUDDER_ANGLE

        self.tbuf[2] = np.uint8(pwm1 & 0xff)
        self.tbuf[3] = np.uint8(pwm1 >> 8)

        self.tbuf[4] = np.uint8(pwm2 & 0xff)
        self.tbuf[5] = np.uint8(pwm2 >> 8)

        self.tbuf[6] = np.uint8(pwm3 & 0xff)
        self.tbuf[7] = np.uint8(pwm3 >> 8)

        self.tbuf[8] = np.uint8(pwm4 & 0xff)
        self.tbuf[9] = np.uint8(pwm4 >> 8)

        self.tbuf[10] = np.uint8(pwm5 & 0xff)
        self.tbuf[11] = np.uint8(pwm5 >> 8)

        self.tbuf[12] = np.uint8(dail & 0xff)
        self.tbuf[13] = np.uint8(dail >> 8)

        self.tbuf[14] = np.uint8(dele & 0xff)
        self.tbuf[15] = np.uint8(dele >> 8)

        self.tbuf[16] = np.uint8(drud & 0xff)
        self.tbuf[17] = np.uint8(drud >> 8)

        chksm = np.sum(self.tbuf[2:18])
        self.tbuf[18] = chksm

        #for byte in self.tbuf:
        self.stream.write(self.tbuf)
        print(self.tbuf)


    def read_sbus_protocol(self, stream):
        buffer = stream.read(1)

        if buffer[0] == self.sbus_header:
            self.rxbytearray[0] = buffer[0]
            buffer = stream.read(24)
            self.rxbytearray[1:25] = buffer

            if self.rxbytearray[-1] == self.sbus_footer1 or (self.rxbytearray[-1] & self.sbus_header) == self.sbus_footer2:

                self.sbus_data.channel0  = (np.uint16(self.rxbytearray[1]) | (np.uint16(self.rxbytearray[2]) << 8)) & 0x07FF

                self.sbus_data.channel1  = ((np.uint16(self.rxbytearray[2]) >> 3)	| (np.uint16(self.rxbytearray[3]) << 5)) & 0x07FF

                self.sbus_data.channel2  = ((np.uint16(self.rxbytearray[3]) >> 6) | (np.uint16(self.rxbytearray[4]) << 2)	| (np.uint16(self.rxbytearray[5]) << 10)) & 0x07FF

                self.sbus_data.channel3  = ((np.uint16(self.rxbytearray[5]) >> 1)	| (np.uint16(self.rxbytearray[6]) << 7)) & 0x07FF

                self.sbus_data.channel4  = ((np.uint16(self.rxbytearray[6]) >> 4) | (np.uint16(self.rxbytearray[7]) << 4)) & 0x07FF

                self.sbus_data.channel5  = ((np.uint16(buffer[7]) >> 7)	| (np.uint16(buffer[8]) << 1) | (np.uint16(buffer[9]) << 9)) & 0x07FF

                self.sbus_data.channel7  = ((np.uint16(self.rxbytearray[10]) >> 5)	| (np.uint16(self.rxbytearray[11]) << 3)) & 0x07FF

                self.sbus_data.channel8  = (np.uint16(self.rxbytearray[12])  | (np.uint16(self.rxbytearray[13]) << 8)) & 0x07FF

                self.sbus_data.channel9  = ((np.uint16(self.rxbytearray[13]) >> 3)	| (np.uint16(self.rxbytearray[14]) << 5)) & 0x07FF

                self.sbus_data.channel10 = ((np.uint16(self.rxbytearray[14]) >> 6)	| (np.uint16(self.rxbytearray[15]) << 2)	| ((np.uint16(self.rxbytearray[16]) << 10))) & 0x07FF

                self.sbus_data.channel11 = ((np.uint16(self.rxbytearray[16]) >> 1) | (np.uint16(self.rxbytearray[17]) << 7)) & 0x07FF

                self.sbus_data.channel12 = ((np.uint16(self.rxbytearray[17]) >> 4)	| (np.uint16(self.rxbytearray[18]) << 4)) & 0x07FF

                self.sbus_data.channel13 = ((np.uint16(self.rxbytearray[18]) >> 7)	| (np.uint16(self.rxbytearray[19]) << 1)	| (np.uint16(self.rxbytearray[20]) << 9)) & 0x07FF
 
                self.sbus_data.channel14 = ((np.uint16(self.rxbytearray[20]) >> 2)	| (np.uint16(self.rxbytearray[21]) << 6)) & 0x07FF
 
                self.sbus_data.channel15 = ((np.uint16(self.rxbytearray[21]) >> 5)	| (np.uint16(self.rxbytearray[22]) << 3)) & 0x07FF

                self.sbus_data.channel17 = np.uint16(self.rxbytearray[23] & 0x01)

                self.sbus_data.channel18 = np.uint16(self.rxbytearray[23] & 0x02)

                self.sbus_data.lostframe = np.uint8(self.rxbytearray[23] & 0x04)

                self.sbus_data.failsafe =  np.uint16(self.rxbytearray[23] & 0x08)

                self._sbus_data_publisher.publish(self.sbus_data)
 


    def read_messages(self, stream, lock):

        while self.reading:
            if stream.in_waiting > 2024:
                stream.flushInput()
            if stream.in_waiting:
                try:
                    self.read_sbus_protocol(stream)

                except Exception as err:
                    print(f"\n\nSomething went wrong {err}\n\n")
                    continue


    def main_thread(self, stream, lock):
        """
        Start read thread
        """
        thr = Thread(target=self.read_messages, args=(stream, lock), daemon=True)
        thr.start()
        return thr

        

def main():
    stm32_ser = serialization()

    rospy.loginfo('CICONIA SERIALIZATION HAS BEEN ACTIVATED!')

    serial_lock = Lock()
    stm32_ser.reading = True
    read_thread = stm32_ser.main_thread(stm32_ser.stream, serial_lock)
    print("\nStarting threads...\n")

    stm32_ser.stream.flushInput()

    while not rospy.is_shutdown():
        if stm32_ser.isReady:
                print("Check")

    stm32_ser.reading = False
    read_thread.join()


    
    rospy.spin()



if __name__ == '__main__':
    main()




