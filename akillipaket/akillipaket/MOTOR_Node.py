import serial
import struct
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class PWMNode(Node):

    def __init__(self):
        super().__init__('pwm_node')
        self.subscription = self.create_subscription(Float64MultiArray,'/akillipaket/motor/input',self.pwm_callback,4)

        self.pwm_max = 40
        self.pwm_min = 0
        self.conn = serial.Serial(port='/dev/ttyUSB0', baudrate=19200, timeout=0.1)

        self.start_byte = '$'.encode()
        self.end_byte   = '&'.encode()
        self.end_byte2    = '%'.encode()

        pwm1 = struct.pack("i", 0)
        pwm2 = struct.pack("i", 0)

        init_data = self.start_byte + pwm1 + pwm2 + self.end_byte + self.end_byte2

        self.send_pwm_data_(init_data)
        time.sleep(10)


    def send_pwm_data_(self, data):
        self.conn.write(data)


    def data_handler_(self, pwm1, pwm2):
        pwm1_bytes = struct.pack("i", int(round(pwm1)))
        pwm2_bytes = struct.pack("i", int(round(pwm2)))
        data = self.start_byte + pwm1_bytes + pwm2_bytes + self.end_byte + self.end_byte2
        self.send_pwm_data_(data)


    def pwm_callback(self, msg):
        if msg.data[0] > self.pwm_max:
            pwm1 = self.pwm_max
        elif msg.data[0] < self.pwm_min:
            pwm1 = self.pwm_min
        else:
            pwm1 = msg.data[0]

        if msg.data[1] > self.pwm_max:
            pwm2 = self.pwm_max
        elif msg.data[1] < self.pwm_min:
            pwm2 = self.pwm_min
        else:
            pwm2 = msg.data[1]

        self.get_logger().info(str(pwm1) + ' ' + str(pwm2))
        self.data_handler_(pwm1, pwm2)


def main(args=None):
    rclpy.init(args=args)

    pwm_node = PWMNode()

    try:
           rclpy.spin(pwm_node)
    except KeyboardInterrupt:
           for i in range(1,101):
               pwm_node.data_handler_(0,0)

    pwm_node.data_handler_(0,0)
    pwm_node.destroy_node()
    rclpy.shutdown()

