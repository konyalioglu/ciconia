import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
from std_msgs.msg import String, Float64MultiArray
import FaBo9Axis_MPU9250
from sensor_msgs.msg import Imu 


class Magnetometer_Node(Node):


    def __init__(self):
        super().__init__('Magnetometer')
        self.magnetometer_publisher_ = self.create_publisher(Float64MultiArray, '/akillipaket/IMU/Magnetometer', 4)
        self.mpu9250 = FaBo9Axis_MPU9250.MPU9250()
        self.magnetometer_msg = Float64MultiArray()
        self.variation = 4.528986*(np.pi/180)
        self.angle_offset = 65


    def wrap(self, angle, offset):
        if 0 < angle < offset:
            yaw = angle + 360 - offset
            return yaw

        elif angle - offset < 0:
            yaw = 360 - angle + offset
            return yaw
        else:
            yaw = angle - offset
            return yaw


    def sensor_callback(self):
        mag = self.mpu9250.readMagnet()
        self.mpu9250.readGyro()

        if mag['x']!=0 and mag['y']!=0 and mag['z']!=0:
            mag = [(mag['x']+25.913)*1.0005692243075357, (mag['y']-15.919)*0.9638164899280269, (mag['z']-12.6975)*1.0383924935620594]
            azimuth = np.arctan2( mag[0],mag[1] ) * 180 / np.pi + 180
            yaw = self.wrap(azimuth, self.variation + self.angle_offset) #calibrated to compass
            
            yaw = yaw - 180

            self.magnetometer_msg.data = [mag[0], mag[1], mag[2]] 

            self.magnetometer_publisher_.publish(self.magnetometer_msg)
            self.get_logger().info('Publishing: "%s"' % yaw)



def main(args=None):
    rclpy.init(args=args)

    magnet = Magnetometer_Node()

    while rclpy.ok():
        magnet.sensor_callback()

    magnet.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
