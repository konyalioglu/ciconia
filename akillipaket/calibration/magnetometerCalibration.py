import yaml
import rclpy
from rclpy.node import Node
import math
import time
import numpy as np
import FaBo9Axis_MPU9250



class magnetometerCalibration(Node):

    def __init__(self):
        super().__init__('Magnetometer_Calibration')

        self.mpu9250 = FaBo9Axis_MPU9250.MPU9250()
        self.variation = 5.93*(np.pi/180)
        self.counter = 0
        self.mag_x_sum = 0
        self.mag_y_sum = 0
        self.mag_z_sum = 0
        self.max_x = 0
        self.max_y = 0
        self.max_z = 0
        self.min_x = 0
        self.min_y = 0
        self.min_z = 0


    def sensor_call(self):
        mag = self.mpu9250.readMagnet()

        if mag['x']!=0 and mag['y']!=0 and mag['z']!=0:
            self.mag_x_sum += mag['x']
            self.mag_y_sum += mag['y']
            self.mag_z_sum += mag['z']
            self.counter += 1

            if(mag['x'] > self.max_x):
                self.max_x = mag['x']
            if(mag['y'] > self.max_y):
                self.max_y = mag['y']
            if(mag['z'] > self.max_z):
                self.max_z = mag['z']

            if(mag['x'] < self.min_x):
                self.min_x = mag['x']
            if(mag['y'] < self.min_y):
                self.min_y = mag['y']
            if(mag['z'] < self.min_z):
                self.min_z = mag['z']


    def calibrate(self):
        average_x = self.mag_x_sum / self.counter
        average_y = self.mag_y_sum / self.counter
        average_z = self.mag_z_sum / self.counter

        vmax_x = self.max_x - average_x
        vmax_y = self.max_y - average_y
        vmax_z = self.max_z - average_z
        vmin_x = self.min_x - average_x
        vmin_y = self.min_y - average_y
        vmin_z = self.min_z - average_z

        avgs_x = vmax_x - vmin_x
        avgs_y = vmax_y - vmin_y
        avgs_z = vmax_z - vmin_z

        avgs_rad = (avgs_x + avgs_y + avgs_z)/3

        x_scale = avgs_rad/avgs_x
        y_scale = avgs_rad/avgs_y
        z_scale = avgs_rad/avgs_z
        print('\nScales : ', str(x_scale) , ', ' , str(y_scale) , ', ' , str(z_scale))
        print('\nAverage values:  ', str(average_x),', ', str(average_y), ', ', str(average_z))
        print('\nVariation: ', str(self.variation))
        data = {'averages':{'x':average_x, 'y':average_y, 'z':average_z},
                'scaling':{'x':x_scale, 'y':y_scale, 'z':z_scale},
                'variation': self.variation}

        self.write_yaml(data)


    def write_yaml(self, data):
        with open('src/akillipaket/calibration/calibrationFiles/magnetometer_calibration.yaml', 'a') as f:
            f.truncate(0)
            yaml.dump(data, f, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)

    magnet = magnetometerCalibration()

    freq = 30
    calibration_duration = 120
    rate = magnet.create_rate(1/freq)
    rate_counter = 0

    magnet.get_logger().info('Calibration started and it will last ' + str(calibration_duration) + 'seconds')
    while True:
        if rate_counter >= calibration_duration*freq:
            magnet.calibrate()
            magnet.get_logger().info('Magnetometer calibration has been done!')
            break
        rate_counter += 1
        magnet.sensor_call()
        time.sleep(1/freq)


    magnet.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


