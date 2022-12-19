
"""
Created on Mon Jun  7 15:18:11 2022

@author: turan
"""


import pandas as pd
import os
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


dir_path = os.path.dirname(os.path.realpath(__file__))

gps_data = pd.read_csv('gps_data.csv')
gps_data = gps_data.to_numpy()

mag_data = pd.read_csv('magnetometer_data.csv')
mag_data = mag_data.to_numpy()

euler_angles = pd.read_csv('euler_angles_data.csv')
euler_angles = euler_angles.to_numpy()

linear_accel = pd.read_csv('acceleration_data.csv')
linear_accel = linear_accel.to_numpy()

gyro = pd.read_csv('gyro_data.csv')
gyro = gyro.to_numpy()

kf = pd.read_csv('kalman_filter_data.csv')
kf = kf.to_numpy()

motor_data = pd.read_csv('motor_input_data.csv')
motor_data = motor_data.to_numpy()

aruco_position = pd.read_csv('aruco_position_data.csv')
aruco_position = aruco_position.to_numpy()

aruco_heading = pd.read_csv('aruco_position_data.csv')
aruco_heading = aruco_heading.to_numpy()


plt.figure(0)
filename = 'heading_data'
plt.plot(mag_data[:, 0], mag_data[:, 1], label="magnetometer")
plt.plot(aruco_heading[:, 0], aruco_heading[:, 1], label="aruco")
plt.title('Heading')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('heading [deg]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)


plt.figure(1)
filename = 'euler_angles'
plt.plot(euler_angles[:, 0], euler_angles[:, 1], label="Phi")
plt.plot(euler_angles[:, 0], euler_angles[:, 2], label="Theta")
plt.title('Euler Angles')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('euler angles [rad]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)


plt.figure(2)
filename = 'linear_accel'
plt.plot(linear_accel[:, 0], linear_accel[:, 1], label="x")
plt.plot(linear_accel[:, 0], linear_accel[:, 2], label="y")
plt.plot(linear_accel[:, 0], linear_accel[:, 3], label="z")
plt.title('Linear Acceleration')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('linear acceleration [m/s^2]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)

plt.figure(3)
filename = 'gyro'
plt.plot(gyro[:, 0], gyro[:, 1], label="p")
plt.plot(gyro[:, 0], gyro[:, 2], label="q")
plt.plot(gyro[:, 0], gyro[:, 3], label="r")
plt.title('Angular Rates')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('angular rates [rad/s]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)
print(np.var(gyro[1100:,1]))
print(np.var(gyro[1100:,2]))
print(np.var(gyro[1100:,3]))


plt.figure(4)
filename = 'kf_position'
plt.plot(kf[:, 0], kf[:, 1], label="x")
plt.plot(kf[:, 0], kf[:, 2], label="y")
plt.plot(kf[:, 0], kf[:, 3], label="z")
plt.title('Position')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Position [m]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)

plt.figure(5)
filename = 'kf_velocity'
plt.plot(kf[:, 0], kf[:, 4], label="xdot")
plt.plot(kf[:, 0], kf[:, 5], label="ydot")
plt.plot(kf[:, 0], kf[:, 6], label="zdot")
plt.title('Velocity')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Velocity [m/s]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)

plt.figure(6)
filename = 'kf_acceleration'
plt.plot(kf[:, 0], kf[:, 7], label="xdotdot")
plt.plot(kf[:, 0], kf[:, 8], label="ydotdot")
plt.plot(kf[:, 0], kf[:, 9], label="ydotdot")
plt.title('Acceleration')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Acceleration [m/s^2]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)

plt.figure(7)
filename = 'kf_psi'
plt.plot(kf[:, 0], kf[:, 10], label="psi")
plt.title('Psi')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Psi')
plt.plot()
plt.savefig(filename + '.png', dpi=300)


plt.figure(8)
filename = 'kf_psi_dot'
plt.plot(kf[:, 0], kf[:, 11], label="psidot")
plt.title('Psidot')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Psidot')
plt.plot()
plt.savefig(filename + '.png', dpi=300)



plt.figure(9)
filename = 'motor_input'
plt.plot(motor_data[:, 0], motor_data[:, 1], label="motor1")
plt.plot(motor_data[:, 0], motor_data[:, 2], label="motor2")
plt.title('Motor Inputs')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('PWM')
plt.plot()
plt.savefig(filename + '.png', dpi=300)


plt.figure(4)
filename = 'aruco_position'
plt.plot(aruco_position[:, 0], aruco_position[:, 1], label="x")
plt.plot(aruco_position[:, 0], aruco_position[:, 2], label="y")
plt.plot(aruco_position[:, 0], aruco_position[:, 3], label="z")
plt.title('Aruco Position')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Position [m]')
plt.plot()
plt.savefig(filename + '.png', dpi=300)
