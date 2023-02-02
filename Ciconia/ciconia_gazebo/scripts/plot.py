#! /usr/bin/python3

"""
Created on Mon Jun  7 15:18:11 2021

@author: turan
"""


import pandas as pd
import os
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


dir_path = os.path.dirname(os.path.realpath(__file__))

rel_path = '/../sim_data/'
abs_path = dir_path + rel_path

state_data = pd.read_csv(abs_path + 'data_s.csv')
state_data = state_data.to_numpy()

input_data = pd.read_csv(abs_path + 'data_i.csv')
input_data = input_data.to_numpy()


plt.figure(0)
filename = 'angular_rates'
plt.plot(state_data[:, 0], state_data[:, 8], label="p")
plt.plot(state_data[:, 0], state_data[:, 10], label="q")
plt.plot(state_data[:, 0], state_data[:, 12], label="r")
plt.title('Angular Rate')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('angular rate [rad/s]')
plt.plot()
plt.savefig(abs_path + filename + '.png', dpi=300)


plt.figure(1)
filename = 'euler_angles'
plt.plot(state_data[:, 0], state_data[:, 7], label="phi")
plt.plot(state_data[:, 0], state_data[:, 9], label="theta")
plt.plot(state_data[:, 0], state_data[:, 11], label="psi")
plt.title('Euler Angles')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Euler Angles [rad]')
plt.plot()
plt.savefig(abs_path + filename + '.png', dpi=300)

plt.figure(2)
filename = 'linear_velocities'
plt.plot(state_data[:, 0], state_data[:, 2], label="u")
plt.plot(state_data[:, 0], state_data[:, 4], label="v")
plt.plot(state_data[:, 0], state_data[:, 6], label="w")
plt.title('Linear Velocities')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Linear Velocities [m/s]')
plt.plot()
plt.savefig(abs_path + filename + '.png', dpi=300)

plt.figure(3)
filename = 'altitude'
plt.plot(state_data[:, 0], -state_data[:, 5], label="h")
plt.title('Altitude')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Altitude [m]')
plt.plot()
plt.savefig(abs_path + filename + '.png', dpi=300)

plt.figure(4)
filename = 'planar_position'
plt.plot(state_data[:, 0], state_data[:, 1], label="x")
plt.plot(state_data[:, 0], state_data[:, 3], label="y")
plt.title('Planar Position')
plt.legend()
plt.grid()
plt.xlabel('time [sec]')
plt.ylabel('Position [m]')
plt.plot()
plt.savefig(abs_path + filename + '.png', dpi=300)



fig = plt.figure(tight_layout=True)
fig.suptitle("Flight Control Input Figures", fontsize=16)
filename = 'u_flight'
gs = gridspec.GridSpec(2, 2)

ax = fig.add_subplot(gs[0, 0])
ax.plot(input_data[:, 0], input_data[:, 1], label="de")
ax.set_title("Elevator Input")
plt.legend()
plt.grid()
ax.grid(which='minor')
ax.set_ylabel('de [rad]')
ax.set_xlabel('time [s]')

ax = fig.add_subplot(gs[0, 1])
ax.plot(input_data[:, 0], input_data[:, 2], label="dt")
ax.set_title("Forward Thrust Input ")
plt.legend()
plt.grid()
# ax.grid(which='minor', color='#CCCCCC', linestyle=':')
ax.grid(which='minor')
ax.set_ylabel('dt [N]')
ax.set_xlabel('time [s]')

ax = fig.add_subplot(gs[1, 0])
ax.plot(input_data[:, 0], input_data[:, 5], label="da")
ax.set_title("Aileron Input")
plt.legend()
plt.grid()
ax.grid(which='minor')
ax.set_ylabel('da [rad]')
ax.set_xlabel('time [s]')

ax = fig.add_subplot(gs[1, 1])
ax.plot(input_data[:, 0], input_data[:, 6], label="dr")
ax.set_title("Rudder Input")
plt.legend()
plt.grid()
ax.set_ylabel('dr [rad]')
ax.set_xlabel('time [s]')

fig.align_labels()
plt.savefig(abs_path + filename + '.png', dpi=300)

fig = plt.figure(tight_layout=True)
fig.suptitle("Quadrotor Mode Control Input Figures", fontsize=16)
filename = 'u_quadrotor'
gs = gridspec.GridSpec(2, 2)

ax = fig.add_subplot(gs[0, 0])
ax.plot(input_data[:, 0], input_data[:, 3], label="u1")
ax.set_title("Upward Thrust Input")
plt.legend()
plt.grid()
ax.grid(which='minor')
ax.set_ylabel('u1 [N]')
ax.set_xlabel('time [s]')

ax = fig.add_subplot(gs[0, 1])
ax.plot(input_data[:, 0], input_data[:, 7], label="u2")
ax.set_title("Rolling Input ")
plt.legend()
plt.grid()
# ax.grid(which='minor', color='#CCCCCC', linestyle=':')
ax.grid(which='minor')
ax.set_ylabel('u2 [N]')
ax.set_xlabel('time [s]')

ax = fig.add_subplot(gs[1, 0])
ax.plot(input_data[:, 0], input_data[:, 4], label="u3")
ax.set_title("Pitching Input")
plt.legend()
plt.grid()
ax.grid(which='minor')
ax.set_ylabel('u3 [N]')
ax.set_xlabel('time [s]')

ax = fig.add_subplot(gs[1, 1])
ax.plot(input_data[:, 0], input_data[:, 8], label="u4")
ax.set_title("Yawing Input")
plt.legend()
plt.grid()
ax.set_ylabel('u4 [N.m]')
ax.set_xlabel('time [s]')
fig.align_labels()
plt.savefig(abs_path + filename + '.png', dpi=300)
