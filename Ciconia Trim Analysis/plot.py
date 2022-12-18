# -*- coding: utf-8 -*-
"""
Created on Thu Dec 15 22:39:11 2022

@author: turan
"""


import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator
import numpy as np

import os
from scipy.io import loadmat

from aerodynamics import aerodynamics
from propulsion import pair, propulsion



dir_path = os.path.dirname(os.path.realpath(__file__))  
prop_data = loadmat(dir_path + '/MN801S_26x85_PropulsionData_py.mat')  

throttle_BP = np.array(prop_data['MN801S_26x85_PropulsionData']['throttle'][0][0]).flatten()
RPM = np.array(prop_data['MN801S_26x85_PropulsionData']['RPM'][0][0]).flatten()
thrust = np.array(prop_data['MN801S_26x85_PropulsionData']['thrust'][0][0]).flatten()
torque = np.array(prop_data['MN801S_26x85_PropulsionData']['torque'][0][0]).flatten()


plt.figure(0)
filename = 'Thrust vs Throttle'
plt.plot(throttle_BP, thrust, label="Thrust")
plt.title('Thrust')
plt.legend()
plt.grid()
plt.xlabel('throttle [0-1]')
plt.ylabel('thrust [g]')
plt.plot()
plt.savefig(filename + '.png', dpi=600)

plt.figure(1)
filename = 'Torque vs Throttle'
plt.plot(throttle_BP, torque, 'tab:green', label="Torque")
plt.title('Torque')
plt.legend()
plt.grid()
plt.xlabel('throttle [0-1]')
plt.ylabel('torque [N.m]')
plt.plot()
plt.savefig(filename + '.png', dpi=600)

plt.figure(2)
filename = 'RPM vs Throttle'
plt.plot(throttle_BP, RPM, 'tab:red', label="RPM")
plt.title('RPM')
plt.legend()
plt.grid()
plt.xlabel('throttle [0-1]')
plt.ylabel('RPM [rev/min]')
plt.plot()
plt.savefig(filename + '.png', dpi=600)


fig, axs = plt.subplots(3)
axs[0].plot(throttle_BP, thrust)
axs[0].set_title('Trust')
axs[0].grid()
axs[0].legend()
axs[0].set(xlabel='Throttle', ylabel='Thrust [gr]')

axs[1].plot(throttle_BP, torque, 'tab:green')
axs[1].set_title('Axis [1, 0]')
axs[1].grid()
axs[2].plot(throttle_BP, RPM, 'tab:red')
axs[2].set_title('Axis [1, 1]')
axs[2].grid()
plt.tight_layout()


prop_data = loadmat(dir_path + '/AT7217_21x10_PropulsionData_py.mat')   

throttle_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['throttle'][0][0]).flatten()
RPM_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM_BP'][0][0]).flatten()
RPM = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM'][0][0]).flatten()
Mach_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['Mach_BP'][0][0]).flatten()
thrust = np.array(prop_data['AT7217_21x10_PropulsionData']['thrust'][0][0])
torque = np.array(prop_data['AT7217_21x10_PropulsionData']['torque'][0][0])


filename = 'fix_prop'
V,RPM = np.meshgrid(Mach_BP * 343,RPM_BP)
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(V, RPM, np.transpose(thrust), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.view_init(30, 190)
fig.colorbar(surf, shrink=0.5, aspect=5)
ax.plot_wireframe(V, RPM, np.transpose(thrust),  linewidth=0.2)
ax.set(xlabel='Vt', ylabel='RPM')
ax.set_title('AT7215 - APC21x10 Thrust Graph at 20m/s')
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)


dir_path = os.path.dirname(os.path.realpath(__file__))
aero_data = loadmat(dir_path + '/aerodynamic_database.mat')

mach = np.array(aero_data['data']['mach'][0][0][0], dtype='float32')
beta = np.array(aero_data['data']['beta'][0][0][0], dtype='float32') * np.pi / 180
alpha = np.array(aero_data['data']['alpha'][0][0][0], dtype='float32') * np.pi / 180

CD = np.array(aero_data['data']['CD'][0][0], dtype='float32')
CL = np.array(aero_data['data']['CL'][0][0], dtype='float32')
Cm = np.array(aero_data['data']['Cm'][0][0], dtype='float32')
Cn = np.array(aero_data['data']['Cn'][0][0], dtype='float32')
Cl = np.array(aero_data['data']['Cl'][0][0], dtype='float32')
Cy = np.array(aero_data['data']['Cy'][0][0], dtype='float32')

CL = CL[:,:,3]
CD = CD[:,:,3]
Cm = Cm[:,:,3]
Cn = Cn[:,:,3]
Cl = Cl[:,:,3]
CY = Cy[:,:,3]

Clp = float(aero_data['data']['Clp'][0][0][0])
Clr = float(aero_data['data']['Clr'][0][0][0])
Clda = float(aero_data['data']['Clda'][0][0][0])
Cldr = float(aero_data['data']['Cldr'][0][0][0])

Cyp = float(aero_data['data']['Cyp'][0][0][0])
Cyr = float(aero_data['data']['Cyr'][0][0][0])
Cydr = float(aero_data['data']['Cydr'][0][0][0])

CX = np.zeros((CL.shape[0], CL.shape[1]))
CZ = np.zeros((CL.shape[0], CL.shape[1]))

for i in range(CL.shape[0]):
    for j in range(CL.shape[1]):
        CX[i,j] = -CD[i,j] * np.cos(alpha[i]) - CL[i,j] * np.sin(alpha[i])
        CZ[i,j] = -CD[i,j] * np.sin(alpha[i]) - CL[i,j] * np.cos(alpha[i])

alpha, beta = np.meshgrid(alpha * 180 / np.pi,beta * 180 / np.pi)

#CL
filename = 'CL'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(CL), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(CL),  linewidth=0.2)
ax.view_init(20, 130)
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('CL')
ax.set(xlabel='\u03B1', ylabel='\u03B2')
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)
#CD
filename = 'CD'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(CD), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(CD),  linewidth=0.2)
ax.view_init(35, 120)
ax.set(xlabel='\u03B1', ylabel='\u03B2')
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('CD')

fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)


#CX
filename = 'CX'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(CX), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(CX),  linewidth=0.2)
ax.view_init(20, 120)
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('CX')
ax.set(xlabel='\u03B1', ylabel='\u03B2')
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)

#CZ
filename = 'CZ'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(CZ), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(CZ),  linewidth=0.2)
ax.view_init(30, 45)
ax.set(xlabel='\u03B1', ylabel='\u03B2')
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('CZ')
fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)


#Cm
filename = 'Cm'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(Cm), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(Cm),  linewidth=0.2)
ax.view_init(25, 110)
ax.set(xlabel='\u03B1', ylabel='\u03B2')
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('Cm')

fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)
#Cn
filename = 'Cn'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(Cn), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(Cn),  linewidth=0.2)
ax.view_init(35, 200)
ax.set(xlabel='\u03B1', ylabel='\u03B2')
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('Cn')

fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)

#Cl
filename = 'Cl'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(Cl), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(Cl),  linewidth=0.2)
ax.view_init(35, 230)
ax.set(xlabel='\u03B1', ylabel='\u03B2')
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('Cl')

fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)

#CY
filename = 'CY'
fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
surf = ax.plot_surface(alpha, beta, np.transpose(CY), cmap=cm.rainbow, linewidth=0, antialiased=False)
ax.plot_wireframe(alpha, beta, np.transpose(CY),  linewidth=0.2)
ax.view_init(35, 120)
ax.set(xlabel='\u03B1', ylabel='\u03B2')
ax.zaxis.set_major_formatter('{x:.02f}')
ax.set_title('CY')

fig.colorbar(surf, shrink=0.5, aspect=5)
plt.tight_layout()
plt.savefig(filename + '.png', dpi=600)
