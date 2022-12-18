# -*- coding: utf-8 -*-
"""
Created on Fri Sep 23 13:43:21 2022

@author: Turan Konyalioglu
"""


from utils import *
from trim_analysis import trim_analysis
from numerical_linearization import numerical_linearization
import numpy as np
from eom import eom
from models import models

from propulsion import pair, propulsion
from gravitational import gravitational
from aerodynamics import aerodynamics
from gyroscopic import gyroscopic

from collections import defaultdict


grav = gravitational(30)
aero = aerodynamics()
prop = propulsion()
gyro = gyroscopic()

delta = 0.1
tol = 1e-6
precision = 17

trim = trim_analysis()
linearization = numerical_linearization(delta, tol, precision)
m = 30
g = 9.8065
density = 1.225
speedofsound = 343

model = models()
eom = eom(model.m, model.inertia_tensor)



c       = 343
Vt      = 21
mach    = Vt / c
phi     = 0
theta   = 0
psi     = 0
lambdaf = 0

euler_angles = np.array([[phi],[theta],[psi]])

alpha = 0
beta  = 0
wind_param   = np.array([[Vt],[alpha],[beta]])

p = 0
q = 0
r = 0
body_rates = np.array([[p],[q],[r]])

de = 0.0
da = 0.0
dr = 0.5
cont_surf  = np.array([[de],[da],[dr]])

u_p = np.array([[0],[0],[0],[0],[5.88282213e-01]])


Fp, Mp = model.calculate_propulsive_forces_and_moments_trim(u_p, mach)
Fa, Ma = aero.aerodynamic_model(wind_param, body_rates, cont_surf)
Ft, Mt = model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u_p)
print(Fa, Ma)


Vt      = 21
mach    = Vt / c
phi     = 0
theta   = 0
psi     = 0
lambdaf = 0
phi = 0

x0 = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
bnds = ((-0.13, 0.13), (-0.13, 0.13), (None, None), (None, None), (None, None), (None, None), (None, None), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5), (0.0, 1.0))
ts = trim.optimize_simplified_straight_flight(mach, x0, phi = 0, bnds=bnds)
trim.check_trim_states(ts, mach * c, phi, lambdaf)
xdot, x, u, d = trim.convert_trim_states2model_states_flight(ts, mach, phi)
E, Ap, Bp, Dp = linearization.numerical_linearization(xdot, x, u, d, 'flight')

Aflight = -np.linalg.inv(E) @ Ap
Bflight = -np.linalg.inv(E) @ Bp
Dflight = -np.linalg.inv(E) @ Dp
Cflight = np.eye(Aflight.shape[0])
w, v = np.linalg.eig(Aflight)


Vt      = 21
mach    = Vt / c
phi     = 45
theta   = 0
psi     = 0
lambdaf = 0
phi = 0
lambdaf = 0
x0 = [0.02, 0.02, 0, 0.0, 0, 0, 1]
bnds = ((-0.13, 0.13), (None, None), (None, None), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5), (0.0, 1.0))

phi = 45 * np.pi / 180
ts = trim.optimize_steady_state_turn(mach, x0, phi, bnds=bnds)
trim.check_steady_turn_trim_states(ts, mach * c, phi)
xdot, x, u, d = trim.convert_steadyturn_trim_states2model_states_flight(ts, mach, phi)
E, Ap, Bp, Dp = linearization.numerical_linearization(xdot, x, u, d, 'flight')


Aturn = -np.linalg.inv(E) @ Ap
Bturn = -np.linalg.inv(E) @ Bp
Dturn = -np.linalg.inv(E) @ Dp
Cturn = np.eye(Aturn.shape[0])



Vt      = 5
mach    = Vt / c
phi     = 0
theta   = 0
psi     = 0
lambdaf = 0
phi = 0
lambdaf = 0

x0 = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 200, 0, 0, 0]
bnds = ((-0.13, 0.13), (-0.13, 0.13), (None, None), (None, None), (None, None), (None, None), (None, None), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5), (0, 1),(None, None),(None, None),(None, None),(None, None))

xtrim = trim.optimize_generalized_transition_flight(mach, x0, phi = 0, bnds=bnds)
trim.check_trim_states_transition(xtrim, mach * c)

xdot, x, u, d = trim.convert_trim_states2model_states_transition(xtrim, mach, 0)

delta = 0.1
tol = 1e-6
precision = 17
E, Ap, Bp, Dp = linearization.numerical_linearization(xdot, x, u, d, lin = 'transition')
Atransition = -np.linalg.inv(E) @ Ap
Btransition = -np.linalg.inv(E) @ Bp
Dtransition = -np.linalg.inv(E) @ Dp
w, v = np.linalg.eig(Atransition)




