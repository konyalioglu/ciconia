
"""
Created on Mon May  3 11:12:52 2021

@author: turan
"""


import numpy as np
import scipy.linalg
from scipy import signal
from model_predictive_controller import Model_Predictive_Control



A_flight = np.array([[  -0.0249373,	0.434151,	-0.306821,	-9.80468,	0,	-0.0134076,	0,	1.5049e-07,	0,	0],
                     [   -1.0119,	-4.03576,	15.9232,	-0.188924,	0,	0.201211,	-1.5049e-07,	0,	0,	0],
                     [   0.0480231,	-2.5199,	-6.97358,	0,	0,	0.0223937,	-4.21831e-11,	-8.0406e-11,	0,	0],
                     [   0,	0,	1,	0,	0,	0,	0,	0,	6.85969e-11,	0],
                     [   0.0192652,	-0.999814,	0,	18,	0,	0,	0,	0,	-1.49012e-07,	0],
                     [   1.35817e-09,	-3.16569e-09,	0,	0,	0,	-0.172437,	0.329692,	-17.7009,	9.80468,	0],
                     [   -4.43797e-09,	9.09345e-09,	5.28334e-11,	0,	0,	0.505768,	-11.1232,	3.14313,	0,	0],
                     [   -3.85613e-09,	-2.59574e-09,	1.09466e-11,	0,	0,	0.465515,	-1.75368,	-0.470645,	0,	0],
                     [   0,	0,	0,	-6.86224e-11,	0,	0,	1,	0.0192688,	1.24375e-12,	0],
                     [   0,	0,	0,	-1.32203e-12,	0,	0,	0,	1.00019,	6.45594e-11,	0]]) 

                     
B_flight = np.array([[  0.163105,	0.04,	0,	0],
                     [  -8.46473,	0,	0,	0],
                     [  -15.2978,	0,	0,	0],
                     [  0,	0,	0,	0],
                     [  0,	0,	0,	0],
                     [  0,	0,	0,	-2.23143],
                     [  0,	0,	-34.0432,	0.626925],
                     [  0,	0,	-2.89961,	5.96312],
                     [  0,	0,	0,	0],
                     [  0,	0,	0,	0]])

                     
C_flight = np.eye(A_flight.shape[0])
D_flight = np.zeros((A_flight.shape[0], B_flight.shape[1])) 

sys  = signal.StateSpace(A_flight, B_flight, C_flight, D_flight)
sysd = sys.to_discrete(1/30)

Ad = sysd.A.copy()
e, v = scipy.linalg.eig(Ad)

Q = np.diag(np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1]))
R = np.diag(np.array([1, 1, 1, 1])) 

Np = 30
Nc = 6

x_mins = -np.array([[30],[30],[1],[0.5],[50],[10],[5],[5],[0.5],[20]])
x_maxs = np.array([[30],[30],[1],[0.5],[50],[10],[5],[5],[0.5],[20]])
x_cons =  np.concatenate((x_mins, x_maxs), axis=0)

u_mins = -np.array([[0.5],[5],[0.5],[0.5]])*2
u_maxs = np.array([[0.5],[200],[0.5],[0.5]])*2
u_cons =  np.concatenate((u_mins, u_maxs), axis=0)

deltau_mins = -np.array([[0.1],[100],[0.1],[0.1]])
deltau_maxs = np.array([[0.1],[100],[0.1],[0.1]])
deltau_cons =  np.concatenate((deltau_mins, deltau_maxs), axis=0)

mpc = Model_Predictive_Control(A_flight, B_flight, C_flight, Np, Nc, Q, R, 30)
mpc.initialize_model_contraints(x_cons, u_cons, deltau_cons)
mpc.initialize_mpc_controller()
Qb, Aeq = mpc.initialize_infinite_horizon()

#Lr, Mr, n = mpc.stable_unstable_decomposition(sysd.A)

#Adsoru = Mr @ Lr @ np.linalg.inv(Mr)

#Qffg = Adsoru - Ad

#Qb, Wbs = mpc.infinite_horizon(Mr, Lr, n)
