#! /usr/bin/python3

from operator import inv
import numpy as np
import scipy
from utils import *


class EKF:


    def __init__(self, dt, acc_cov, mag_cov, gyr_cov):

        self.dt = dt

        self.q = np.array([[0],[0],[0],[1]])
        self.xt = np.array([[0],[0],[0],[1],[0],[0],[0],[0],[0],[0]])

        self.prev_acc_measurement = np.array([[0],[0],[0]])
        self.prev_mag_measurement = np.array([[0],[0],[0]])

        self.w = np.array([[0],[0],[0]])

        self.state_transition = np.eye(10)


        self.Ra = np.array([[acc_cov[0], 0, 0],
                            [0, acc_cov[1], 0],
                            [0, 0, acc_cov[2]]])

        self.Rm = np.array([[mag_cov[0], 0, 0],
                            [0, mag_cov[1], 0],
                            [0, 0, mag_cov[2]]])

        self.Rg = np.array([[gyr_cov[0], 0, 0],
                            [0, gyr_cov[1], 0],
                            [0, 0, gyr_cov[2]]])


        self.Q = np.zeros((10,10))
        self.Q[4,4] = acc_cov[0]
        self.Q[5,5] = acc_cov[1]
        self.Q[6,6] = acc_cov[2]

        self.Q[7,7] = mag_cov[0]
        self.Q[8,8] = mag_cov[1]
        self.Q[9,9] = mag_cov[2]


        self.P = self.Q * 2
        

    def Jacobian(self, q, x, sensor):
        e  = np.linalg.norm(q[0:3])
        q1 = q[0,0]
        q2 = q[1,0]
        q3 = q[2,0]
        q4 = q[3,0]
        
        gmx = x[0,0]
        gmy = x[1,0]
        gmz = x[2,0]
        
        F = np.zeros((3,10))
        
        J1 = np.sqrt(1/(e**2 + q4**2))
        J2 = (2*gmx*q1 + 2*gmy*q2 + 2*gmz*q3) * J1
        J3 = J1**3
        J4 = 2*J1*gmz*q4
        J5 = 2*J1*gmz*q1
        J6 = 2*J1*gmy*q4
        J7 = 2*J1*gmy*q3
        J8 = 2*J1*gmy*q4
        J9 = 2*J1*gmy*q2
        J10=-2*J1*gmz*q2
        J11=-2*J1*gmy*q1
        J12=-2*J1*gmx*q3 
        J13=-q4**2
        J14= q3**2
        J15= 2*q3*q4
        J16= 2*q2*q4
        J17= q2**2
        J18= q1**2
        J19= 2*q1*q4
        J20= 2*q2*q3
        J21= 2*q1*q3
        J22= 2*q1*q2
        J23= J7 + J8 + J10
        J24= J4 + J9 + J11
        J25= J5 + J6 + J12
        
        F[0,0] = J2
        F[0,1] = 2 * J1 * gmy * q1 - J9 - J4
        F[0,2] = J25
        F[0,3] = J23 + J3 * gmx * q4 * (- q4**2 + J14 + J17 - J18) - J3 * gmy * q4 * (J15 + J22) + J3 * gmz * q4 * (J16 - J21)
        
        
        F[1,0] = J24
        F[1,1] = J2 
        F[1,2] = 2 * J1 * gmz * q2 - J8 - J7
        F[1,3] = J25 + J3 * gmy * q4 * (J13 + J14 - J17 + J18) - J3 * gmz * q4 * (J19 + J20) + J3 * gmx * q4 *(J15 - J22)
        
        
        F[2,0] = 2 * J1 * gmx * q3 - J6 - J5
        F[2,1] = J23
        F[2,2] = J2
        F[2,3] = J24 + J3 * gmz * q4 * (J13 - J14 + J17 + J18) - J3 * gmx * q4 * (J16 + J21) + J3 * gmy * q4 * (J19 - J20)
        if sensor == 'acc':
            F[0,4] = 1
            F[1,5] = 1
            F[2,6] = 1
        elif sensor == 'mag':
            F[0,7] = 1
            F[1,8] = 1
            F[2,9] = 1

        return F


    def Jacobian1(self, x):
        q1 = self.q[0,0]
        q2 = self.q[1,0]
        q3 = self.q[2,0]
        q4 = self.q[3,0]
        
        vx = x[0,0]
        vy = x[1,0]
        vz = x[2,0]

        J = np.zeros((3,10))

        J[0,0] = 2 * ( q1 * vx + q2 * vy + q3 * vz)
        J[0,1] = 2 * (-q2 * vx + q1 * vy - q4 * vz)
        J[0,2] = 2 * (-q3 * vx + q4 * vy + q1 * vz)
        J[0,3] = 2 * ( q4 * vx + q3 * vy - q2 * vz)

        J[1,0] = 2 * ( q2 * vx - q1 * vy + q4 * vz)
        J[1,1] = 2 * ( q1 * vx + q2 * vy + q3 * vz)
        J[1,2] = 2 * (-q4 * vx - q3 * vy + q2 * vz)
        J[1,3] = 2 * (-q3 * vx + q4 * vy + q1 * vz)

        J[2,0] = 2 * ( q3 * vx - q4 * vy - q1 * vz)
        J[2,1] = 2 * ( q4 * vx + q3 * vy - q2 * vz)
        J[2,2] = 2 * ( q1 * vx + q2 * vy + q3 * vz)
        J[2,3] = 2 * ( q2 * vx - q1 * vy + q4 * vz)

        J[0,7] = 1
        J[1,8] = 1
        J[2,9] = 1
        return J


    def Jacobian2(self, x):
        q1 = self.q[0,0]
        q2 = self.q[1,0]
        q3 = self.q[2,0]
        q4 = self.q[3,0]
        
        vz = x[2,0]

        J = np.zeros((3,10))

        J[0,0] =  2 * q3 * vz
        J[0,1] = -2 * q4 * vz
        J[0,2] =  2 * q1 * vz
        J[0,3] = -2 * q2 * vz

        J[1,0] =  2 * q4 * vz
        J[1,1] =  2 * q3 * vz
        J[1,2] =  2 * q2 * vz
        J[1,3] =  2 * q1 * vz

        J[2,0] = -2 * q1 * vz
        J[2,1] = -2 * q2 * vz
        J[2,2] =  2 * q3 * vz
        J[2,3] =  2 * q4 * vz

        J[0,4] = 1
        J[1,5] = 1
        J[2,6] = 1
        return J


    def update_process_cov(self):
        cross = np.array([[0, -self.q[2,0], self.q[1,0]],
                         [ self.q[2,0], 0, -self.q[0,0]],
                         [-self.q[1,0], self.q[0,0], 0]]) 
        
        Ek = np.vstack((cross + self.q[3,0] * np.eye(3), -np.transpose(self.q[0:3,0])))
        self.Q[0:4,0:4] = (self.dt/2)**2 * Ek @ self.Rg @ np.transpose(Ek)


    def angular_rate_input(self, p, q, r):
        self.w = np.array([[p],[q],[r]])


    def measurement(self, meas):
        return DCM_NB(self.q) @ meas


    def correctMAG(self, mag):
        Fmag = self.Jacobian(self.q, mag, 'mag')
        Kt   = self.P @ np.transpose(Fmag) @ np.linalg.inv(Fmag @ self.P @ np.transpose(Fmag) + self.Rm)
        
        zt = self.measurement(mag)
        self.xt = self.xt + Kt @ (self.measurement(mag) - self.prev_mag_measurement)

        self.xt[0:4] = self.xt[0:4] / np.linalg.norm(self.xt[0:4])
        self.qt = self.xt[0:4] / np.linalg.norm(self.xt[0:4])
        self.prev_mag_measurement = zt.copy()

        self.P = self.P - Kt @ Fmag @ self.P
        return self.xt


    def correctACC(self, acc):
        Facc = self.Jacobian(self.q, acc, 'acc')
        Kt   = self.P @ np.transpose(Facc) @ np.linalg.inv(Facc @ self.P @ np.transpose(Facc) + self.Ra)
        
        zt = self.measurement(acc)
        print(zt)
        self.xt = self.xt + Kt @ (self.measurement(acc) - self.prev_acc_measurement)
        self.xt[0:4] = self.xt[0:4] / np.linalg.norm(self.xt[0:4])
        self.q  = self.xt[0:4] / np.linalg.norm(self.xt[0:4])
        self.prev_acc_measurement = zt.copy()

        self.P = self.P - Kt @ Facc @ self.P
        return self.xt


    def predict(self):
        self.update_process_cov()
        Phi = self.Phi()
        self.xt = Phi @ self.xt
        self.P  = Phi @ self.P @ np.transpose(Phi) + self.Q
        self.xt[0:4] = self.xt[0:4] / np.linalg.norm(self.xt[0:4])
        self.q  = self.xt[0:4] / np.linalg.norm(self.xt[0:4])
        return self.xt


    def Phi(self):
        self.state_transition[0:4,0:4] = scipy.linalg.expm(omega(self.w)*self.dt)
        return self.state_transition