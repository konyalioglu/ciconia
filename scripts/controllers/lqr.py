#! /usr/bin/python3

"""
Created on Wed Apr 28 11:12:36 2021

@author: turan
"""

import numpy as np
import math
from scipy.linalg import expm
import scipy.linalg
from scipy import signal


class LinearQuadraticRegulator():
    
    def __init__(self, A, B, C):
        
        self.A = A
        self.B = B
        self.C = C       
        
        self.n = self.A.shape[0]
        self.m = self.B.shape[1]
        self.q  = self.C.shape[0]
        print('LQR initialization is completed!')
        
        
    '''    
    def __del__(self):
        print('Something went wrong and destructor is called!')
       ''' 
    
    def lqr(self, Q, R):
        """
        Solve the continuous time lqr controller.
         
        dx/dt = A x + B u
        
        cost = integral x.T*Q*x + u.T*R*u
              
        """
        
        #solve the ricatti equation
        P = np.array(scipy.linalg.solve_continuous_are(self.A, self.B, Q, R))
         
        #compute the LQR gain
        self.K = np.array(scipy.linalg.inv(R)@(self.B.T@P))
         
        eigVals, eigVecs = scipy.linalg.eig(self.A - self.B@self.K)
         
        return self.K, P, eigVals
 
    
    def dlqr(self, Q,R):
        """        
        Solve the discrete time lqr controller.
         
         
        x[k+1] = A x[k] + B u[k]
         
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        """

        #solve the ricatti equation
        P = np.array(scipy.linalg.solve_discrete_are(self.A, self.B, Q, R))
         
        #compute the LQR gain
        Kd = np.array(scipy.linalg.inv(self.B.T@P@self.B+R)@(self.B.T@P@self.A))
         
        eigVals, eigVecs = scipy.linalg.eig(self.A-self.B@Kd)
         
        return Kd, P, eigVals
    
    
    def lqrd(self, Q, R, dt):
        
        """
        
        Solve the discrete time lqr controller.
         
         
        x[k+1] = A x[k] + B u[k]
         
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        
        """
        
        
        D = np.zeros((self.q, self.m), dtype = 'float32')
        sys  = signal.StateSpace(self.A, self.B, self.C, D)
        sysd = sys.to_discrete(dt)
        Ad = sysd.A
        Bd = sysd.B
        Cd = sysd.C
        Dd = sysd.D
        
        #solve the ricatti equation
        P = np.matrix(scipy.linalg.solve_discrete_are(Ad, Bd, Q, R))
         
        #compute the LQR gain
        Kd = np.matrix(scipy.linalg.inv(Bd.T@P@Bd+R)@(Bd.T@P@Ad))
         
        eigVals, eigVecs = scipy.linalg.eig(Ad-Bd@Kd)
         
        return Kd, P, eigVals

         
    def compute_continuous_optimalinput(self, x, ref_x):
        return -self.K @ (x - ref_x)
    
    
    def compute_discrete_optimalinput(self, x, ref_x):
        return -self.Kd @ (x - ref_x)
        
        
