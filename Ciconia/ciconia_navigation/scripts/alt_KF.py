import numpy as np


class ALT_State_Estimation:

        def __init__(self, P0, gamma, var_r, var_b):

            self.Ht_RNG = np.array([[1.0, 0.0, 0.0]])

            self.Ht_BAR = np.array([[1.0, 0.0, 1.0]])

            self.xt = np.array([[0.0],[0.0],[0.0]])
            self.gamma = gamma

            self.P = np.array(P0)

            self.F = np.eye(3)
            self.Q = np.zeros((3,3))
            self.wk= np.zeros((3,1))

            self.R_RNG = np.array([[var_r]])
            self.R_BAR = np.array([[var_b]])


        def KF_Predict(self, dt):
            '''
            The states are [z zdot b_barometer]
            x is current states
            '''

            self.F[0,1] = dt

            self.wk[0,0] = dt**2 / 2 * self.gamma
            self.wk[1,0] = dt * self.gamma
            self.wk[2,0] = dt**2 / 2 * self.gamma

            Q = self.wk @ np.transpose(self.wk)

            self.P = self.F @ self.P @ np.transpose(self.F) + Q
            self.xt  = self.F @ self.xt
            return self.xt, self.P


        def RNG_Update(self, zt):
            St = self.Ht_RNG @ self.P @ np.transpose(self.Ht_RNG) + self.R_RNG
            Kt = self.P @ np.transpose(self.Ht_RNG) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_RNG @ self.xt)
            self.P = (np.eye(3) - Kt @ self.Ht_RNG) @ self.P
            return self.xt


        def BAR_Update(self, zt):
            St = self.Ht_BAR @ self.P @ np.transpose(self.Ht_BAR) + self.R_BAR
            Kt = self.P @ np.transpose(self.Ht_BAR) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_BAR @ self.xt)
            self.P = (np.eye(3) - Kt @ self.Ht_BAR) @ self.P
            return self.xt
