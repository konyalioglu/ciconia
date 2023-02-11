import numpy as np


class ALT_State_Estimation:

        def __init__(self, P0, gamma, var_r, var_b, var_a):

            self.Ht_RNG = np.array([[1.0, 0.0, 0.0, 0.0, 0.0]])

            self.Ht_BAR = np.array([[1.0, 0.0, 0.0, 1.0, 0.0]])

            self.Ht_ACC = np.array([[0.0, 0.0, 1.0, 0.0, 1.0]])

            self.xt = np.array([[0.0],[0.0],[0.0],[0.0],[0.0]])
            self.gamma = gamma

            self.P = np.array(P0)

            self.F = np.eye(5)
            self.Q = np.zeros((5,5))
            self.wk= np.zeros((5,1))

            self.R_RNG = np.array([[var_r]])
            self.R_BAR = np.array([[var_b]])
            self.R_ACC = np.array([[var_a]])


        def KF_Predict(self, dt):
            '''
            The states are [z zdot b_barometer]
            x is current states
            '''

            self.F[0,1] = dt
            self.F[0,2] = dt**2 / 2
            self.F[1,2] = dt

            self.wk[0,0] = dt**3 / 6 * self.gamma
            self.wk[1,0] = dt**2 / 2 * self.gamma
            self.wk[2,0] = dt * self.gamma
            self.wk[3,0] = dt**3 / 6 * self.gamma
            self.wk[4,0] = dt * self.gamma

            Q = self.wk @ np.transpose(self.wk)

            self.P = self.F @ self.P @ np.transpose(self.F) + Q
            self.xt  = self.F @ self.xt
            return self.xt, self.P


        def RNG_Update(self, zt):
            St = self.Ht_RNG @ self.P @ np.transpose(self.Ht_RNG) + self.R_RNG
            Kt = self.P @ np.transpose(self.Ht_RNG) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_RNG @ self.xt)
            self.P = (np.eye(5) - Kt @ self.Ht_RNG) @ self.P
            return self.xt


        def BAR_Update(self, zt):
            St = self.Ht_BAR @ self.P @ np.transpose(self.Ht_BAR) + self.R_BAR
            Kt = self.P @ np.transpose(self.Ht_BAR) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_BAR @ self.xt)
            self.P = (np.eye(5) - Kt @ self.Ht_BAR) @ self.P
            return self.xt


        def ACC_Update(self, zt):
            St = self.Ht_ACC @ self.P @ np.transpose(self.Ht_ACC) + self.R_ACC
            Kt = self.P @ np.transpose(self.Ht_ACC) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_ACC @ self.xt)
            self.P = (np.eye(5) - Kt @ self.Ht_ACC) @ self.P
            return self.xt
