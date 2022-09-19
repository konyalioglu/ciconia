import numpy as np

class State_Estimation:


        def __init__(self, P0, gamma_t, gamma_r, Qt_gps, Qt_imu, Qt_mag, Qt_aru, Qt_aru2):

            self.Ht_GPS = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

            self.Ht_IMU = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

            self.Ht_ARU2= np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])

            self.Ht_ARU = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])

            self.Ht_MAG = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]])

            self.xt = np.array([[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]])
            self.gamma_t = gamma_t
            self.gamma_r = gamma_r
            self.P = np.array(P0)

            self.F = np.eye(14)
            self.Q = np.zeros((14,14))
            self.B = np.array([[1],[1],[1]])
            self.wk= np.zeros((14,1))

            self.R_imu = Qt_imu
            self.R_gps = Qt_gps
            self.R_mag = Qt_mag
            self.R_aru = Qt_aru
            self.R_aru2= Qt_aru2


        def KF_Predict(self, dt):
            '''
            The states are [x y z xdot ydot zdot xdotdot ydotdot zdotdot psi psidot]
            x is current states
            '''

            self.F[0,3] = dt
            self.F[0,6] = dt*dt/2
            self.F[1,4] = dt
            self.F[1,7] = dt*dt/2
            self.F[2,5] = dt
            self.F[2,8] = dt*dt/2
            self.F[3,6] = dt
            self.F[4,7] = dt
            self.F[5,8] = dt
            self.F[9,10]= dt

            self.wk[0:3] = dt**3 / 6 * self.B * self.gamma_t
            self.wk[3:6] = dt**2 / 2 * self.B * self.gamma_t
            self.wk[6:9] = dt * self.B * self.gamma_t
            self.wk[9]   = dt**2 / 2 * self.gamma_r
            self.wk[10]  = dt * self.gamma_r
            self.wk[11:14] = dt * self.B * self.gamma_t

            Q = self.wk @ np.transpose(self.wk)

            self.P = self.F @ self.P @ np.transpose(self.F) + Q
            self.xt  = self.F @ self.xt
            return self.xt


        def GPS_Update(self, zt):
            St = self.Ht_GPS @ self.P @ np.transpose(self.Ht_GPS) + self.R_gps
            Kt = self.P @ np.transpose(self.Ht_GPS) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_GPS @ self.xt)
            self.P = (np.eye(14) - Kt @ self.Ht_GPS) @ self.P
            return self.xt


        def IMU_Update(self, zt):
            St = self.Ht_IMU @ self.P @ np.transpose(self.Ht_IMU) + self.R_imu
            Kt = self.P @ np.transpose(self.Ht_IMU) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_IMU @ self.xt)
            self.P = (np.eye(14) - Kt @ self.Ht_IMU) @ self.P
            return self.xt


        def MAG_Update(self, zt):
            St = self.Ht_MAG @ self.P @ np.transpose(self.Ht_MAG) + self.R_mag
            Kt = self.P @ np.transpose(self.Ht_MAG) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_MAG @ self.xt)
            self.P = (np.eye(14) - Kt @ self.Ht_MAG) @ self.P
            return self.xt


        def ARU_Update(self, zt):
            St = self.Ht_ARU @ self.P @ np.transpose(self.Ht_ARU) + self.R_aru
            Kt = self.P @ np.transpose(self.Ht_ARU) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_ARU @ self.xt)
            self.P = (np.eye(14) - Kt @ self.Ht_ARU) @ self.P
            return self.xt

        def ARU2_Update(self, zt):
            St = self.Ht_ARU2 @ self.P @ np.transpose(self.Ht_ARU2) + self.R_aru2
            Kt = self.P @ np.transpose(self.Ht_ARU2) @ np.linalg.inv(St)
            self.xt = self.xt + Kt @ (zt - self.Ht_ARU2 @ self.xt)
            self.P = (np.eye(14) - Kt @ self.Ht_ARU2) @ self.P
            return self.xt
