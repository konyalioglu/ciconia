#! /usr/bin/python3
"""
Created on Tue Dec  1 18:20:02 2020

@author: turan
"""


import numpy as np
import math
import scipy.linalg
from scipy import signal
import cvxopt
from cvxopt import matrix


class Model_Predictive_Control:
    
    def __init__(self, A, B, C, Np, Nc, Q, P, delta_time, controlOption = 'Tracking'):
        
        self.dt = delta_time
        self.A  = np.array(A, dtype='float32')
        self.B  = np.array(B, dtype='float32')
        self.C  = np.array(C, dtype='float32')
                
        self.ns = self.A.shape[0]
        self.m  = self.B.shape[1]
        self.q  = self.C.shape[0]
        self.Np = Np
        self.Nc = Nc
        self.D  = np.zeros((self.q, self.m), dtype = 'float32')
        
        
        sys  = signal.StateSpace(self.A, self.B, self.C, self.D)
        sysd = sys.to_discrete(self.dt)
        self.Ad = sysd.A
        self.Bd = sysd.B
        self.Cd = sysd.C
        self.Dd = sysd.D
        
        self.Q  = np.array(Q, dtype='float32')
        self.P  = np.array(P, dtype='float32')
        self.R  = None
        
        self.uk0 = np.zeros((self.m, 1))
        self.uk1 = np.zeros((self.m, 1))
        self.xk  = np.zeros((self.ns, 1))
        self.reference_trajectory = np.zeros((self.Np * self.ns ,1))

        self.H = 0
        self.M = 0
        self.stabilization   = False
        self.infiniteHorizon = False
        self.terminalState   = False
        self.controlOption   = controlOption

    def stabilized_prediction(self, K):
        
        self.Kd = K
        self.Ad = self.Ad - self.Bd @ K
        self.Bd = self.Bd
        self.Cd = self.Cd
        self.stabilization = True
        return
    
    
    def define_terminal_cost(self, R):
        self.R = R
        return
        
        
    def mpc_gain_maciejowski(self):
        
        # number of states, inputs and outputs
        n  = self.Ad.shape[0]
        m  = self.Bd.shape[1]
        
        psi = np.array(self.Ad)        
        for i in range(2,self.Np+1):
            psi = np.concatenate((psi, np.linalg.matrix_power(self.Ad, i)),axis=0)
        
        
        for i in range(self.Np):
            store = np.zeros((n,m))
            
            for j in range(0,i+1):
                store = np.linalg.matrix_power(self.Ad, j) @ self.Bd + store
                
            if i == 0:
                gamma = store
            else:
                gamma = np.concatenate((gamma, store), axis=0)
        
        theta = np.zeros((self.Np * n, self.Nc * m))
        for i in range(0, self.Np):
            store = np.zeros((n,m))
            for k in range(0, self.Nc):
                if i >= k:
                    for j in range(i-k+1):
                        store = np.linalg.matrix_power(self.Ad, j) @ self.Bd + store
                    theta[i * n : (i+1) * n, k * m : (k+1) * m] = store
        return psi, gamma, theta
    
        
    def calculate_weight_matrices(self):
        n  = self.Ad.shape[0]
        m  = self.Bd.shape[1]
        
        Qbar = np.zeros((self.Np * n, self.Np * n))
        Pbar = np.zeros((self.Nc * m, self.Nc * m))
        
        for i in range(self.Np):
            Qbar[i*n: i*n+n, i*n: i*n+n] = self.Q
            if self.R is not None and i == self.Np-1:
                Qbar[i*n: i*n+n, i*n: i*n+n] = self.R
            
            if i <= self.Nc-1:
                Pbar[i*m: i*m+m, i*m: i*m+m] = self.P    

            
        return Qbar, Pbar
            

    def unconstrained_mpc_gain(self):
        """
        Gives the solution of the Unconstraint MPC Problem
        
        DeltaU = -1/2 / H * G
        
        where
            H is Hessian Matrix
            G is Gradient
            

        Returns
        -------
        Float
            K_uMPC is the MPC Gain that regulates the system.

        """
        
        n  = self.Ad.shape[0]
        m  = self.Bd.shape[1]
                
        Kfull = np.linalg.inv(self.H) @ np.transpose(self.theta) @ self.Qbar
        
        Nu = np.zeros((m, m*self.Nc))
        Nu[0:m, 0:m] = np.eye(m)
        
        print(Kfull.shape)
        self.K_uMPC = Nu @ Kfull       
        return self.K_uMPC
    
    
    def calculate_mpc_unconstraint_input(self, arg):
        """
        arg is xk if the controlOption is Regulator
        arg is Tracking Error if the controlOption is Tracking
        """
        
        if self.controlOption == 'Regulator':
            du          = -self.K_uMPC @ (self.psi @ arg + self.gamma @ self.uk0)
            u           = du + self.uk0 
            self.uk0    = u.copy()
        elif self.controlOption == 'Tracking':
            du          = -self.K_uMPC @ arg
            u           = du + self.uk0  
            self.uk0    = u.copy()
        
        return u      
    

    def mpc_qp_constraint_generator(self, theta):
    
        n  = self.Ad.shape[0]
        m  = self.Bd.shape[1]
        
        theta = np.array(theta, dtype='float32')
        
        C2 = np.zeros((self.Nc * m, self.Nc * m))
        
        for i in range(self.Nc):
            for j in range(self.Nc):
                if i >= j:
                    C2[i*m:(i+1)*m,j*m:(j+1)*m] = np.eye(m)
                    
        M1 = np.concatenate((-C2, C2), axis = 0)
        M2 = np.concatenate((-np.eye(self.Nc * m), np.eye(self.Nc * m)), axis = 0)
        M3 = np.concatenate((-theta, theta), axis = 0)
        
        M = np.concatenate((M1, M2), axis=0)
        M = np.concatenate((M, M3), axis=0)
        
        return M
    
    
    def qp_u_constraint_matrix(self):
        m  = self.Bd.shape[1]
        
        C2 = np.zeros((self.Nc * m, self.Nc * m))
        
        for i in range(self.Nc):
            for j in range(self.Nc):
                if i >= j:
                    C2[i*m:(i+1)*m,j*m:(j+1)*m] = np.eye(m)
                    
        M = np.concatenate((-C2, C2), axis = 0)
        return M
    
    
    def qp_du_constraint_matrix(self):
        m  = self.Bd.shape[1]
        M = np.concatenate((-np.eye(self.Nc * m), np.eye(self.Nc * m)), axis = 0)
        return M
    
    
    def qp_y_constraint_matrix(self, theta):
        M = np.concatenate((-theta, theta), axis = 0)
        return M
    
    
    def qp_u_constraint_vector(self):
        return
    
    
    def qp_du_constraint_vector(self):
        return
    
    
    def qp_y_constraint_vector(self, theta):
        return
    
    
    def mpc_qp_constraint_generator2(self, psi, gamma, xk, uk_1, x_cons, u_cons, deltau_cons):
        
        n  = self.Ad.shape[0]
        m  = self.Bd.shape[1]
        
        C1 = np.eye(m, dtype='float32')
        Umin = u_cons[0:m]
        Umax = u_cons[m:]
        
        deltaUmin = deltau_cons[0:m]
        deltaUmax = deltau_cons[m:]
        
        for i in range(2,self.Nc+1):
            C1 = np.concatenate((C1, np.eye(m)), axis=0)
            
            Umin = np.concatenate((Umin, u_cons[0:m]), axis=0)
            Umax = np.concatenate((Umax, u_cons[m:]), axis=0)
            
            deltaUmin = np.concatenate((deltaUmin, deltau_cons[0:m]), axis=0)
            deltaUmax = np.concatenate((deltaUmax, deltau_cons[m:]), axis=0)
            
        N1 = np.concatenate((-Umin, Umax), axis=0)
        N1 = N1 + np.concatenate((C1 @ uk_1, -C1 @ uk_1), axis=0)        
        N2 = np.concatenate((-deltaUmin, deltaUmax), axis=0)
        
        Ymin = x_cons[0:n]
        Ymax = x_cons[n:]
        
        for i in range(2,self.Np+1):
            
            Ymin = np.concatenate((Ymin, x_cons[0:n]), axis=0)
            Ymax = np.concatenate((Ymax, x_cons[n:]), axis=0)
        
        N3 = -Ymin + psi @ xk + gamma @ uk_1
        N3 = np.concatenate((N3, Ymax - psi @ xk - gamma @ uk_1), axis=0)

        N = np.concatenate((N1, N2), axis=0)
        N = np.concatenate((N, N3), axis=0)
    
        return N
        
        
    def create_reference_trajectory(self, set_point, filter_list, states):
        ns = set_point.shape[0]
        for i in range(ns):
            for j in range(self.Np-1):
                if j == 0:
                    self.reference_trajectory[i,0] = filter_list[i,0]**(j+1) * states[i,0] + (1 - filter_list[i,0]**(j+1)) *  set_point[i,0]
                else:
                    self.reference_trajectory[ns * j + i,0] = filter_list[i,0]**(j+1) * self.reference_trajectory[ns * (j-1) + i,0] + (1 - filter_list[i,0]**(j+1)) *  set_point[i,0]
                
        return self.reference_trajectory
    
    
    def qp_hildreth(self, H, f, Ac, B):
        n1, m1 = np.shape(Ac)

        eta = -np.linalg.inv(H/2) @ f
        
        ind = 0
        for i in range(n1):
            if Ac[i, :] @ eta > B[i]:
                ind += 1
        
        if ind == 0:
            return eta
        
        P = Ac @ np.linalg.inv(H) @ Ac.transpose()
        d = Ac @ np.linalg.inv(H) @ f + B
        
        n1, m1 = np.shape(d)
        lambda_var = np.zeros((n1, m1))
    
        error = 10
    
        for k in range(1):
            lambda_tmp = lambda_var.copy()
            for i in range(n1):
                w = P[i, :] @ lambda_var - P[i, i] * lambda_var[i]
                w = w + d[i]
                la = -w / P[i, i]
                lambda_var[i] = max(0, la)
    
            error = np.transpose(lambda_var - lambda_tmp) @ (lambda_var - lambda_tmp)
            if error < 10e-4:
                break
    
        eta = -np.linalg.inv(H) @ f - np.linalg.inv(H) @ np.transpose(Ac) @ lambda_var
        
        return eta
        
    
    def solve_qp_problem(self, H, f, Ac, B):
        sol = cvxopt.solvers.qp(matrix(H / 2), matrix(f), matrix(Ac), matrix(B))
        return np.array(sol['x'])
    
    
    def solve_qp_problem2(self, H, f, Ac, B, Aeq, Beq):
        sol = cvxopt.solvers.qp(matrix(H), matrix(np.transpose(f)), matrix(Ac), matrix(B), \
                                matrix(Aeq), matrix(Beq))
        print(type(sol['x']))
        return np.array(sol['x'])


    def initialize_mpc_controller(self):
        self.psi, self.gamma, self.theta = self.mpc_gain_maciejowski()
        self.Qbar, Pbar = self.calculate_weight_matrices()

        self.H = np.transpose(self.theta) @ self.Qbar @ self.theta + Pbar 
        self.M = self.mpc_qp_constraint_generator(self.theta)
        return 


    def initialize_model_contraints(self, x_cons, u_cons, deltau_cons):
        self.x_cons = x_cons
        self.u_cons = u_cons
        self.deltau_cons = deltau_cons
        return
    
    
    def calculate_qp_G(self, xk, trajectory):
        if trajectory is None or self.controlOption == 'Regulator':
            G = 2 * np.transpose(self.theta) @ self.Qbar @ (self.psi @ xk + self.gamma @ self.uk0)
        elif self.controlOption == 'Tracking':
            tracking_error = trajectory - self.psi @ xk - self.gamma @ self.uk0
            G = -2 * np.transpose(self.theta) @ self.Qbar @ tracking_error
        return G

	
    def calculate_control_input(self, xk, trajectory = None, option='CVXOPT'):
        G = self.calculate_qp_G(xk, trajectory)
        N = self.mpc_qp_constraint_generator2(self.psi, self.gamma, xk, self.uk0, self.x_cons, self.u_cons, self.deltau_cons)
        
        if option == 'Hildreth':
            Delta_u  = self.qp_hildreth(self.H, G, self.M, N)
        elif option == 'CVXOPT' and self.terminalState == False:
            Delta_u  = self.solve_qp_problem(self.H, G, self.M, N)
        elif option == 'CVXOPT' and self.terminalState == True:
            Beq      = self.calculate_terminal_state_equality_vector(xk, trajectory)
            Delta_u  = self.solve_qp_problem2(self.H, G, self.M, N, self.Aeq, Beq)

        deltau_mpc     = Delta_u[0:self.m]
        
        self.uk1 = deltau_mpc + self.uk0
        self.uk0  = self.uk1.copy()
        if self.stabilization == True:
            qk = -self.Kd @ xk
            u  = self.uk1 + qk
        else:
            u  = self.uk1   
        return u
    
    
    def calculate_terminal_state_equality_matrix(self):
        
        '''Static part of the terminal state equality constraint'''
        n = self.Bd.shape[0]
        m = self.Bd.shape[1]
        theta_hu = self.theta[n*(self.Nc - 1):n*(self.Nc),:]
        Aeq1 = self.Wbu @ theta_hu     
        return Aeq1
    
    
    def calculate_terminal_state_equality_vector(self, xk, trajectory):
        n = self.Bd.shape[0]
        m = self.Bd.shape[1]
        if trajectory is None or self.controlOption == 'Regulator':
            Beq = self.Wbu @ (self.psi_hu @ xk  + self.gamma_hu @ self.uk0)
        elif self.controlOption == 'Tracking':
            rk  = trajectory[n*(self.Nc - 1):n*(self.Nc),0].reshape(n, 1)
            Beq = self.Wbu @ (self.psi_hu @ xk  + self.gamma_hu @ self.uk0 - rk)
        
        return Beq
    
    
    def initialize_terminal_state_equality_constraint(self):
        n  = self.Ad.shape[0]
        self.Aeq = self.calculate_terminal_state_equality_matrix()
        self.gamma_hu = self.gamma[n*(self.Nc - 1):n*(self.Nc),:]
        self.psi_hu   = self.psi[n*(self.Nc - 1):n*(self.Nc),:]
        return
    
    
    def initialize_infinite_horizon(self):
        Lr, Mr, nus = self.stable_unstable_decomposition(self.Ad)
        self.Qb, self.Wbu     = self.infinite_horizon(Mr, Lr, nus)        
        self.Qbar[-self.Qb.shape[0]:,-self.Qb.shape[0]:] = self.Qb           
        self.infiniteHorizon = True
        if not self.Wbu is None:
            self.initialize_terminal_state_equality_constraint()
        
        return self.Qb, self.Wbu
        
        
    def infinite_horizon(self, V, J, ns):
        '''
        In order to guarantee the stability or provide stability for the system, 
        MPC problem can be converted into the infinite horizon control problem.
        
        V : Transformation Matrix
        J : Diagonally Grouped Eigenvalue Matrix (Sorted regarding stability)
        ns : number of unstable modes
        where
            A = V*J/V
        '''
        
        isstable = True
        if ns == 0:
            isstable = True
            
        else:
            isstable = False
            self.terminalState = True
        if isstable == True:
            print('System is stable!')
            P  = np.transpose(self.Cd) @ self.Q @ self.Cd
            Qb = scipy.linalg.solve_discrete_lyapunov(np.transpose(self.Ad), P)
            print('rank of the Qb: ', np.linalg.matrix_rank(Qb))
            return Qb, None
        else:
            print('System is unstable!')
            Wu = V[:,0:ns] 
            Ws = V[:,ns:]
            Ju = J[0:ns,0:ns] 
            Js = J[ns:, ns:]

            Wbu = V[0:ns,:]
            Wbs = V[ns:,:]
            P   = np.transpose(Ws) @ np.transpose(self.Cd) @ self.Q @ self.Cd @ Ws
            pi  = scipy.linalg.solve_discrete_lyapunov(np.transpose(Js), P)
            Qb  = np.transpose(Wbs) @ pi @ Wbs
            print('Unstable Eigenvector: ', Wbu)
            return Qb, Wbu
        
        
    def stable_unstable_decomposition(self, Ad):
        L,  M  = scipy.linalg.eig(Ad)
        nus = 0
        for i in range(len(L)):
            if abs(L[i]) >= 0.9999:                
                if nus == 0 and i == 0:
                    pass
                else:
                    tmpL = L[nus]
                    tmpM = M[:,nus].copy()
                    
                    L[nus]   = L[i]
                    M[:,nus] = M[:,i].copy()
                    
                    L[i]   = tmpL
                    M[:,i] = tmpM.copy()
                    
                nus += 1    

        Lr, Mr = scipy.linalg.cdf2rdf(L,M)
        
        return Lr, Mr, nus
            
    
