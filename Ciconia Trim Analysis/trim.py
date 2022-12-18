# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 12:49:05 2021

@author: turan
"""
import numpy as np
from numpy import nan
import math, os, time
from scipy.io import loadmat
from scipy.interpolate import LinearNDInterpolator
from scipy.interpolate import RegularGridInterpolator
from scipy.interpolate import interpn
from scipy.optimize import minimize

from utils import *
from lqr import LinearQuadraticRegulator


class trim_analysis:
    
    
    def __init__(self):
        self.aerodynamic_data_initialization()
        print("Aerodynamic Initialization is Completed")
        
        self.thrust_coefficient = 0.0245
        self.thrust_coefficientFW = 0.102
        
        self.m = 25
        self.g = 9.8065
        self.density = 1.225
        self.speedofsound = 343

        Ixx = 8.638
        Iyy = 9.014
        Izz = 16.738
        Ixy = 0
        Ixz = 1.3
        Iyz = 0
        
        self.a = 635
        self.b = 530
        
        self.inertia_tensor = np.array([[Ixx,-Ixy,-Ixz],\
                                        [-Ixy, Iyy, -Iyz],\
                                        [-Ixz, -Iyz, Izz]], dtype = "float32")
    
        self.CG = np.array([[0],[0],[0.016389]])
        
        self.r1 = np.array([[0.530], [-0.635], [0.0305]]) - self.CG
        self.r2 = np.array([[-0.530], [0.635], [0.0305]]) - self.CG
        self.r3 = np.array([[0.530], [0.635], [0.0305]]) - self.CG
        self.r4 = np.array([[-0.530], [-0.635], [0.0305]]) - self.CG
        
    
    def func(self, x, u):
        
        "Define states and control inputs"
        
        de = u[0,0]
        dt = u[1,0]
        da = u[2,0]
        dr = u[3,0]
        
        u = x[0,0]
        w = x[1,0]
        q = x[2,0]
        theta = x[3,0]
        h = x[4,0]
        
        v = x[5,0]
        p = x[6,0]
        r = x[7,0]
        phi = x[8,0]
        psi = x[9,0]
        
        
        """Wind Axis Parameters"""
        Vt = math.sqrt(u**2 + v**2 + w**2)
        #print(Vt)
        alpha = math.atan(w / u)
        beta  = math.asin(v / Vt)
                
        body_rates = np.array([[p],[q],[r]])
        euler_angles = np.array([[phi],[theta],[psi]])
        wind_param   = np.array([[Vt],[alpha],[beta]])        
        cont_surf  = np.array([[de],[da],[dr]])
        
        prop_vel = np.array([[0],[0],[0],[0],[dt]])
        Fp, Mp = self.propulsional_model(prop_vel)
        Fg = self.gravitational_model(euler_angles)
        Fa, Ma = self.aerodynamic_model(wind_param, body_rates, cont_surf)
        
        total_forces = Fp + Fg + Fa
        total_moments = Mp + Ma
                
        """"Translational Dynamics"""
        udot = total_forces[0,0] / self.m - q * w + r * v
        vdot = total_forces[1,0] / self.m - r * u + p * w
        wdot = total_forces[2,0] / self.m - p * v + q * u
                
        """Rotational Dynamics"""
        rate_matrix = np.array([[0, -r, q],[r, 0, -p],[-q, p, 0]])
        rate_dot = np.linalg.inv(self.inertia_tensor) \
        @ (total_moments - rate_matrix @ self.inertia_tensor @ body_rates)
        
        pdot = rate_dot[0,0]
        qdot = rate_dot[1,0]
        rdot = rate_dot[2,0]
        
        phidot, thetadot, psidot = body2earth_rate_transformation(euler_angles, body_rates)
        
        vel_earth = body2earth_transformation(euler_angles, np.array([[u],[v],[w]]))
        
        xdot = vel_earth[0,0]
        ydot = vel_earth[1,0]
        zdot = vel_earth[2,0]
        
        xdot = np.array([[udot],[wdot],[qdot],[thetadot],[-zdot],[vdot],[pdot],[rdot],[phidot],[psidot]])
        return xdot
    
    
    def func_transition(self, x, u):
        
        "Define states and control inputs"
        de = u[0,0]
        dt = u[1,0]
        uz = u[2,0]
        up = u[3,0]
        da = u[4,0]
        dr = u[5,0]
        ur = u[6,0]
        uy = u[7,0]
        
        u = x[0,0]
        w = x[1,0]
        q = x[2,0]
        theta = x[3,0]
        h = x[4,0]
        
        v = x[5,0]
        p = x[6,0]
        r = x[7,0]
        phi = x[8,0]
        psi = x[9,0]
        
        """Wind Axis Parameters"""
        Vt = math.sqrt(u**2 + v**2 + w**2)
        alpha = math.atan(w / u)
        beta  = math.asin(v / Vt)
                
        body_rates = np.array([[p],[q],[r]])
        euler_angles = np.array([[phi],[theta],[psi]])
        wind_param   = np.array([[Vt],[alpha],[beta]])        
        cont_surf  = np.array([[de],[da],[dr]])
        
        prop_u = np.array([[uz],[ur],[up],[uy],[dt]])
        Fp, Mp = self.propulsional_model(prop_u)
        Fg = self.gravitational_model(euler_angles)
        Fa, Ma = self.aerodynamic_model(wind_param, body_rates, cont_surf)
        
        total_forces = Fp + Fg + Fa
        total_moments = Mp + Ma
                
        """"Translational Dynamics"""
        udot = total_forces[0,0] / self.m - q * w + r * v
        vdot = total_forces[1,0] / self.m - r * u + p * w
        wdot = total_forces[2,0] / self.m - p * v + q * u
                
        """Rotational Dynamics"""
        rate_matrix = np.array([[0, -r, q],[r, 0, -p],[-q, p, 0]])
        rate_dot = np.linalg.inv(self.inertia_tensor) \
        @ (total_moments - rate_matrix @ self.inertia_tensor @ body_rates)
        
        pdot = rate_dot[0,0]
        qdot = rate_dot[1,0]
        rdot = rate_dot[2,0]
        
        phidot, thetadot, psidot = body2earth_rate_transformation(euler_angles, body_rates)
        
        vel_earth = body2earth_transformation(euler_angles, np.array([[u],[v],[w]]))
        
        xdot = vel_earth[0,0]
        ydot = vel_earth[1,0]
        zdot = vel_earth[2,0]
        
        xdot = np.array([[udot],[wdot],[qdot],[thetadot],[-zdot],[vdot],[pdot],[rdot],[phidot],[psidot]])
        return xdot
    
    
    def implicit_function(self, xdot, x, u):
        return self.func(x,u) - xdot
    
    
    def implicit_function_transition(self, xdot, x, u):
        return self.func_transition(x,u) - xdot
        
    
    def numerical_linearization(self, xdot, x, u, delta, tol, prec):
        ''' 
        Pass implicit function for forward flight and pass 
        implicit_function_transition() for transition flight trim analysis
            forward flight    -> implicit_function(self, xdot, x, u)
            transition flight -> implicit_function_transition(self, xdot, x, u)
        '''
        
        n = x.shape[0]
        m = u.shape[0]
        
        
        """ Linearization for Matrix E """
        E = np.zeros((n,n))
        last = np.zeros((n,1))
        deltaE = delta
        
        for i in range(n):
            for j in range(prec):
                xdot_f = xdot.copy()
                xdot_f[i,0] = xdot_f[i,0] + deltaE
                F = self.implicit_function(xdot_f, x, u)
                delta_xdot_f = F.copy()
                
                xdot_b = xdot.copy()
                xdot_b[i,0] = xdot_b[i,0] - deltaE
                F = self.implicit_function(xdot_b, x, u)
                delta_xdot_b = F.copy()
                
                E[:,i] = ((delta_xdot_f - delta_xdot_b) / (2 * deltaE)).reshape((n,))
                
                if np.amax( np.absolute(E[:,i] - last) / np.absolute(E[:,i] + 1e-12)) < tol:
                    break
                
                deltaE = deltaE * 0.5
                last = E[:,i].copy()
                
            
            iteration = j
            if iteration == prec:
                print('not converged on E column' +  ': ' + str(i + 1))
                
        
        """ Linearization for Matrix A """
        Ap = np.zeros((n,n))
        last = np.zeros((n,1))
        deltaAp = delta
        
        for i in range(n):
            for j in range(prec):
                x_f = x.copy()
                x_f[i,0] = x_f[i,0] + deltaAp
                #print(x_f)
                F = self.implicit_function(xdot, x_f, u)
                delta_x_f = F.copy()
                
                x_b = x.copy()
                #print(x_b)
                x_b[i,0] = x_b[i,0] - deltaAp
                F = self.implicit_function(xdot, x_b, u)
                delta_x_b = F.copy()
                
                Ap[:,i] = ((delta_x_f - delta_x_b) / (2 * deltaAp)).reshape((n,))
                
                if np.amax( np.absolute(Ap[:,i] - last) / np.absolute(Ap[:,i] + 1e-12)) < tol:
                    break
                
                deltaAp = deltaAp * 0.5
                last = Ap[:,i].copy()
                
                
            iteration = j
            if iteration == prec:
                print('not converged on Ap column' +  ': ' + str(i + 1))
        
        
        """ Linearization for Matrix Bp """
        Bp = np.zeros((n,m))
        last = np.zeros((m,1))
        deltaBp = delta
        
        for i in range(m):
            for j in range(prec):
                u_f = u.copy()
                u_f[i,0] = u_f[i,0] + deltaBp
                F = self.implicit_function(xdot, x, u_f)
                delta_u_f = F.copy()
                
                u_b = u.copy()
                u_b[i,0] = u_b[i,0] - deltaBp
                F = self.implicit_function(xdot, x, u_b)
                delta_u_b = F.copy()
                
                Bp[:,i] = ((delta_u_f - delta_u_b) / (2 * deltaBp)).reshape((n,))
                
                if np.amax( np.absolute(Bp[:,i] - last) / np.absolute(Bp[:,i] + 1e-12)) < tol:
                    break
                
                deltaBp = deltaBp * 0.5
                last = Bp[:,i].copy()
                
                
            iteration = j
            if iteration == prec:
                print('not converged on B column' +  ': ' + str(i + 1))
                
        return E, Ap, Bp
    
    
    def numerical_linearization_transition(self, xdot, x, u, delta, tol, prec):
        n = x.shape[0]
        m = u.shape[0]
        
        
        """ Linearization for Matrix E """
        E = np.zeros((n,n))
        last = np.zeros((n,1))
        deltaE = delta
        
        for i in range(n):
            for j in range(prec):
                xdot_f = xdot.copy()
                xdot_f[i,0] = xdot_f[i,0] + deltaE
                F = self.implicit_function_transition(xdot_f, x, u)
                delta_xdot_f = F.copy()
                
                xdot_b = xdot.copy()
                xdot_b[i,0] = xdot_b[i,0] - deltaE
                F = self.implicit_function_transition(xdot_b, x, u)
                delta_xdot_b = F.copy()
                
                E[:,i] = ((delta_xdot_f - delta_xdot_b) / (2 * deltaE)).reshape((n,))
                
                if np.amax( np.absolute(E[:,i] - last) / np.absolute(E[:,i] + 1e-12)) < tol:
                    break
                
                deltaE = deltaE * 0.5
                last = E[:,i].copy()
                
            
            iteration = j
            if iteration == prec:
                print('not converged on E column' +  ': ' + str(i + 1))
                
        
        """ Linearization for Matrix A """
        Ap = np.zeros((n,n))
        last = np.zeros((n,1))
        deltaAp = delta
        
        for i in range(n):
            for j in range(prec):
                x_f = x.copy()
                x_f[i,0] = x_f[i,0] + deltaAp
                #print(x_f)
                F = self.implicit_function_transition(xdot, x_f, u)
                delta_x_f = F.copy()
                
                x_b = x.copy()
                #print(x_b)
                x_b[i,0] = x_b[i,0] - deltaAp
                F = self.implicit_function_transition(xdot, x_b, u)
                delta_x_b = F.copy()
                
                Ap[:,i] = ((delta_x_f - delta_x_b) / (2 * deltaAp)).reshape((n,))
                
                if np.amax( np.absolute(Ap[:,i] - last) / np.absolute(Ap[:,i] + 1e-12)) < tol:
                    break
                
                deltaAp = deltaAp * 0.5
                last = Ap[:,i].copy()
                
                
            iteration = j
            if iteration == prec:
                print('not converged on Ap column' +  ': ' + str(i + 1))
        
        
        """ Linearization for Matrix Bp """
        Bp = np.zeros((n,m))
        last = np.zeros((m,1))
        deltaBp = delta
        
        for i in range(m):
            for j in range(prec):
                u_f = u.copy()
                u_f[i,0] = u_f[i,0] + deltaBp
                F = self.implicit_function_transition(xdot, x, u_f)
                delta_u_f = F.copy()
                
                u_b = u.copy()
                u_b[i,0] = u_b[i,0] - deltaBp
                F = self.implicit_function_transition(xdot, x, u_b)
                delta_u_b = F.copy()
                
                Bp[:,i] = ((delta_u_f - delta_u_b) / (2 * deltaBp)).reshape((n,))
                
                if np.amax( np.absolute(Bp[:,i] - last) / np.absolute(Bp[:,i] + 1e-12)) < tol:
                    break
                
                deltaBp = deltaBp * 0.5
                last = Bp[:,i].copy()
                
                
            iteration = j
            if iteration == prec:
                print('not converged on B column' +  ': ' + str(i + 1))
                
        return E, Ap, Bp
    
    
    def convert_trim_states2model_states_flight(self, x_trim, mach, phi):
        """General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
        x(1) ... alpha [rad]
        x(2) ... beta [rad]
        x(3) ... p [rad/s]
        x(4) ... q [rad/s]
        x(5) ... r [rad/s]
        x(6) ... theta [rad]
        x(7) ... psi [rad]
        x(8) ... de [deg]
        x(9) ... da [deg]
        x(10) ... dr [deg]
        x(11) ... dt [m/s2]
        
        mach    : mach number
        phi     : bank angle
        lambdaf : lambda (flight path angle)
        """
        
        "Define states and control inputs"
        Vt = self.speedofsound * mach
        alpha = x_trim[0]
        beta = x_trim[1]
        p = x_trim[2]
        q = x_trim[3]
        r = x_trim[4]
        theta = x_trim[5]
        psi = x_trim[6]        
        de = x_trim[7]
        dt = x_trim[10]
        da = x_trim[8]
        dr = x_trim[9]
                
        b_rates = np.array([[p],[q],[r]])
        euler_angles = np.array([[phi],[theta],[psi]])
        wind_param   = np.array([[Vt],[alpha],[beta]])        
        cont_surf  = np.array([[de],[da],[dr]])
        
        body_vel = wind2body_transformation(wind_param, phi)
        u = body_vel[0,0]
        v = body_vel[1,0]
        w = body_vel[2,0]
        h = 0
        
        x_model = np.array([[u],[w],[q],[theta],[h],[v],[p],[r],[phi],[psi]])
        xdot_model = np.zeros((x_model.shape[0],x_model.shape[1]))
        u_model = np.array([[de],[dt],[da],[dr]])
        
        #print('derivatives of the state: ', xdot_model, '\n')
        #print('states:  ', x_model, '\n')
        #print('input vector:  ', u_model)
        print(x_model)
        return xdot_model, x_model, u_model
    
    
    def convert_trim_states2model_states_transition(self, x_trim, mach, phi):
        """General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
        x(1) ... alpha [rad]
        x(2) ... beta [rad]
        x(3) ... p [rad/s]
        x(4) ... q [rad/s]
        x(5) ... r [rad/s]
        x(6) ... theta [rad]
        x(7) ... psi [rad]
        x(8) ... de [deg]
        x(9) ... da [deg]
        x(10) ... dr [deg]
        x(11) ... dt [m/s2]
        
        mach    : mach number
        phi     : bank angle
        lambdaf : lambda (flight path angle)
        """
        
        "Define states and control inputs"
        Vt = self.speedofsound * mach
        alpha = x_trim[0]
        beta = x_trim[1]
        p = x_trim[2]
        q = x_trim[3]
        r = x_trim[4]
        theta = x_trim[5]
        psi = x_trim[6]        
        de = x_trim[7]
        dt = x_trim[10]
        uz = x_trim[11]
        up = x_trim[13]
        da = x_trim[8]
        dr = x_trim[9]
        ur = x_trim[12]
        uy = x_trim[14]
                
        b_rates = np.array([[p],[q],[r]])
        euler_angles = np.array([[phi],[theta],[psi]])
        wind_param   = np.array([[Vt],[alpha],[beta]])        
        cont_surf  = np.array([[de],[da],[dr]])
        
        body_vel = wind2body_transformation(wind_param, phi)
        u = body_vel[0,0]
        v = body_vel[1,0]
        w = body_vel[2,0]
        h = 0
        
        x_model = np.array([[u],[w],[q],[theta],[h],[v],[p],[r],[phi],[psi]])
        xdot_model = np.zeros((x_model.shape[0],x_model.shape[1]))
        u_model = np.array([[de],[dt],[uz],[up],[da],[dr],[ur],[uy]])
        
        print('derivatives of the state: ', xdot_model, '\n')
        print('states:  ', x_model, '\n')
        print('input vector:  ', u_model)
        print(x_model)
        return xdot_model, x_model, u_model
    

    def trim_ff(self):
        return 0
    
        
    def aerodynamic_data_initialization(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        aero_data = loadmat(dir_path + '/aerodynamic_database.mat')
        
        "Aerodynamic Parameters"
        self.sref = float(aero_data['data']['sref'][0])
        self.cbar = float(aero_data['data']['cbar'][0])
        self.bref = float(aero_data['data']['bref'][0])
		

        "Breakpoints"
        alpha = np.array(aero_data['data']['alpha'][0][0][0], dtype='float32') * np.pi / 180
        self.alpha_min = alpha[0]
        self.alpha_max = alpha[-1]
        nalpha = float(aero_data['data']['nalpha'][0])

        beta = np.array(aero_data['data']['beta'][0][0][0], dtype='float32') * np.pi / 180
        self.beta_min = beta[0]
        self.beta_max = beta[-1]
        nbeta = float(aero_data['data']['nbeta'][0])
		
        mach = np.array(aero_data['data']['mach'][0][0][0], dtype='float32')
        nmach = float(aero_data['data']['nmach'][0])
        self.mach_min = mach[0]
        self.mach_max = mach[-1]
		
        "Drag Force Coefficients"
        CD = np.array(aero_data['data']['CD'][0][0], dtype='float32')
		
        self.CD_interpn = RegularGridInterpolator((alpha, beta, mach), CD, method='linear', bounds_error=False, fill_value=nan)

        "Lift Force Coefficients"
        CL = np.array(aero_data['data']['CL'][0][0], dtype='float32')
        self.CLq = float(aero_data['data']['CLq'][0][0][0])
        self.CLde = -float(aero_data['data']['Czde'][0][0][0])

        self.CL_interpn = RegularGridInterpolator((alpha,beta, mach), CL, method='linear', bounds_error=False, fill_value=nan)
		#self.clq_interpn = RegularGridInterpolator((alpha,mach), clq, method='linear', bounds_error=False, fill_value=nan)

        "Force Coefficient on y-axis"
        Cy = np.array(aero_data['data']['Cy'][0][0], dtype='float32')
		
        self.Cyp = float(aero_data['data']['Cyp'][0][0][0])
        self.Cyr = float(aero_data['data']['Cyr'][0][0][0])
        self.Cydr = float(aero_data['data']['Cydr'][0][0][0])
		
        self.Cy_interpn = RegularGridInterpolator((alpha, beta, mach), Cy, method='linear', bounds_error=False, fill_value=nan)
				
        "Rolling Moment Coefficients"
        Cl = np.array(aero_data['data']['Cl'][0][0], dtype='float32')

        self.Clp = float(aero_data['data']['Clp'][0][0][0])
        self.Clr = float(aero_data['data']['Clr'][0][0][0])
        self.Clda = float(aero_data['data']['Clda'][0][0][0])
        self.Cldr = float(aero_data['data']['Cldr'][0][0][0])
        self.Cl_interpn = RegularGridInterpolator((alpha, beta, mach), Cl, method='linear', bounds_error=False, fill_value=nan)

        "Pitching Moment Coefficients"
        Cm = np.array(aero_data['data']['Cm'][0][0], dtype='float32')
        self.Cmq = float(aero_data['data']['Cmq'][0][0][0])
        self.Cmde = float(aero_data['data']['Cmde'][0][0][0])
        self.Cm_interpn = RegularGridInterpolator((alpha, beta, mach), Cm, method='linear', bounds_error=False, fill_value=nan)		
		
        "Yawing Moment Coefficients"
        Cn = np.array(aero_data['data']['Cn'][0][0], dtype='float32')
        self.Cnp = float(aero_data['data']['Cnp'][0][0][0])
        self.Cnr = float(aero_data['data']['Cnr'][0][0][0])
        self.Cnda = float(aero_data['data']['Cnda'][0][0][0])
        self.Cndr = float(aero_data['data']['Cndr'][0][0][0])
        self.Cn_interpn = RegularGridInterpolator((alpha, beta, mach), Cn, method='linear', bounds_error=False, fill_value=nan)
        
        return 0
    
    
    def aerodynamic_model(self, wind_param, rates_body, cont_surf):
		
        Vt = wind_param[0,0]
        alpha = wind_param[1,0]
        beta = wind_param[2,0]
        
        p = rates_body[0,0]
        q = rates_body[1,0]
        r = rates_body[2,0]
        
        delev = cont_surf[0,0]
        dail = cont_surf[1,0]
        drud = cont_surf[2,0]
        
        mach = Vt / self.speedofsound
        
        if alpha > self.alpha_max:
            alpha = self.alpha_max
            #return np.array([[1000],[1000],[1000]]), np.array([[1000],[1000],[1000]])
        elif alpha < self.alpha_min:
            alpha = self.alpha_min
            #return np.array([[1000],[1000],[1000]]), np.array([[1000],[1000],[1000]])
        if beta > self.beta_max:
            beta = self.beta_max
        elif beta < self.beta_min:
            beta = self.beta_min

        if mach > self.mach_max:
            mach = self.mach_max
        elif mach < self.mach_min:
            mach = self.mach_min
        
        CD = self.CD_interpn((alpha, beta, mach))
        CL = self.CL_interpn((alpha, beta, mach))
        Cy = self.Cy_interpn((alpha, beta, mach))
        Cl = self.Cl_interpn((alpha, beta, mach))
        Cm = self.Cm_interpn((alpha, beta, mach))
        Cn = self.Cn_interpn((alpha, beta, mach))
        
        dyn_press = Vt**2 * self.density / 2
        
        Lift = (CL + q * self.cbar / (2 * Vt) * self.CLq + self.CLde * delev) * dyn_press * self.sref
        Drag = CD * dyn_press * self.sref
        
        X = -Drag * np.cos(alpha) + Lift * np.sin(alpha)
        Z = -Drag * np.sin(alpha) - Lift * np.cos(alpha)
        Y = (Cy + self.bref * p / (2 * Vt) * self.Cyp + self.bref * r / (2 * Vt) * self.Cyr + self.Cydr * drud) * dyn_press * self.sref
        L = (Cl + self.bref * p / (2 * Vt) * self.Clp + self.bref * r / (2 * Vt) * self.Clr + self.Clda * dail + self.Cldr * drud) * self.sref * self.bref * dyn_press
        M = (Cm + self.Cmde * delev * self.cbar + self.cbar * q / (2 * Vt) * self.Cmq) * self.sref * self.cbar * dyn_press
        N = (Cn + self.bref * p / (2 * Vt) * self.Cnp + self.bref * r / (2 * Vt) * self.Cnr + self.Cnda * dail + self.Cndr * drud) * self.sref * self.bref * dyn_press

        return np.array([[X],[Y],[Z]]), np.array([[L],[M],[N]])
    
    
    def gravitational_model(self, euler_angles):
        "Euler Angles must have the format of [phi, theta, psi]'"
        Fg1 = -self.m * self.g * np.sin(euler_angles[1,0])
        Fg2 =  self.m * self.g * np.cos(euler_angles[1,0]) * np.sin(euler_angles[0,0])
        Fg3 =  self.m * self.g * np.cos(euler_angles[1,0]) * np.cos(euler_angles[0,0])
        
        return np.array([[Fg1],[Fg2],[Fg3]])
    
    
    def propulsional_model(self, u):
        ''' u = [u1 u2 u3 u4 ut]
            where
            u1 = force in z-axis
            u2 = force that creates rolling moment
            u3 = force that creates pitching moment
            u4 = yawing moment
            ut = force in x-axis
        '''
        
        '''Force1 = self.thrust_coefficient * prop_vel[0,0]**2
        Force2 = self.thrust_coefficient * prop_vel[1,0]**2
        Force3 = self.thrust_coefficient * prop_vel[2,0]**2
        Force4 = self.thrust_coefficient * prop_vel[3,0]**2
        Force5 = self.thrust_coefficientFW * prop_vel[4,0]**2
                
        prop1_force_body = np.array([[0],[0],[Force1]])
        prop2_force_body = np.array([[0],[0],[Force2]])
        prop3_force_body = np.array([[0],[0],[Force3]])
        prop4_force_body = np.array([[0],[0],[Force4]])
        prop5_force_body = np.array([[Force5],[0],[0]])
        
        prop1_moment_body = np.cross(self.r1.reshape((1,3)), prop1_force_body.reshape((1,3))).reshape((3,1))
        prop2_moment_body = np.cross(self.r2.reshape((1,3)), prop2_force_body.reshape((1,3))).reshape((3,1))
        prop3_moment_body = np.cross(self.r3.reshape((1,3)), prop3_force_body.reshape((1,3))).reshape((3,1))
        prop4_moment_body = np.cross(self.r4.reshape((1,3)), prop4_force_body.reshape((1,3))).reshape((3,1))
        
        prop_force_body = prop1_force_body + prop2_force_body + prop3_force_body + prop4_force_body + prop5_force_body
        prop_moment_body = prop1_moment_body + prop2_moment_body + prop3_moment_body + prop4_moment_body'''
        if u[4,0] <= 0:
            u[4,0] = 0
        
        prop_force_body = np.array([[u[4,0]],[0],[-u[0,0]]])
        prop_moment_body = np.array([[u[1,0]],[u[2,0]],[u[3,0]]])
		
        return prop_force_body, prop_moment_body
    
    
    def equation_of_motion(self, total_forces, total_moments, b_rates, euler_angles, wind_param):
        
        u = wind_param[0,0] * np.cos(wind_param[1,0]) * np.cos(wind_param[2,0])
        v = wind_param[0,0] * np.sin(wind_param[2,0])
        w = wind_param[0,0] * np.sin(wind_param[1,0]) * np.cos(wind_param[2,0])
        
        p = b_rates[0,0]
        q = b_rates[1,0]
        r = b_rates[2,0]
        
        "Translational Dynamics"
        u_dot = total_forces[0,0] / self.m - q * w + r * v
        v_dot = total_forces[1,0] / self.m - r * u + p * w
        w_dot = total_forces[2,0] / self.m - p * v + q * u
        
        vel_earth = body2earth_transformation(euler_angles, np.array([[u],[v],[w]]))
        
        x_dot = vel_earth[0,0]
        y_dot = vel_earth[1,0]
        z_dot = vel_earth[2,0]
        
        Vt_dot, alpha_dot, beta_dot = body2windDot_transformation(wind_param, np.array([[u_dot],[v_dot],[w_dot]]))
        
        "Rotational Dynamics"
        rate_matrix = np.array([[0, -r, q],[r, 0, -p],[-q, p, 0]])
        rate_dot = np.linalg.inv(self.inertia_tensor) \
        @ (total_moments - rate_matrix @ self.inertia_tensor @ b_rates)
        
        p_dot = rate_dot[0,0]
        q_dot = rate_dot[1,0]
        r_dot = rate_dot[2,0]
        
        phi_dot, theta_dot, psi_dot = body2earth_rate_transformation(euler_angles, b_rates)
                
        return np.array([[Vt_dot],[alpha_dot],[beta_dot],[p_dot],[q_dot],[r_dot],[x_dot],[y_dot],[z_dot],[phi_dot],[theta_dot],[psi_dot]])
    
    
    def generalized_straight_flight_cost_function(self, x):
        """General Straight Flight - 11 Equations and 11 Unknown
            Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
            x(1) ... alpha [rad]
            x(2) ... beta [rad]
            x(3) ... p [rad/s]
            x(4) ... q [rad/s]
            x(5) ... r [rad/s]
            x(6) ... theta [rad]
            x(7) ... psi [rad]
            x(8) ... de [deg]
            x(9) ... da [deg]
            x(10) ... dr [deg]
            x(11) ... dt [ratio]
            
            mach    : mach number
            phi     : bank angle
            lambdaf : lambda (flight path angle)
            """
        mach    = 18 / self.speedofsound
        phi     = 0
        lambdaf = 0
        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        
        Vt = mach * self.speedofsound
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])
        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])
        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        prop_vel = np.array([[0],[0],[0],[0],[x[10]]])
        Fp, Mp = self.propulsional_model(prop_vel)
        Fg = self.gravitational_model(euler_angles)
        Fa, Ma = self.aerodynamic_model(wind_param, body_rates, cont_surf)
        
        total_forces = Fp + Fg + Fa
        total_moments = Mp + Ma
        
        out = self.equation_of_motion(total_forces, total_moments, body_rates, euler_angles, wind_param)
        
        """
        out = [[Vt_dot],[alpha_dot],[beta_dot],[p_dot],[q_dot],[r_dot],
              [x_dot],[y_dot],[z_dot],[phi_dot],[theta_dot],[psi_dot]]
        """
        
        td1 = out[0,0]
        td2 = out[1,0]
        td3 = out[2,0]
        
        rd1 = out[3,0]
        rd2 = out[4,0]
        rd3 = out[5,0]
        
        tk2 = out[7,0]
        tk3 = np.sin(lambdaf) - (np.cos(x[0])*np.cos(x[1])*np.sin(x[5]) \
                     -np.sin(x[1])*np.cos(x[5])*np.sin(phi) \
                     -np.sin(x[0])*np.cos(x[1])*np.cos(x[5])*np.cos(phi)) 
        
        rk1 = out[9,0]
        rk2 = out[10,0]
        rk3 = out[11,0]
        
        return td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2 + tk2**2 \
                    + tk3**2 + rk1**2 + rk2**2 + rk3**2
                    
                    
    def generalized_transition_flight_cost_function(self, x):
        """General Straight Flight - 11 Equations and 11 Unknown
            Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt, uz, ur, up, uy]
            x(1) ... alpha [rad]
            x(2) ... beta [rad]
            x(3) ... p [rad/s]
            x(4) ... q [rad/s]
            x(5) ... r [rad/s]
            x(6) ... theta [rad]
            x(7) ... psi [rad]
            x(8) ... de [deg]
            x(9) ... da [deg]
            x(10) ... dr [deg]
            x(11) ... dt [ratio]
            
            mach    : mach number
            phi     : bank angle
            lambdaf : lambda (flight path angle)
            """
        mach    = 10 / self.speedofsound
        phi     = 0
        lambdaf = 0
        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        
        Vt = mach * self.speedofsound
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])
        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])
        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        prop_u = np.array([[x[11]],[x[12]],[x[13]],[x[14]],[x[10]]])
        Fp, Mp = self.propulsional_model(prop_u)
        Fg = self.gravitational_model(euler_angles)
        Fa, Ma = self.aerodynamic_model(wind_param, body_rates, cont_surf)
        
        total_forces = Fp + Fg + Fa
        total_moments = Mp + Ma
        
        out = self.equation_of_motion(total_forces, total_moments, body_rates, euler_angles, wind_param)
        
        """
        out = [[Vt_dot],[alpha_dot],[beta_dot],[p_dot],[q_dot],[r_dot],
              [x_dot],[y_dot],[z_dot],[phi_dot],[theta_dot],[psi_dot]]
        """
        
        td1 = out[0,0]
        td2 = out[1,0]
        td3 = out[2,0]
        
        rd1 = out[3,0]
        rd2 = out[4,0]
        rd3 = out[5,0]
        
        tk2 = out[7,0]
        tk3 = np.sin(lambdaf) - (np.cos(x[0])*np.cos(x[1])*np.sin(x[5]) \
                     -np.sin(x[1])*np.cos(x[5])*np.sin(phi) \
                     -np.sin(x[0])*np.cos(x[1])*np.cos(x[5])*np.cos(phi)) 
        
        rk1 = out[9,0]
        rk2 = out[10,0]
        rk3 = out[11,0]
        
        return td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2 + tk2**2 \
                    + tk3**2 + rk1**2 + rk2**2 + rk3**2
    
    
    def optimize_generalized_straight_flight(self):
        """
        General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
        """
        
        x0 = [0.01, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 1]
        bnds = ((-0.13, 0.13), (-0.13, 0.13), (None, None), (None, None), (None, None), (None, None), (None, None), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5), (0, None))
        
        trim = minimize(self.generalized_straight_flight_cost_function, \
                        x0, method='Powell', options={'maxiter': 10000, 'disp': True}, tol=1e-10)
        
        print(trim.x)
        return trim.x
    
    
    def constraint1(self, x):
        return -x[0]
    
    def constraint2(self, x):
        return -x[5]
    
    def constraint3(self, x):
        return -x[12]
    
    def constraint4(self, x):
        return -x[14]
    
    def constraint5(self, x):
        return -x[7]
    
    def constraint6(self, x):
        return -x[8]
    
    def constraint7(self, x):
        return -x[9]
    
    
    def optimize_generalized_transition_flight(self):
        """
        General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt, uz, ur, up, uy]
        """
        
        x0 = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 200, 0, 0, 0]
        bnds = ((-0.13, 0.13), (-0.13, 0.13), (None, None), (None, None), (None, None), (None, None), (None, None), (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5), (0, None),(None, None),(None, None),(None, None),(None, None))
        cons = ({'type': 'eq', 'fun': self.constraint1},
                {'type': 'eq', 'fun': self.constraint2},
                {'type': 'eq', 'fun': self.constraint3},
                {'type': 'eq', 'fun': self.constraint4},
                {'type': 'eq', 'fun': self.constraint5},
                {'type': 'eq', 'fun': self.constraint6},
                {'type': 'eq', 'fun': self.constraint7})
        trim = minimize(self.generalized_transition_flight_cost_function, \
                        x0, method='SLSQP', bounds=bnds, constraints=cons, options={'maxiter': 10000, 'disp': True}, tol=1e-15)
        
        print(trim.x)
        return trim.x
    
    
    def check_trim_states(self, x):
        mach    = 18.0 / self.speedofsound
        phi     = 0
        lambdaf = 0
        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        
        Vt = mach * self.speedofsound
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])
        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])
        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        u = np.array([[0],[0],[0],[0],[x[10]]])
        Fp, Mp = self.propulsional_model(u)
        Fg = self.gravitational_model(euler_angles)
        Fa, Ma = self.aerodynamic_model(wind_param, body_rates, cont_surf)
        print('Propulsive Forces: \n', Fp, '\n', 'Aerodynamic Forces: \n', Fa, '\n', 'Gravitational Forces: \n', Fg, '\n')
        total_forces = Fp + Fg + Fa
        total_moments = Mp + Ma
        print('Total Forces: \n', total_forces, '\n', 'Total Moments: \n', total_moments, '\n')
        return
    
    
    def check_trim_states_transition(self, x, Vt):
        mach    = Vt / self.speedofsound
        phi     = 0
        lambdaf = 0
        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        
        Vt = mach * self.speedofsound
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])
        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])
        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        u = np.array([[x[11]],[x[12]],[x[13]],[x[14]],[x[10]]])
        Fp, Mp = self.propulsional_model(u)
        Fg = self.gravitational_model(euler_angles)
        Fa, Ma = self.aerodynamic_model(wind_param, body_rates, cont_surf)
        
        total_forces = Fp + Fg + Fa
        total_moments = Mp + Ma
        
        print('Propulsive Forces: \n', Fp, '\n', 'Aerodynamic Forces: \n', Fa, '\n', 'Gravitational Forces: \n', Fg, '\n')
        print('Propulsive Moments: \n', Mp, '\n', 'Aerodynamic Moments: \n', Ma)
        print('Total Forces: \n', total_forces, '\n', 'Total Moments: \n', total_moments, '\n')
        return
    
    
    def steady_state_turn(self):
        return
    
    
    def complete_flight_analysis(self):
        ts = self.optimize_generalized_straight_flight()
        trim.check_trim_states(ts)
        return
    
    
    def complete_transition_analysis(self, Vt):
        xtrim = self.optimize_generalized_transition_flight()
        self.check_trim_states_transition(xtrim, Vt)
        mach = Vt/343
        xdot, x, u = self.convert_trim_states2model_states_transition(xtrim, mach, 0)
        
        delta = 0.1
        tol = 1e-6
        precision = 17
        E, Ap, Bp = self.numerical_linearization_transition(xdot, x, u, delta, tol, precision)
        A = -np.linalg.inv(E) @ Ap
        B = -np.linalg.inv(E) @ Bp
        w, v = np.linalg.eig(A)
        
        return A, B
    
    
    def transition_linearization(self, xdot, x, u):
        delta = 0.1
        tol = 1e-6
        precision = 17
        E, Ap, Bp = self.numerical_linearization_transition(xdot, x, u, delta, tol, precision)
        A = -np.linalg.inv(E) @ Ap
        B = -np.linalg.inv(E) @ Bp
        w, v = np.linalg.eig(A)
        
        return A, B


if __name__ == '__main__':
    

    # trim = trim_analysis()
    # ts = trim.optimize_generalized_straight_flight()
    # trim.check_trim_states(ts)
    
    # xdot, x, u = trim.convert_trim_states2model_states_flight(ts, 18/343, 0)
    # #print(xdot.shape)
    # delta = 0.01
    # tol = 1e-8
    # precision = 17
    # E, Ap, Bp = trim.numerical_linearization(xdot, x, u, delta, tol, precision)
    # A = -np.linalg.inv(E) @ Ap
    # B = -np.linalg.inv(E) @ Bp
    # C = np.eye(A.shape[0])
    # w, v = np.linalg.eig(A)
    # print(w)
    
    # lqr = LinearQuadraticRegulator(A, B, C)
    # Q = np.diag(np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1]))
    # R = np.diag(np.array([1, 1, 1, 1]))
    # K, P, eigVals = lqr.lqr(Q,R)  

    trim = trim_analysis()
    
    Vt = 10
    wind_param   = np.array([[Vt],[0.0],[0]])     
    body_rates = np.array([[0],[0],[0]])       
    cont_surf  = np.array([[0],[0],[0]])
    euler_angles = np.array([[0],[0],[0]])
    
    Fa, Ma = trim.aerodynamic_model(wind_param, body_rates, cont_surf)
    Ga = trim.gravitational_model(euler_angles)
    FTotal = Fa+Ga
    print(-(Fa+Ga))
    print(Ma)
    
    x = np.array([[Vt],[0],[0],[0],[0],[0],[0],[0],[0],[0]])
    xdot = np.zeros((x.shape[0],x.shape[1]))
    u = np.array([[0],[-FTotal[0,0]],[-FTotal[2,0]],[-Ma[1,0]],[0],[0],[0],[0]])
    A, B = trim.transition_linearization(xdot, x, u)
    A, B = trim.complete_transition_analysis(10)
    
