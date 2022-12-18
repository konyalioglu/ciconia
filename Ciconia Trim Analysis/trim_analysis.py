# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 16:12:04 2022

@author: Turan Konyalioglu
"""

import numpy as np
import math
from utils import  *
from eom import eom
from models import models
from scipy.optimize import minimize


class trim_analysis:
    
    
    def __init__(self):
        
        self.m = 30
        self.g = 9.8065
        self.density = 1.225
        self.speedofsound = 343
        
        self.model = models()
        self.eom = eom(self.model.m, self.model.inertia_tensor)
    
    
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
        d_model = np.zeros((6, 1))
        
        print(x_model)
        return xdot_model, x_model, u_model, d_model
    
    
    def convert_steadyturn_trim_states2model_states_flight(self, x_trim, mach, phi):
        """General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
            x(1) ... alpha [rad]
            x(2) ... theta [rad]
            x(3) ... psidot [rad/s]
            x(4) ... de [deg]
            x(5) ... da [deg]
            x(6) ... dr [deg]
            x(7) ... dt [ratio]
    
        mach    : mach number
        phi     : bank angle
        lambdaf : lambda (flight path angle)
        """
        
        "Define states and control inputs"
        Vt = self.speedofsound * mach
        alpha = x_trim[0]
        beta = 0
        
        p = - x_trim[3] * np.sin(x_trim[1]);
        q =   x_trim[3] * np.cos(x_trim[1]) * np.sin(phi);
        r =   x_trim[3] * np.cos(x_trim[1]) * np.cos(phi);    
        
        theta = x_trim[1]
        psi = 0     
        de = x_trim[3]
        dt = x_trim[6]
        da = x_trim[4]
        dr = x_trim[5]
                
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
        d_model = np.zeros((6, 1))
        
        print(x_model)
        return xdot_model, x_model, u_model, d_model
    
    
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
        d_model = np.zeros((6, 1))
        
        print('derivatives of the state: ', xdot_model, '\n')
        print('states:  ', x_model, '\n')
        print('input vector:  ', u_model)
        print(x_model)
        return xdot_model, x_model, u_model, d_model


    def generalized_flight_cost_function(self, x, mach, phi=0, lambdaf=0, option = 'general'):
        """
        Parameters
        ----------
        x : Aircraft states
        mach : Mach number
        option : string, optional
            The default is 'general'.
            '"general" for general straight flight trim analysis'
            '"simplified" for general simplified straight flight trim analysis'
            '"special" for constant altitude and wing level trim analysis'

        Returns
        -------
        Float
            Returns cost values for the quadratic cost function.

        """
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

        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        
        Vt = mach * self.speedofsound
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])
        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])
        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        u_p = np.array([[0],[0],[0],[0],[x[10]]])
        
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u_p)
        
        out = self.eom.eom(Ft, Mt, body_rates, euler_angles, wind_param)
        
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
        
        if option == 'general':
            return td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2 + rk1**2 + rk2**2 + rk3**2 + tk2**2 + tk3**2
        elif option == 'simplified':        
            return td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2 + tk2**2 + tk3**2
        elif option == 'special':        
            return td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2
        elif option == 'turn':        
            return  td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2 + tk3**2


    def turn_flight_cost_function(self, x, mach, phi, lambdaf=0):
        """
        Parameters
        ----------
        x : Aircraft states
        mach : Mach number


        Returns
        -------
        Float
            Returns cost values for the quadratic cost function.

        """
        """General Straight Flight - 11 Equations and 11 Unknown
            Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
            x(1) ... alpha [rad]
            x(2) ... theta [rad]
            x(3) ... psidot [rad/s]
            x(4) ... de [deg]
            x(5) ... da [deg]
            x(6) ... dr [deg]
            x(7) ... dt [ratio]
            
            mach    : mach number
            phi     : bank angle
            lambdaf : lambda (flight path angle)
            """

        p = - x[3] * np.sin(x[1]);
        q =   x[3] * np.cos(x[1]) * np.sin(phi);
        r =   x[3] * np.cos(x[1]) * np.cos(phi);        


        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        
        Vt = mach * self.speedofsound
        wind_param   = np.array([[Vt],[x[0]],[0]])
        
        body_rates = np.array([[p],[q],[r]])
        
        cont_surf  = np.array([[x[3]],[x[4]],[x[5]]])
        
        u_p = np.array([[0],[0],[0],[0],[x[6]]])
        
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u_p)
        
        out = self.eom.eom(Ft, Mt, body_rates, euler_angles, wind_param)
        
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
             
        return  td1**2 + td2**2 + td3**2 + rd1**2 + rd2**2 + rd3**2 + tk3**2            
            
                    
    def generalized_transition_flight_cost_function(self, x, mach, phi, lambdaf):
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
        
        Vt = mach * self.speedofsound
        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])                   
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        u_p = np.array([[x[11]],[x[12]],[x[13]],[x[14]],[x[10]]])
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u_p)
        
        out = self.eom.eom(Ft, Mt, body_rates, euler_angles, wind_param)
        
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
    
    
    def optimize_generalized_straight_flight(self, mach, x0, phi=0, lambdaf=0, bnds=(), method = 'trust-constr', cons = ()):
        """
        General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt]
        """
        
        trim = minimize(self.generalized_flight_cost_function,  \
                        x0, args=(mach, phi, lambdaf, 'general'), method='trust-constr', bounds=bnds, constraints=cons, options={'maxiter': 10000, 'disp': True}, tol=1e-20)
        
        print(trim.x)
        return trim.x
    
    
    def optimize_simplified_straight_flight(self, mach, x0, phi=0, lambdaf=0, bnds=(), method = 'trust-constr', cons = ()):
        """
        General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, 0, 0, 0, theta, psi, de, da, dr, dt]
        """
        cons = self.simplified_general_straight_flight_constraints()
        
        trim = minimize(self.generalized_flight_cost_function,  \
                        x0, args=(mach, phi, lambdaf, 'simplified'), method='trust-constr', bounds=bnds, constraints=cons, options={'maxiter': 10000, 'disp': True}, tol=1e-16)
        
        print(trim.x)
        return trim.x
    
    
    def optimize_constant_altitude_straight_flight(self, mach, x0, phi=0, lambdaf=0, bnds=(), method = 'trust-constr', cons = ()):
        """
        General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, 0, 0, 0, theta, psi, de, da, dr, dt]
        """
        cons = self.constant_altitude_general_straight_flight_constraints()
        
        trim = minimize(self.generalized_flight_cost_function,  \
                        x0, args=(mach, phi, lambdaf, 'special'), method='trust-constr', bounds=bnds, constraints=cons, options={'maxiter': 10000, 'disp': True}, tol=1e-16)
        
        print(trim.x)
        return trim.x
        
    
    def optimize_steady_state_turn(self, mach, x0,  phi=45, lambdaf=0, bnds=(), method = 'trust-constr', cons = ()):
        """
        General Straight Flight - 7 Equations and 7 Unknown
        Unknown Vector = [alpha, theta, psidot, de, da, dr, dt]
        """
        
        trim = minimize(self.turn_flight_cost_function,  \
                        x0, args=(mach, phi, lambdaf), method='trust-constr', bounds=bnds, constraints=cons, options={'maxiter': 10000, 'disp': True}, tol=1e-20)
        
        print(trim.x)
        return trim.x
    
    
    def optimize_generalized_transition_flight(self, mach, x0,  phi=0, lambdaf=0, bnds=(), method = 'trust-constr', cons = ()):
        """
        General Straight Flight - 11 Equations and 11 Unknown
        Unknown Vector = [alpha, beta, p, q, r, theta, psi, de, da, dr, dt, uz, ur, up, uy]
        """
        cons = ({'type': 'eq', 'fun': self.constraint1},
                {'type': 'eq', 'fun': self.constraint2},
                {'type': 'eq', 'fun': self.constraint3},
                {'type': 'eq', 'fun': self.constraint4},
                {'type': 'eq', 'fun': self.constraint5},
                {'type': 'eq', 'fun': self.constraint6},
                {'type': 'eq', 'fun': self.constraint7})
        trim = minimize(self.generalized_transition_flight_cost_function, \
                        x0, args=(mach, phi, lambdaf), method='SLSQP', bounds=bnds, constraints=cons, options={'maxiter': 10000, 'disp': True}, tol=1e-12)
        
        print(trim.x)
        return trim.x
    
    
    
    def sgsfc_beta(self, x):
        return -x[1]
    
    def sgsfc_p(self, x):
        return -x[2]
    
    def sgsfc_q(self, x):
        return -x[3]
    
    def sgsfc_r(self, x):
        return -x[4]
    
    def sgsfc_beta_psi(self, x):
        return x[1]-x[6]
    
    def sgsfc_p_alpha_thetha(self, x):
        return x[0]-x[5]
    
    def simplified_general_straight_flight_constraints(self):
        cons = ({'type': 'eq', 'fun': self.sgsfc_p},
                {'type': 'eq', 'fun': self.sgsfc_q},
                {'type': 'eq', 'fun': self.sgsfc_r})
        
        return cons
    
    
    def constant_altitude_general_straight_flight_constraints(self):
        cons = ({'type': 'eq', 'fun': self.sgsfc_p},
                {'type': 'eq', 'fun': self.sgsfc_q},
                {'type': 'eq', 'fun': self.sgsfc_r},
                {'type': 'eq', 'fun': self.sgsfc_beta_psi},
                {'type': 'eq', 'fun': self.sgsfc_p_alpha_thetha})
        
        return cons
    
    
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

            
    def check_trim_states(self, x, Vt, phi, lambdaf): 
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
            lambdaf : lambda (flight path angle) """
        
        euler_angles = np.array([[phi],[x[5]],[x[6]]])
        wind_param   = np.array([[Vt],[x[0]],[x[1]]])        
        body_rates = np.array([[x[2]],[x[3]],[x[4]]])        
        cont_surf  = np.array([[x[7]],[x[8]],[x[9]]])
        
        u = np.array([[0],[0],[0],[0],[x[10]]])
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u)

        print('Total Forces: \n', Ft, '\n', 'Total Moments: \n', Mt, '\n')
        return Ft, Mt
    
    
    def check_steady_turn_trim_states(self, x, Vt, phi, lambdaf=0): 
        """ x(1) ... alpha [rad]
            x(2) ... theta [rad]
            x(3) ... psidot [rad/s]
            x(4) ... de [deg]
            x(5) ... da [deg]
            x(6) ... dr [deg]
            x(7) ... dt [ratio]
            
            mach    : mach number
            phi     : bank angle
            lambdaf : lambda (flight path angle)
            """
        
        p = - x[3] * np.sin(x[1]);
        q =   x[3] * np.cos(x[1]) * np.sin(phi);
        r =   x[3] * np.cos(x[1]) * np.cos(phi);    
        
        euler_angles = np.array([[phi],[x[1]],[0]])
        wind_param   = np.array([[Vt],[x[0]],[0]])        
        body_rates = np.array([[p],[q],[r]])        
        cont_surf  = np.array([[x[3]],[x[4]],[x[5]]])
        
        u = np.array([[0],[0],[0],[0],[x[6]]])
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u)

        print('Total Forces: \n', Ft, '\n', 'Total Moments: \n', Mt, '\n')
        return Ft, Mt
    
    
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
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, u)

        print('Total Forces: \n', Ft, '\n', 'Total Moments: \n', Mt, '\n')
        return Ft, Mt