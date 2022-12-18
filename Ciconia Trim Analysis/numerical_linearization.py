# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 16:12:15 2022

@author: Turan Konyalioglu
"""

import numpy as np
from utils import *
from models import models
from eom import eom



class numerical_linearization:
    
    def __init__(self, delta, tol, prec):
        self.delta = delta
        self.tol   = tol
        self.prec  = prec
        
        self.model = models()
        self.eom = eom(self.model.m, self.model.inertia_tensor)
        
        
    def implicit_function(self, xdot, x, u, d):
        return self.func(x,u,d) - xdot
    
    
    def implicit_function_transition(self, xdot, x, u, d):
        return self.func_transition(x,u,d) - xdot  


    def func(self, x, u, d):
        
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
        
        uw = d[0,0]
        vw = d[1,0]
        qw = d[2,0]
        ww = d[3,0]
        pw = d[4,0]
        rw = d[5,0]
        
        
        """Wind Axis Parameters"""
        if not u == 0:
            Vt = np.sqrt((u+uw)**2 + (v+vw)**2 + (w+ww)**2)
            alpha = np.arctan((w + uw) / (u + uw))
            beta  = np.arcsin((v + vw) / Vt)
        else:
            Vt = 0
            alpha = 0
            beta = 0
            
                
        body_rates = np.array([[p + pw],[q + qw],[r + rw]])
        euler_angles = np.array([[phi],[theta],[psi]])
        wind_param   = np.array([[Vt],[alpha],[beta]])        
        cont_surf  = np.array([[de],[da],[dr]])
        
        prop_input = np.array([[0],[0],[0],[0],[dt]])
        Ft, Mt = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, prop_input)
        
        return self.eom.eom2(Ft, Mt, body_rates, euler_angles, wind_param)
    
    
    def func_transition(self, x, u, d):
        
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
        
        uw = d[0,0]
        vw = d[1,0]
        qw = d[2,0]
        ww = d[3,0]
        pw = d[4,0]
        rw = d[5,0]
        
        
        """Wind Axis Parameters"""

        Vt = np.sqrt((u+uw)**2 + (v+vw)**2 + (w+ww)**2)
        alpha = np.arctan((w + uw) / (u + uw))
        beta  = np.arcsin((v + vw) / Vt)

                
        body_rates = np.array([[p + pw],[q + qw],[r + rw]])
        euler_angles = np.array([[phi],[theta],[psi]])
        wind_param   = np.array([[Vt],[alpha],[beta]])        
        cont_surf  = np.array([[de],[da],[dr]])
        
        prop_u = np.array([[uz],[ur],[up],[uy],[dt]])
        total_forces, total_moments = self.model.calculate_forces_moments(euler_angles, wind_param, body_rates, cont_surf, prop_u)
        
        return self.eom.eom2(total_forces, total_moments, body_rates, euler_angles, wind_param)
        
        
    def numerical_linearization(self, xdot, x, u, d, lin = 'flight'):
        ''' 
        Pass implicit function for forward flight and pass 
        implicit_function_transition() for transition flight trim analysis
            forward flight    -> implicit_function(self, xdot, x, u, d)
            transition flight -> implicit_function_transition(self, xdot, x, u, d)
        '''
        if lin == 'flight':
            func = self.implicit_function
        elif lin == 'transition':
            func = self.implicit_function_transition
        
        n = x.shape[0]
        m = u.shape[0]
        p = d.shape[0]
        
        
        """ Linearization for Matrix E """
        E = np.zeros((n,n))
        last = np.zeros((n,1))
        deltaE = self.delta
        
        for i in range(n):
            for j in range(self.prec):
                xdot_f = xdot.copy()
                xdot_f[i,0] = xdot_f[i,0] + deltaE
                F = func(xdot_f, x, u, d)
                delta_xdot_f = F.copy()
                
                xdot_b = xdot.copy()
                xdot_b[i,0] = xdot_b[i,0] - deltaE
                F = func(xdot_b, x, u, d)
                delta_xdot_b = F.copy()
                
                E[:,i] = ((delta_xdot_f - delta_xdot_b) / (2 * deltaE)).reshape((n,))
                
                if np.amax( np.absolute(E[:,i] - last) / np.absolute(E[:,i] + 1e-12)) < self.tol:
                    break
                
                deltaE = deltaE * 0.5
                last = E[:,i].copy()
                
            
            iteration = j
            if iteration == self.prec:
                print('not converged on E column' +  ': ' + str(i + 1))
                
        
        """ Linearization for Matrix Ap """
        Ap = np.zeros((n,n))
        last = np.zeros((n,1))
        deltaAp = self.delta
        
        for i in range(n):
            for j in range(self.prec):
                x_f = x.copy()
                x_f[i,0] = x_f[i,0] + deltaAp
                #print(x_f)
                F = func(xdot, x_f, u, d)
                delta_x_f = F.copy()
                
                x_b = x.copy()
                #print(x_b)
                x_b[i,0] = x_b[i,0] - deltaAp
                F = func(xdot, x_b, u, d)
                delta_x_b = F.copy()
                
                Ap[:,i] = ((delta_x_f - delta_x_b) / (2 * deltaAp)).reshape((n,))
                
                if np.amax( np.absolute(Ap[:,i] - last) / np.absolute(Ap[:,i] + 1e-12)) < self.tol:
                    break
                
                deltaAp = deltaAp * 0.5
                last = Ap[:,i].copy()
                
                
            iteration = j
            if iteration == self.prec:
                print('not converged on Ap column' +  ': ' + str(i + 1))
        
        
        """ Linearization for Matrix Bp """
        Bp = np.zeros((n,m))
        last = np.zeros((m,1))
        deltaBp = self.delta
        
        for i in range(m):
            for j in range(self.prec):
                u_f = u.copy()
                u_f[i,0] = u_f[i,0] + deltaBp
                F = func(xdot, x, u_f, d)
                delta_u_f = F.copy()
                
                u_b = u.copy()
                u_b[i,0] = u_b[i,0] - deltaBp
                F = func(xdot, x, u_b, d)
                delta_u_b = F.copy()
                
                Bp[:,i] = ((delta_u_f - delta_u_b) / (2 * deltaBp)).reshape((n,))
                
                if np.amax( np.absolute(Bp[:,i] - last) / np.absolute(Bp[:,i] + 1e-12)) < self.tol:
                    break
                
                deltaBp = deltaBp * 0.5
                last = Bp[:,i].copy()
                
                
            iteration = j
            if iteration == self.prec:
                print('not converged on B column' +  ': ' + str(i + 1))
                
                
        """ Linearization for Matrix Dp """
        Dp = np.zeros((n,p))
        last = np.zeros((p,1))
        deltaDp = self.delta
        
        for i in range(p):
            for j in range(self.prec):
                d_f = d.copy()
                d_f[i,0] = d_f[i,0] + deltaDp
                F = func(xdot, x, u, d_f)
                delta_d_f = F.copy()
                
                d_b = d.copy()
                d_b[i,0] = d_b[i,0] - deltaDp
                F = func(xdot, x, u, d_b)
                delta_d_b = F.copy()
                
                Dp[:,i] = ((delta_d_f - delta_d_b) / (2 * deltaDp)).reshape((n,))
                
                if np.amax( np.absolute(Dp[:,i] - last) / np.absolute(Dp[:,i] + 1e-12)) < self.tol:
                    break
                
                deltaDp = deltaDp * 0.5
                last = Dp[:,i].copy()
                
                
            iteration = j
            if iteration == self.prec:
                print('not converged on D column' +  ': ' + str(i + 1))
                
        return E, Ap, Bp, Dp
    

