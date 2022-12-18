# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 15:28:38 2022

@author: Turan Konyalioglu
"""

import numpy as np
from utils import *



class eom:
    
    def __init__(self, m, inertia_tensor, g = 9.8065):
        
        self.m = m
        self.g = g
        self.inertia_tensor = inertia_tensor
        
        

    def eom(self, total_forces, total_moments, b_rates, euler_angles, wind_param):
        
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
        
        if not wind_param[0,0] == 0:
            Vt_dot, alpha_dot, beta_dot = body2windDot_transformation(wind_param, np.array([[u_dot],[v_dot],[w_dot]]))
        else:
            Vt_dot, alpha_dot, beta_dot = 0, 0, 0
            
        "Rotational Dynamics"
        rate_matrix = np.array([[0, -r, q],[r, 0, -p],[-q, p, 0]])
        rate_dot = np.linalg.inv(self.inertia_tensor) \
        @ (total_moments - rate_matrix @ self.inertia_tensor @ b_rates)
        
        p_dot = rate_dot[0,0]
        q_dot = rate_dot[1,0]
        r_dot = rate_dot[2,0]
        
        phi_dot, theta_dot, psi_dot = body2earth_rate_transformation(euler_angles, b_rates)
                
        return np.array([[Vt_dot],[alpha_dot],[beta_dot],[p_dot],[q_dot],[r_dot],[x_dot],[y_dot],[z_dot],[phi_dot],[theta_dot],[psi_dot]])
    
    
    def eom2(self, total_forces, total_moments, b_rates, euler_angles, wind_param):
        
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
                
        return np.array([[u_dot],[w_dot],[q_dot],[theta_dot],[-z_dot],[v_dot],[p_dot],[r_dot],[phi_dot],[psi_dot]])
    
    
    def eom3(self, total_forces, total_moments, b_rates, euler_angles, linear_vel):
        
        u = linear_vel[0,0]
        v = linear_vel[1,0]
        w = linear_vel[2,0]
        
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
        
        
        "Rotational Dynamics"
        rate_matrix = np.array([[0, -r, q],[r, 0, -p],[-q, p, 0]])
        rate_dot = np.linalg.inv(self.inertia_tensor) \
        @ (total_moments - rate_matrix @ self.inertia_tensor @ b_rates)
        
        p_dot = rate_dot[0,0]
        q_dot = rate_dot[1,0]
        r_dot = rate_dot[2,0]
        
        phi_dot, theta_dot, psi_dot = body2earth_rate_transformation(euler_angles, b_rates)
                
        return np.array([[u_dot],[w_dot],[q_dot],[theta_dot],[-z_dot],[v_dot],[p_dot],[r_dot],[phi_dot],[psi_dot]])
