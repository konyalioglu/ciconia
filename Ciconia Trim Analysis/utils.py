# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 14:52:53 2022

@author: Turan Konyalioglu
"""

import numpy as np


def body2earth_transformation(angles, vector):
    rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]]) 
    roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]]) 
    rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
    return np.matrix.transpose(rotx @ roty @ rotz) @ vector


def earth2body_transformation(angles, vector):
    rotx = np.array([[1, 0, 0], [0, np.cos(angles[0,0]), np.sin(angles[0,0])], [0, -np.sin(angles[0,0]), np.cos(angles[0,0])]]) 
    roty = np.array([[np.cos(angles[1,0]), 0, -np.sin(angles[1,0])], [0, 1, 0], [np.sin(angles[1,0]), 0, np.cos(angles[1,0])]]) 
    rotz = np.array([[np.cos(angles[2,0]), np.sin(angles[2,0]), 0], [-np.sin(angles[2,0]), np.cos(angles[2,0]), 0], [0, 0, 1]])
    return rotx @ roty @ rotz @ vector


def body2windDot_transformation(wind_param, linear_vel):
    vec1 = np.array([[np.cos(wind_param[1,0]) * np.cos(wind_param[2,0]), \
                      np.sin(wind_param[2,0]), \
                      np.sin(wind_param[1,0]) * np.cos(wind_param[2,0])]],dtype = "float32")
    
    vec2 = np.array([[-np.cos(wind_param[1,0]) * np.sin(wind_param[2,0]) / wind_param[0,0], \
                       np.cos(wind_param[2,0]) / wind_param[0,0], \
                      -np.sin(wind_param[1,0]) * np.cos(wind_param[2,0]) / wind_param[0,0]]],dtype = "float32")
    
    vec3 = np.array([[-np.sin(wind_param[1,0]) / (np.cos(wind_param[2,0]) * wind_param[0,0]), \
                      0, \
                       np.cos(wind_param[1,0]) / (np.cos(wind_param[2,0]) / wind_param[0,0])]],dtype = "float32")
    
    transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)
        
    wind_dot = transformation_matrix @ linear_vel
        
    Vt_dot = wind_dot[0,0]
    alpha_dot = wind_dot[2,0]
    beta_dot = wind_dot[1,0]
    return Vt_dot, alpha_dot, beta_dot
    
    
def body2earth_rate_transformation(angles, rates):
    vec1 = np.array([[1, \
                      np.tan(angles[1,0]) * np.sin(angles[0,0]), \
                      np.cos(angles[0,0]) * np.tan(angles[1,0])]])
    
    vec2 = np.array([[0, \
                      np.cos(angles[0,0]), \
                     -np.sin(angles[0,0])]])
    
    vec3 = np.array([[0, \
                      1/np.cos(angles[1,0]) * np.sin(angles[0,0]), \
                      1/np.cos(angles[1,0]) * np.cos(angles[0,0])]])
    
    transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)
        
    vec = transformation_matrix @ rates
        
    phi_dot = vec[0,0]
    theta_dot = vec[1,0]
    psi_dot = vec[2,0]
        
    return phi_dot, theta_dot, psi_dot
       
        
def wind2body_transformation(wind_param, phi):
    """ The Transformation is for velocity vector in body axis
        
    Vw: Velocity Vector in Wind Frame
    Vb: Velocity Vector in Wind Frame
    """
    alpha = wind_param[1,0]
    beta  = wind_param[2,0]
    Vt    = wind_param[0,0]
        
    Vw = np.array([[Vt],[0],[0]])
    u = Vt * np.cos(alpha) * np.cos(beta)
    v = Vt * (np.sin(alpha)*np.sin(phi)*np.cos(beta)+np.sin(beta)*np.cos(phi))
    w = Vt * (np.sin(alpha)*np.cos(phi)*np.cos(beta)+np.sin(beta)*np.sin(phi))
        
    Vb = np.array([[u],[v],[w]])
        
    return Vb