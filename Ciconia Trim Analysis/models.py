# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 16:40:39 2022

@author: Turan Konyalioglu
"""

import numpy as np

from propulsion import pair, propulsion
from gravitational import gravitational
from aerodynamics import aerodynamics
from gyroscopic import gyroscopic


class models:
    
    def __init__(self):
        
        self.m = 30
        self.density = 1.225
        self.c = 343
        self.g = 9.8065
    
        self.Ixx = 8.638
        self.Iyy = 9.014
        self.Izz = 16.738
        self.Ixy = 0
        self.Ixz = 1.3
        self.Iyz = 0
        
        self.a = 0.635
        self.b = 0.530
        
        self.inertia_tensor = np.array([[self.Ixx,-self.Ixy,-self.Ixz],\
                                        [-self.Ixy, self.Iyy, -self.Iyz],\
                                        [-self.Ixz, -self.Iyz, self.Izz]], dtype = "float32")
            
            
        r1 = np.array([[ 0.530, 0.635, 0]])
        r2 = np.array([[-0.530,-0.635, 0]])
        r3 = np.array([[ 0.530,-0.635, 0]])
        r4 = np.array([[-0.530, 0.635, 0]])
        rF = np.array([[-0.490, 0.0, 0.0]])

        orientation = np.array([[0],[0],[1]])

        self.pair1 = pair(0, r1, orientation, rotation =  1)
        self.pair2 = pair(0, r2, orientation, rotation =  1)
        self.pair3 = pair(0, r3, orientation, rotation = -1)
        self.pair4 = pair(0, r4, orientation, rotation = -1)
        
        orientation = np.array([[1],[0],[0]])

        self.pairF = pair(1, rF, orientation, rotation = 1)
        
        self.prop_set = self.pair1, self.pair2, self.pair3, self.pair4, self.pairF
        
        self.grav = gravitational(self.m)
        self.aero = aerodynamics()
        self.prop = propulsion()
        self.gyro = gyroscopic()
        

    def calculate_propulsive_forces_and_moments_trim(self, u, mach):
        if u[4,0] >= 1:
            u[4,0] = 1
        elif u[4,0] <= 0:
            u[4,0] = 0
        
        thrust = u[0,0] / 4 / self.g * 1000
        
        throttle = self.prop.throttle_interpn1(thrust)
        
        dt = np.array([[throttle],[throttle],[throttle],[throttle],[u[4,0]]])
        
        self.set_propulsion_throttle(dt)
        self.prop.propulsive_forces_moments(mach, self.prop_set)

        Fpf, Mpf = self.prop.propulsive_forces_moments(mach, self.pairF) 
        Fpq = np.array([[0],[0],[-u[0,0]]])
        Mpq = np.array([[u[1,0]],[u[2,0]],[u[3,0]]])
        
        return Fpq + Fpf, Mpq + Mpf
        
        
    def set_propulsion_throttle(self, throttle):
        self.pair1.dt = throttle[0,0]
        self.pair2.dt = throttle[1,0]
        self.pair3.dt = throttle[2,0]
        self.pair4.dt = throttle[3,0]
        self.pairF.dt = throttle[4,0]
            
          
    def calculate_forces_moments(self, euler_angles, wind_param, rates, cont_surf, u):
        mach = wind_param[0,0] / self.c
        Fa, Ma = self.aero.aerodynamic_model(wind_param, rates, cont_surf)
        Fp, Mp = self.calculate_propulsive_forces_and_moments_trim(u, mach) 
        Mg     = self.gyro.gyroscopic_moment(rates, self.prop_set)
        Fg     = self.grav.gravitational_forces(euler_angles)
        return Fa + Fp + Fg , Ma + Mg + Mp 

    
    
    def calculate_trimmed_throttle(self, euler_angles, wind_param, rates, cont_surf, u):
        mach = wind_param[0,0] / self.c
        Fa, Ma = self.aero.aerodynamic_model(wind_param, rates, cont_surf)
        Fp, Mp = self.calculate_propulsive_forces_and_moments_trim(u, mach) 
        Mg     = self.gyro.gyroscopic_moment(rates, self.prop_set)
        Fg     = self.grav.gravitational_forces(euler_angles)
        return Fa + Fp + Fg , Ma + Mg + Mp 

