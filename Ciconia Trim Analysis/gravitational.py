# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 15:57:28 2022

@author: Turan Konyalioglu
"""

import numpy as np


class gravitational:
    
    def __init__(self, m, g = 9.8065, r = None):
        self.m = m
        self.g = g
        self.r = r
        
        
    def gravitational_forces(self, euler_angles):
        Fg1 = -self.m * self.g * np.sin(euler_angles[1,0])
        Fg2 =  self.m * self.g * np.cos(euler_angles[1,0]) * np.sin(euler_angles[0,0])
        Fg3 =  self.m * self.g * np.cos(euler_angles[1,0]) * np.cos(euler_angles[0,0])
        
        return np.array([[Fg1],[Fg2],[Fg3]])
    
    
    def gravitational_moments(self, euler_angles):
        if self.r == None:
            assert False, "Position vector hasn't selected!"
        else:
            Fg = self.gravitation_forces(euler_angles)
            Mg = np.cross(self.r.reshape((1,3)), Fg.reshape((1,3))).reshape((3,1))
            return Mg

        
        
        