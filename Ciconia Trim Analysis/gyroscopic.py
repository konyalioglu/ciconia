# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 15:13:43 2022

@author: Turan Konyalioglu
"""

import numpy as np

class gyroscopic:
    
    def __init__(self, rotation_matrix=np.eye(3)):
        
        self.P_2685_R = 26 * 0.0254 * 2
        self.P_2110_R = 21 * 0.0254 * 2
        
        self.P_2685_M = 0.041
        self.P_2110_M = 7.94 * 28.35 / 1000
        
        self.P_2685_J = 1 / 2 * self.P_2685_M * self.P_2685_R**2
        self.P_2110_J = 1 / 2 * self.P_2110_M * self.P_2110_R**2
        
        self.TBP = rotation_matrix
        
        
    def gyroscopic_moment(self, angular_rates, *args):
        Hp = 0
        for arg in args[0]:
            if arg.type == 0:
                J = self.P_2685_J
            elif arg.type == 1:
                J = self.P_2110_J
            Hp += self.TBP @ arg.orientation * J * arg.RPM * 2 * np.pi / 60 * arg.rotation
            
        return -np.cross(angular_rates.reshape((1,3)), Hp.reshape((1,3))).reshape((3,1))