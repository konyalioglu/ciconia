# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 15:26:43 2022

@author: Turan Konyalioglu
"""

import os

from scipy.io import loadmat
from scipy.interpolate import RegularGridInterpolator

import numpy as np
from numpy import nan



class aerodynamics:
    
    def __init__(self):
        

        dir_path = os.path.dirname(os.path.realpath(__file__))
        aero_data = loadmat(dir_path + '/aerodynamic_database.mat')
        #aero_data = data['AerodynamicDatabase']
        
        "Aerodynamic Parameters"
        self.sref = float(aero_data['data']['sref'][0])
        self.cbar = float(aero_data['data']['cbar'][0])
        self.bref = float(aero_data['data']['bref'][0])
        self.c    = 343
        self.density = 1.225

        "Breakpoints"
        alpha = np.array(aero_data['data']['alpha'][0][0][0], dtype='float32') * np.pi / 180
        self.alpha_min = alpha[0]
        self.alpha_max = alpha[-1]
        nalpha = alpha.size

        beta = np.array(aero_data['data']['beta'][0][0][0], dtype='float32') * np.pi / 180
        self.beta_min = beta[0]
        self.beta_max = beta[-1]
        nbeta = beta.size
		
        mach = np.array(aero_data['data']['mach'][0][0][0], dtype='float32')
        nmach = mach.size
        self.mach_min = mach[0]
        self.mach_max = mach[-1]
		
        "Drag Force Coefficients"
        CD = np.array(aero_data['data']['CD'][0][0], dtype='float32')
		
        self.CD_interpn = RegularGridInterpolator((alpha, beta, mach), CD, method='linear', bounds_error=False, fill_value=None)

        "Lift Force Coefficients"
        CL = np.array(aero_data['data']['CL'][0][0], dtype='float32')
        self.CLq = float(aero_data['data']['CLq'][0][0][0])
        self.Czde = float(aero_data['data']['Czde'][0][0][0])

        self.CL_interpn = RegularGridInterpolator((alpha,beta, mach), CL, method='linear', bounds_error=False, fill_value=None)
		#self.clq_interpn = RegularGridInterpolator((alpha,mach), clq, method='linear', bounds_error=False, fill_value=nan)

        "Force Coefficient on y-axis"
        Cy = np.array(aero_data['data']['Cy'][0][0], dtype='float32')
		
        self.Cyp = float(aero_data['data']['Cyp'][0][0][0])
        self.Cyr = float(aero_data['data']['Cyr'][0][0][0])
        self.Cydr = float(aero_data['data']['Cydr'][0][0][0])
		
        self.Cy_interpn = RegularGridInterpolator((alpha, beta, mach), Cy, method='linear', bounds_error=False, fill_value=None)
				
        "Rolling Moment Coefficients"
        Cl = np.array(aero_data['data']['Cl'][0][0], dtype='float32')

        self.Clp = float(aero_data['data']['Clp'][0][0][0])
        self.Clr = float(aero_data['data']['Clr'][0][0][0])
        self.Clda = float(aero_data['data']['Clda'][0][0][0])
        self.Cldr = float(aero_data['data']['Cldr'][0][0][0])
        self.Cl_interpn = RegularGridInterpolator((alpha, beta, mach), Cl, method='linear', bounds_error=False, fill_value=None)

        "Pitching Moment Coefficients"
        Cm = np.array(aero_data['data']['Cm'][0][0], dtype='float32')
        self.Cmq = float(aero_data['data']['Cmq'][0][0][0])
        self.Cmde = float(aero_data['data']['Cmde'][0][0][0])
        self.Cm_interpn = RegularGridInterpolator((alpha, beta, mach), Cm, method='linear', bounds_error=False, fill_value=None)		
		
        "Yawing Moment Coefficients"
        Cn = np.array(aero_data['data']['Cn'][0][0], dtype='float32')
        self.Cnp = float(aero_data['data']['Cnp'][0][0][0])
        self.Cnr = float(aero_data['data']['Cnr'][0][0][0])
        self.Cnda = float(aero_data['data']['Cnda'][0][0][0])
        self.Cndr = float(aero_data['data']['Cndr'][0][0][0])
        self.Cn_interpn = RegularGridInterpolator((alpha, beta, mach), Cn, method='linear', bounds_error=False, fill_value=None)   
        
    
    def aerodynamic_model(self, wind_param, rates_body, cont_surf):
        Vt = wind_param[0,0]
        if Vt == 0:
            return np.array([[0],[0],[0]]), np.array([[0],[0],[0]])
        alpha = wind_param[1,0]
        beta = wind_param[2,0]
        
        p = rates_body[0,0]
        q = rates_body[1,0]
        r = rates_body[2,0]
        
        delev = cont_surf[0,0]
        dail = cont_surf[1,0]
        drud = cont_surf[2,0]
        
        mach = Vt / self.c
        
        if alpha > self.alpha_max:
            alpha = self.alpha_max
        elif alpha < self.alpha_min:
            alpha = self.alpha_min
        if beta > self.beta_max:
            beta = self.beta_max
        elif beta < self.beta_min:
            beta = self.beta_min
        if mach > self.mach_max:
            mach = self.mach_max
        elif mach < self.mach_min:
            mach = self.mach_min
            
        dyn_press = Vt**2 * self.density / 2
        
        CD = self.CD_interpn((alpha, beta, mach))
        CL = self.CL_interpn((alpha, beta, mach)) + q * self.cbar / (2 * Vt) * self.CLq
        Cy = self.Cy_interpn((alpha, beta, mach))
        Cl = self.Cl_interpn((alpha, beta, mach))
        Cm = self.Cm_interpn((alpha, beta, mach))
        Cn = self.Cn_interpn((alpha, beta, mach))
        
        Cx = -CD * np.cos(alpha) + CL * np.sin(alpha)
        Cz = -CD * np.sin(alpha) - CL * np.cos(alpha)
        
        X = Cx * dyn_press * self.sref
        Z = (Cz + self.Czde * delev)  * dyn_press * self.sref
        Y = (Cy + self.bref * p / (2 * Vt) * self.Cyp + self.bref * r / (2 * Vt) * self.Cyr + self.Cydr * drud) * dyn_press * self.sref
        L = (Cl + self.bref * p / (2 * Vt) * self.Clp + self.bref * r / (2 * Vt) * self.Clr + self.Clda * dail + self.Cldr * drud) * self.sref * self.bref * dyn_press
        M = (Cm + self.Cmde * delev * self.cbar + self.cbar * q / (2 * Vt) * self.Cmq) * self.sref * self.cbar * dyn_press
        N = (Cn + self.bref * p / (2 * Vt) * self.Cnp + self.bref * r / (2 * Vt) * self.Cnr + self.Cnda * dail + self.Cndr * drud) * self.sref * self.bref * dyn_press

        return np.array([[X],[Y],[Z]]), np.array([[L],[M],[N]])
    
    
if __name__ == '__main__':
    aero = aerodynamics()
    wind_param = np.array([[0],[0],[0]])
    wind_param[0,0] = 25
    wind_param[1,0] = 0
    wind_param[2,0] = 0
    
    rates_body = np.array([[0],[0],[0]])
    rates_body[0,0] = 0
    rates_body[1,0] = 0
    rates_body[2,0] = 0
    
    cont_surf = np.array([[0],[0],[0]])
    cont_surf[0,0] = 0
    cont_surf[1,0] = 0
    cont_surf[2,0] = 0
    
    Fa, Ma = aero.aerodynamic_model(wind_param, rates_body, cont_surf)
        