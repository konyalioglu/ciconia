# -*- coding: utf-8 -*-
"""
Created on Wed Sep 21 14:59:55 2022

@author: Turan Konyalioglu
"""

import os

from scipy.io import loadmat
from scipy.interpolate import RegularGridInterpolator, interp1d
from collections.abc import Iterable
 
import numpy as np


class pair:
    
    def __init__(self, pair_type, r, orientation, rotation):
        """
        Parameters
        ----------
        pair_type : 0 for MN801S - 26x8.5 prop
                    1 for AT7212 - 21x10 prop
        r         : 
                    position vector in type of numpy array
        orientation:
            orientation vector that pair has, relative to body
            
        rotation:  1 for CW
                  -1 for CCW

        Returns
        -------
        None.

        """
        self.type = pair_type
        self.position = r
        self.orientation = orientation
        self.rotation = rotation
        
        self.RPM = 0.0
        self.dt = 0.0
        self.thrust = np.array([[0],[0],[0]])
        self.torque = np.array([[0],[0],[0]])
        
        
        

class propulsion:
    
    def __init__(self):
        
        self.g = 9.8065
        
        
        dir_path = os.path.dirname(os.path.realpath(__file__))
        
        
        prop_data = loadmat(dir_path + '/MN801S_26x85_PropulsionData_py.mat')  
        
        throttle_BP = np.array(prop_data['MN801S_26x85_PropulsionData']['throttle'][0][0]).flatten()
        RPM = np.array(prop_data['MN801S_26x85_PropulsionData']['RPM'][0][0]).flatten()
        thrust = np.array(prop_data['MN801S_26x85_PropulsionData']['thrust'][0][0]).flatten()
        torque = np.array(prop_data['MN801S_26x85_PropulsionData']['torque'][0][0]).flatten()
        
        self.thrust_interpn1 = interp1d(throttle_BP, thrust, kind='linear', bounds_error=False, fill_value=None)
        self.torque_interpn1 = interp1d(throttle_BP, torque, kind='linear', bounds_error=False, fill_value=None)
        self.rpm_interpn1 = interp1d(throttle_BP, RPM, kind='linear', bounds_error=False, fill_value=None)
        self.throttle_interpn1 = interp1d(thrust, throttle_BP, kind='linear', bounds_error=False, fill_value=None)
        
        
        prop_data = loadmat(dir_path + '/AT7217_21x10_PropulsionData_py.mat')   
        
        throttle_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['throttle'][0][0]).flatten()
        RPM_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM_BP'][0][0]).flatten()
        RPM = np.array(prop_data['AT7217_21x10_PropulsionData']['RPM'][0][0]).flatten()
        Mach_BP = np.array(prop_data['AT7217_21x10_PropulsionData']['Mach_BP'][0][0]).flatten()
        thrust = np.array(prop_data['AT7217_21x10_PropulsionData']['thrust'][0][0])
        torque = np.array(prop_data['AT7217_21x10_PropulsionData']['torque'][0][0])
        
        self.rpm_interpn2 = interp1d(throttle_BP, RPM, kind='linear', bounds_error=False, fill_value=None)        
        self.thrust_interpn2 = RegularGridInterpolator((Mach_BP, RPM_BP), thrust, method='linear', bounds_error=False, fill_value=None)   
        self.torque_interpn2 = RegularGridInterpolator((Mach_BP, RPM_BP), torque, method='linear', bounds_error=False, fill_value=None)
        

    def propulsive_forces_moments(self, mach, *pairs):
        Fp = 0
        Mp = 0
        
        if isinstance(pairs[0], Iterable):
            for pair in pairs[0]:
                self.calculate_forces_and_moments(mach, pair)
                Fp += pair.thrust
                Mp += pair.torque + np.cross(pair.position.reshape((1,3)), pair.thrust.reshape((1,3))).reshape((3,1))
        else:
            pair = pairs[0]
            self.calculate_forces_and_moments(mach, pair)
            Fp += pair.thrust
            Mp += pair.torque + np.cross(pair.position.reshape((1,3)), pair.thrust.reshape((1,3))).reshape((3,1))

        return Fp, Mp
            
    
    def calculate_forces_and_moments(self, mach, pair):
        if pair.type == 0:
            pair.RPM = self.rpm_interpn1(pair.dt)
            pair.thrust = self.thrust_interpn1(pair.dt) * pair.orientation  / 1000 * self.g
            pair.torque = self.torque_interpn1(pair.dt) * pair.orientation * -pair.rotation
        elif pair.type == 1:
            pair.RPM = self.rpm_interpn2(pair.dt)
            pair.torque = self.torque_interpn2((mach, pair.RPM)) * pair.orientation * -pair.rotation
            pair.thrust = self.thrust_interpn2((mach, pair.RPM)) * pair.orientation
            
            
    def get_hover_constants(self, thrust):
        throttle  = self.throttle_interpn1(thrust)
        ang_speed = self.rpm_interpn1(throttle)
        torque    = self.torque_interpn1(throttle)
        
        ch = torque / (ang_speed**2)
        ct = thrust / (ang_speed**2)
        
        return throttle, ch, ct
        
            
    