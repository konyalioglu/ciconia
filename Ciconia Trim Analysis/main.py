# -*- coding: utf-8 -*-
"""
Created on Thu Sep 22 11:14:44 2022

@author: Turan Konyalioglu
"""

from utils import *
from trim_analysis import trim_analysis
from numerical_linearization import numerical_linearization
import numpy as np

from models import models
from eom import eom
from collections import defaultdict

import json
import pickle

class analysis:
    
    def __init__(self):
        delta = 0.1
        tol = 1e-6
        precision = 17
        
        self.trim = trim_analysis()
        self.lin = numerical_linearization(delta, tol, precision)
        
        self.model = models()
        self.eom = eom(self.model.m, self.model.inertia_tensor)
        
        self.data = defaultdict(create_int_defaultdict)
        
        

           
    
    def save(self):
        with open('data.pickle', 'wb') as handle:
            pickle.dump(self.data, handle, protocol=pickle.HIGHEST_PROTOCOL)
            
            
            
    def parameters(self):
        
        self.data['Aircraft']['Parameters']['g'] = self.model.g
        self.data['Aircraft']['Parameters']['m'] = self.model.m
        self.data['Aircraft']['Parameters']['density'] = self.model.density
        self.data['Aircraft']['Parameters']['c'] = self.model.c
    
        self.data['Aircraft']['Parameters']['Ixx'] = self.model.Ixx
        self.data['Aircraft']['Parameters']['Iyy'] = self.model.Iyy
        self.data['Aircraft']['Parameters']['Izz'] = self.model.Izz
        self.data['Aircraft']['Parameters']['Ixy'] = self.model.Ixy
        self.data['Aircraft']['Parameters']['Ixz'] = self.model.Ixz
        self.data['Aircraft']['Parameters']['Iyz'] = self.model.Iyz
        
        self.data['Aircraft']['Parameters']['a'] = self.a
        self.data['Aircraft']['Parameters']['b'] = self.b
            
    def quadrotor_case(self):
        
        self.data['Quadrotor']['Full']['Vt'] = 0
        self.data['Quadrotor']['Full']['mach'] = self.data['Quadrotor']['Full']['Vt'] / self.model.c
        
        self.data['Quadrotor']['Full']['A'] = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, -self.model.g, 0, 0, 0],
                        [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, self.model.g, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]], dtype='float32')
        
        
        self.data['Quadrotor']['Full']['B'] = np.array([[0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [-1/self.model.m, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, self.model.b/self.model.Ixx, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, self.model.a/self.model.Iyy, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 1/self.model.Izz]], dtype='float32')
        
        self.data['Quadrotor']['Attitude']['Vt'] = 0
        self.data['Quadrotor']['Attitude']['mach'] = self.data['Quadrotor']['Attitude']['Vt'] / self.model.c
        
        self.data['Quadrotor']['Attitude']['A'] = np.array([[0, 1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, 0, 0],
                                    [0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1],
                                    [0, 0, 0, 0, 0, 0]])
        
        self.data['Quadrotor']['Attitude']['B'] = np.array([[0, 0, 0],
                                    [self.model.b/self.model.Ixx, 0, 0],
                                    [0, 0, 0],
                                    [0, self.model.a/self.model.Iyy, 0],
                                    [0, 0, 0],
                                    [0, 0, 1/self.model.Izz]])
        
        self.data['Quadrotor']['Attitude']['C'] = np.eye(self.data['Quadrotor']['Attitude']['A'].shape[0])
        self.data['Quadrotor']['Attitude']['D'] = 0
        
        trimmed_hover_dt, ch, ct = self.model.prop.get_hover_constants(self.model.m * 1000 / 4)
        self.data['Quadrotor']['Attitude']['hover_dt'] = trimmed_hover_dt
        self.data['Quadrotor']['Attitude']['ch'] = ch
        self.data['Quadrotor']['Attitude']['ct'] = ct
        self.data['Quadrotor']['Attitude']['motor_input_velocity_matrix'] = np.linalg.inv(np.array([[ct, ct, ct, ct],
                                                                         [ct, -ct, -ct, ct],
                                                                         [ct, -ct, ct, -ct],
                                                                         [-ch, -ch, ch, ch]]))
        
        self.data['Quadrotor']['Full']['hover_dt'] = trimmed_hover_dt
        self.data['Quadrotor']['Full']['ch'] = ch
        self.data['Quadrotor']['Full']['ct'] = ct
        self.data['Quadrotor']['Full']['motor_input_velocity_matrix'] = np.linalg.inv(np.array([[ct, ct, ct, ct],
                                                                         [ct, -ct, -ct, ct],
                                                                         [ct, -ct, ct, -ct],
                                                                         [-ch, -ch, ch, ch]]))
        
    
    
    def cases(self):        
        
        self.data['Vt1']['1']['Vt'] = 1
        self.data['Vt1']['1']['mach'] = self.data['Vt1']['1']['Vt'] / self.model.c
        self.data['Vt1']['1']['phi'] = 0
        self.data['Vt1']['1']['lambdaf'] = 0
        self.data['Vt1']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 300, 0, 0, 0]
        self.data['Vt1']['1']['type'] = 'transition'
        self.data['Vt1']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt2']['1']['Vt'] = 2
        self.data['Vt2']['1']['mach'] = self.data['Vt2']['1']['Vt'] / self.model.c
        self.data['Vt2']['1']['phi'] = 0
        self.data['Vt2']['1']['lambdaf'] = 0
        self.data['Vt2']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 300, 0, 0, 0]
        self.data['Vt2']['1']['type'] = 'transition'
        self.data['Vt2']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
        self.data['Vt3']['1']['Vt'] = 3
        self.data['Vt3']['1']['mach'] = self.data['Vt3']['1']['Vt'] / self.model.c
        self.data['Vt3']['1']['phi'] = 0
        self.data['Vt3']['1']['lambdaf'] = 0
        self.data['Vt3']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 300, 0, 0, 0]
        self.data['Vt3']['1']['type'] = 'transition'
        self.data['Vt3']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
        self.data['Vt4']['1']['Vt'] = 4
        self.data['Vt4']['1']['mach'] = self.data['Vt4']['1']['Vt'] / self.model.c
        self.data['Vt4']['1']['phi'] = 0
        self.data['Vt4']['1']['lambdaf'] = 0
        self.data['Vt4']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 280, 0, 0, 0]
        self.data['Vt4']['1']['type'] = 'transition'
        self.data['Vt4']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
        self.data['Vt5']['1']['Vt'] = 5
        self.data['Vt5']['1']['mach'] = self.data['Vt5']['1']['Vt'] / self.model.c
        self.data['Vt5']['1']['phi'] = 0
        self.data['Vt5']['1']['lambdaf'] = 0
        self.data['Vt5']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 260, 0, 0, 0]
        self.data['Vt5']['1']['type'] = 'transition'
        self.data['Vt5']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
        self.data['Vt6']['1']['Vt'] = 6
        self.data['Vt6']['1']['mach'] = self.data['Vt6']['1']['Vt'] / self.model.c
        self.data['Vt6']['1']['phi'] = 0
        self.data['Vt6']['1']['lambdaf'] = 0
        self.data['Vt6']['1']['x0'] = 0
        self.data['Vt6']['1']['type'] = 'transition'
        self.data['Vt6']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
        self.data['Vt7']['1']['Vt'] = 7
        self.data['Vt7']['1']['mach'] = self.data['Vt7']['1']['Vt'] / self.model.c
        self.data['Vt7']['1']['phi'] = 0
        self.data['Vt7']['1']['lambdaf'] = 0
        self.data['Vt7']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 240, 0, 0, 0]
        self.data['Vt7']['1']['type'] = 'transition'
        self.data['Vt7']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt8']['1']['Vt'] = 8
        self.data['Vt8']['1']['mach'] = self.data['Vt8']['1']['Vt'] / self.model.c
        self.data['Vt8']['1']['phi'] = 0
        self.data['Vt8']['1']['lambdaf'] = 0
        self.data['Vt8']['1']['x0'] = 0
        self.data['Vt8']['1']['type'] = 'transition'
        self.data['Vt8']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt9']['1']['Vt'] = 9
        self.data['Vt9']['1']['mach'] = self.data['Vt9']['1']['Vt'] / self.model.c
        self.data['Vt9']['1']['phi'] = 0
        self.data['Vt9']['1']['lambdaf'] = 0
        self.data['Vt9']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 220, 0, 0, 0]
        self.data['Vt9']['1']['type'] = 'transition'
        self.data['Vt9']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
        
            
        self.data['Vt10']['1']['Vt'] = 10
        self.data['Vt10']['1']['mach'] = self.data['Vt10']['1']['Vt'] / self.model.c
        self.data['Vt10']['1']['phi'] = 0
        self.data['Vt10']['1']['lambdaf'] = 0
        self.data['Vt10']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 200, 0, 0, 0]
        self.data['Vt10']['1']['type'] = 'transition'
        self.data['Vt10']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt11']['1']['Vt'] = 11
        self.data['Vt11']['1']['mach'] = self.data['Vt11']['1']['Vt'] / self.model.c
        self.data['Vt11']['1']['phi'] = 0
        self.data['Vt11']['1']['lambdaf'] = 0
        self.data['Vt11']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 140, 0, 0, 0]
        self.data['Vt11']['1']['type'] = 'transition'
        self.data['Vt11']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt12']['1']['Vt'] = 12
        self.data['Vt12']['1']['mach'] = self.data['Vt12']['1']['Vt'] / self.model.c
        self.data['Vt12']['1']['phi'] = 0
        self.data['Vt12']['1']['lambdaf'] = 0
        self.data['Vt12']['1']['x0'] = 0
        self.data['Vt12']['1']['type'] = 'transition'
        self.data['Vt12']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt13']['1']['Vt'] = 13
        self.data['Vt13']['1']['mach'] = self.data['Vt13']['1']['Vt'] / self.model.c
        self.data['Vt13']['1']['phi'] = 0
        self.data['Vt13']['1']['lambdaf'] = 0
        self.data['Vt13']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 120, 0, 0, 0]
        self.data['Vt13']['1']['type'] = 'transition'
        self.data['Vt13']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
        
            
        self.data['Vt14']['1']['Vt'] = 14
        self.data['Vt14']['1']['mach'] = self.data['Vt14']['1']['Vt'] / self.model.c
        self.data['Vt14']['1']['phi'] = 0
        self.data['Vt14']['1']['lambdaf'] = 0
        self.data['Vt14']['1']['x0'] = 0
        self.data['Vt14']['1']['type'] = 'transition'
        self.data['Vt14']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt15']['1']['Vt'] = 15
        self.data['Vt15']['1']['mach'] = self.data['Vt15']['1']['Vt'] / self.model.c
        self.data['Vt15']['1']['phi'] = 0
        self.data['Vt15']['1']['lambdaf'] = 0
        self.data['Vt15']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 110, 0, 0, 0]
        self.data['Vt15']['1']['type'] = 'transition'
        self.data['Vt15']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
            
            
        self.data['Vt16']['1']['Vt'] = 16
        self.data['Vt16']['1']['mach'] = self.data['Vt16']['1']['Vt'] / self.model.c
        self.data['Vt16']['1']['phi'] = 0
        self.data['Vt16']['1']['lambdaf'] = 0
        self.data['Vt16']['1']['x0'] = 0
        self.data['Vt16']['1']['type'] = 'transition'
        self.data['Vt16']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                           (None, None), (None, None), (None, None), \
                                               (None, None), (None, None), (-0.5, 0.5), \
                                                   (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                       (None, None),(None, None),\
                                                           (None, None),(None, None))
           
            
        self.data['Vt17']['1']['Vt'] = 17
        self.data['Vt17']['1']['mach'] = self.data['Vt17']['1']['Vt'] / self.model.c
        self.data['Vt17']['1']['phi'] = 0
        self.data['Vt17']['1']['lambdaf'] = 0
        self.data['Vt17']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 90, 0, 0, 0]
        self.data['Vt17']['1']['type'] = 'transition'
        self.data['Vt17']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                            (None, None), (None, None), (None, None), \
                                                (None, None), (None, None), (-0.5, 0.5), \
                                                    (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                        (None, None),(None, None),\
                                                            (None, None),(None, None))
            
            
        self.data['Vt18']['1']['Vt'] = 18
        self.data['Vt18']['1']['mach'] = self.data['Vt18']['1']['Vt'] / self.model.c
        self.data['Vt18']['1']['phi'] = 0
        self.data['Vt18']['1']['lambdaf'] = 0
        self.data['Vt18']['1']['x0'] = 0
        self.data['Vt18']['1']['type'] = 'transition'
        self.data['Vt18']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                            (None, None), (None, None), (None, None), \
                                                (None, None), (None, None), (-0.5, 0.5), \
                                                    (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                        (None, None),(None, None),\
                                                            (None, None),(None, None))      
        
            
        self.data['Vt19']['1']['Vt'] = 19
        self.data['Vt19']['1']['mach'] = self.data['Vt19']['1']['Vt'] / self.model.c
        self.data['Vt19']['1']['phi'] = 0
        self.data['Vt19']['1']['lambdaf'] = 0
        self.data['Vt19']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 70, 0, 0, 0]
        self.data['Vt19']['1']['type'] = 'transition'
        self.data['Vt19']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                            (None, None), (None, None), (None, None), \
                                                (None, None), (None, None), (-0.5, 0.5), \
                                                    (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                        (None, None),(None, None),\
                                                            (None, None),(None, None))
                 
            
        self.data['Vt20']['1']['Vt'] = 20
        self.data['Vt20']['1']['mach'] = self.data['Vt20']['1']['Vt'] / self.model.c
        self.data['Vt20']['1']['phi'] = 0
        self.data['Vt20']['1']['lambdaf'] = 0
        self.data['Vt20']['1']['x0'] = [0, 0, 0, 0, 0, 0.0, 0, 0.0, 0, 0, 0, 50, 0, 0, 0]
        self.data['Vt20']['1']['type'] = 'transition'
        self.data['Vt20']['1']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13),\
                                            (None, None), (None, None), (None, None), \
                                                (None, None), (None, None), (-0.5, 0.5), \
                                                    (-0.5, 0.5), (-0.5, 0.5), (0, 1),\
                                                        (None, None),(None, None),\
                                                            (None, None),(None, None))
            
            
        self.data['Vt20']['2']['Vt'] = 20
        self.data['Vt20']['2']['mach'] = self.data['Vt20']['2']['Vt'] / self.model.c
        self.data['Vt20']['2']['phi'] = 0
        self.data['Vt20']['2']['lambdaf'] = 0
        self.data['Vt20']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt20']['2']['type'] = 'simplified-flight'
        self.data['Vt20']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt20']['3']['Vt'] = 20
        self.data['Vt20']['3']['mach'] = self.data['Vt20']['3']['Vt'] / self.model.c
        self.data['Vt20']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt20']['3']['lambdaf'] = 0
        self.data['Vt20']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt20']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt20']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt20']['4']['Vt'] = 20
        self.data['Vt20']['4']['mach'] = self.data['Vt20']['4']['Vt'] / self.model.c
        self.data['Vt20']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt20']['4']['lambdaf'] = 0
        self.data['Vt20']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt20']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt20']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt20.5']['2']['Vt'] = 20.5
        self.data['Vt20.5']['2']['mach'] = self.data['Vt20.5']['2']['Vt'] / self.model.c
        self.data['Vt20.5']['2']['phi'] = 0
        self.data['Vt20.5']['2']['lambdaf'] = 0
        self.data['Vt20.5']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt20.5']['2']['type'] = 'simplified-flight'
        self.data['Vt20.5']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt20.5']['3']['Vt'] = 20.5
        self.data['Vt20.5']['3']['mach'] = self.data['Vt20.5']['3']['Vt'] / self.model.c
        self.data['Vt20.5']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt20.5']['3']['lambdaf'] = 0
        self.data['Vt20.5']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt20.5']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt20.5']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt20.5']['4']['Vt'] = 20.5
        self.data['Vt20.5']['4']['mach'] = self.data['Vt20.5']['4']['Vt'] / self.model.c
        self.data['Vt20.5']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt20.5']['4']['lambdaf'] = 0
        self.data['Vt20.5']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt20.5']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt20.5']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
                
            
        self.data['Vt21']['2']['Vt'] = 21
        self.data['Vt21']['2']['mach'] = self.data['Vt21']['2']['Vt'] / self.model.c
        self.data['Vt21']['2']['phi'] = 0
        self.data['Vt21']['2']['lambdaf'] = 0
        self.data['Vt21']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt21']['2']['type'] = 'simplified-flight'
        self.data['Vt21']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt21']['3']['Vt'] = 21
        self.data['Vt21']['3']['mach'] = self.data['Vt21']['3']['Vt'] / self.model.c
        self.data['Vt21']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt21']['3']['lambdaf'] = 0
        self.data['Vt21']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt21']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt21']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
              
            
        self.data['Vt21']['4']['Vt'] = 21
        self.data['Vt21']['4']['mach'] = self.data['Vt21']['4']['Vt'] / self.model.c
        self.data['Vt21']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt21']['4']['lambdaf'] = 0
        self.data['Vt21']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt21']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt21']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
                  
            
        self.data['Vt22']['2']['Vt'] = 22
        self.data['Vt22']['2']['mach'] = self.data['Vt22']['2']['Vt'] / self.model.c
        self.data['Vt22']['2']['phi'] = 0
        self.data['Vt22']['2']['lambdaf'] = 0
        self.data['Vt22']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt22']['2']['type'] = 'simplified-flight'
        self.data['Vt22']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt22']['3']['Vt'] = 22
        self.data['Vt22']['3']['mach'] = self.data['Vt22']['3']['Vt'] / self.model.c
        self.data['Vt22']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt22']['3']['lambdaf'] = 0
        self.data['Vt22']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt22']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt22']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt22']['4']['Vt'] = 22
        self.data['Vt22']['4']['mach'] = self.data['Vt22']['4']['Vt'] / self.model.c
        self.data['Vt22']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt22']['4']['lambdaf'] = 0
        self.data['Vt22']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt22']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt22']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
                 
            
        self.data['Vt23']['2']['Vt'] = 23
        self.data['Vt23']['2']['mach'] = self.data['Vt23']['2']['Vt'] / self.model.c
        self.data['Vt23']['2']['phi'] = 0
        self.data['Vt23']['2']['lambdaf'] = 0
        self.data['Vt23']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt23']['2']['type'] = 'simplified-flight'
        self.data['Vt23']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt23']['3']['Vt'] = 23
        self.data['Vt23']['3']['mach'] = self.data['Vt23']['3']['Vt'] / self.model.c
        self.data['Vt23']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt23']['3']['lambdaf'] = 0
        self.data['Vt23']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt23']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt23']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt23']['4']['Vt'] = 23
        self.data['Vt23']['4']['mach'] = self.data['Vt23']['4']['Vt'] / self.model.c
        self.data['Vt23']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt23']['4']['lambdaf'] = 0
        self.data['Vt23']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt23']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt23']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt24']['2']['Vt'] = 24
        self.data['Vt24']['2']['mach'] = self.data['Vt24']['2']['Vt'] / self.model.c
        self.data['Vt24']['2']['phi'] = 0
        self.data['Vt24']['2']['lambdaf'] = 0
        self.data['Vt24']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt24']['2']['type'] = 'simplified-flight'
        self.data['Vt24']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt24']['3']['Vt'] = 24
        self.data['Vt24']['3']['mach'] = self.data['Vt24']['3']['Vt'] / self.model.c
        self.data['Vt24']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt24']['3']['lambdaf'] = 0
        self.data['Vt24']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt24']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt24']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt24']['4']['Vt'] = 24
        self.data['Vt24']['4']['mach'] = self.data['Vt24']['4']['Vt'] / self.model.c
        self.data['Vt24']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt24']['4']['lambdaf'] = 0
        self.data['Vt24']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt24']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt24']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt25']['2']['Vt'] = 25
        self.data['Vt25']['2']['mach'] = self.data['Vt25']['2']['Vt'] / self.model.c
        self.data['Vt25']['2']['phi'] = 0
        self.data['Vt25']['2']['lambdaf'] = 0
        self.data['Vt25']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt25']['2']['type'] = 'simplified-flight'
        self.data['Vt25']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt25']['3']['Vt'] = 25
        self.data['Vt25']['3']['mach'] = self.data['Vt25']['3']['Vt'] / self.model.c
        self.data['Vt25']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt25']['3']['lambdaf'] = 0
        self.data['Vt25']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt25']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt25']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt25']['4']['Vt'] = 25
        self.data['Vt25']['4']['mach'] = self.data['Vt25']['4']['Vt'] / self.model.c
        self.data['Vt25']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt25']['4']['lambdaf'] = 0
        self.data['Vt25']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt25']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt25']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt26']['2']['Vt'] = 26
        self.data['Vt26']['2']['mach'] = self.data['Vt26']['2']['Vt'] / self.model.c
        self.data['Vt26']['2']['phi'] = 0
        self.data['Vt26']['2']['lambdaf'] = 0
        self.data['Vt26']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt26']['2']['type'] = 'simplified-flight'
        self.data['Vt26']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt26']['3']['Vt'] = 26
        self.data['Vt26']['3']['mach'] = self.data['Vt26']['3']['Vt'] / self.model.c
        self.data['Vt26']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt26']['3']['lambdaf'] = 0
        self.data['Vt26']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt26']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt26']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt26']['4']['Vt'] = 26
        self.data['Vt26']['4']['mach'] = self.data['Vt26']['4']['Vt'] / self.model.c
        self.data['Vt26']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt26']['4']['lambdaf'] = 0
        self.data['Vt26']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt26']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt26']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt27']['2']['Vt'] = 27
        self.data['Vt27']['2']['mach'] = self.data['Vt27']['2']['Vt'] / self.model.c
        self.data['Vt27']['2']['phi'] = 0
        self.data['Vt27']['2']['lambdaf'] = 0
        self.data['Vt27']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt27']['2']['type'] = 'simplified-flight'
        self.data['Vt27']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt27']['3']['Vt'] = 27
        self.data['Vt27']['3']['mach'] = self.data['Vt27']['3']['Vt'] / self.model.c
        self.data['Vt27']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt27']['3']['lambdaf'] = 0
        self.data['Vt27']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt27']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt27']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt27']['4']['Vt'] = 27
        self.data['Vt27']['4']['mach'] = self.data['Vt27']['4']['Vt'] / self.model.c
        self.data['Vt27']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt27']['4']['lambdaf'] = 0
        self.data['Vt27']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt27']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt27']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt28']['2']['Vt'] = 28
        self.data['Vt28']['2']['mach'] = self.data['Vt28']['2']['Vt'] / self.model.c
        self.data['Vt28']['2']['phi'] = 0
        self.data['Vt28']['2']['lambdaf'] = 0
        self.data['Vt28']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt28']['2']['type'] = 'simplified-flight'
        self.data['Vt28']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))


        self.data['Vt28']['3']['Vt'] = 28
        self.data['Vt28']['3']['mach'] = self.data['Vt28']['3']['Vt'] / self.model.c
        self.data['Vt28']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt28']['3']['lambdaf'] = 0
        self.data['Vt28']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt28']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt28']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt28']['4']['Vt'] = 28
        self.data['Vt28']['4']['mach'] = self.data['Vt28']['4']['Vt'] / self.model.c
        self.data['Vt28']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt28']['4']['lambdaf'] = 0
        self.data['Vt28']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt28']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt28']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
            
        self.data['Vt29']['2']['Vt'] = 29
        self.data['Vt29']['2']['mach'] = self.data['Vt29']['2']['Vt'] / self.model.c
        self.data['Vt29']['2']['phi'] = 0
        self.data['Vt29']['2']['lambdaf'] = 0
        self.data['Vt29']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt29']['2']['type'] = 'simplified-flight'
        self.data['Vt29']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt29']['3']['Vt'] = 29
        self.data['Vt29']['3']['mach'] = self.data['Vt29']['3']['Vt'] / self.model.c
        self.data['Vt29']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt29']['3']['lambdaf'] = 0
        self.data['Vt29']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt29']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt29']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt29']['4']['Vt'] = 29
        self.data['Vt29']['4']['mach'] = self.data['Vt29']['4']['Vt'] / self.model.c
        self.data['Vt29']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt29']['4']['lambdaf'] = 0
        self.data['Vt29']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt29']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt29']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
                
            
        self.data['Vt30']['2']['Vt'] = 30
        self.data['Vt30']['2']['mach'] = self.data['Vt30']['2']['Vt'] / self.model.c
        self.data['Vt30']['2']['phi'] = 0
        self.data['Vt30']['2']['lambdaf'] = 0
        self.data['Vt30']['2']['x0'] = [0.02, 0, 0, 0, 0, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt30']['2']['type'] = 'simplified-flight'
        self.data['Vt30']['2']['bnds'] = ((-0.13, 0.13), (-0.13, 0.13), (None, None), \
                                          (None, None), (None, None), (None, None), \
                                              (None, None), (-0.5, 0.5), (-0.5, 0.5), \
                                                  (-0.5, 0.5), (0.0, 1.0))
            
            
        self.data['Vt30']['3']['Vt'] = 30
        self.data['Vt30']['3']['mach'] = self.data['Vt30']['3']['Vt'] / self.model.c
        self.data['Vt30']['3']['phi'] = 45 * np.pi / 180
        self.data['Vt30']['3']['lambdaf'] = 0
        self.data['Vt30']['3']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt30']['3']['type'] = 'steady-state-turn-flight'
        self.data['Vt30']['3']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
        
            
        self.data['Vt30']['4']['Vt'] = 30
        self.data['Vt30']['4']['mach'] = self.data['Vt30']['4']['Vt'] / self.model.c
        self.data['Vt30']['4']['phi'] = -45 * np.pi / 180
        self.data['Vt30']['4']['lambdaf'] = 0
        self.data['Vt30']['4']['x0'] = [0.02, 0.02, 0, 0.0, 0, 0, 1]
        self.data['Vt30']['4']['type'] = 'steady-state-turn-flight'
        self.data['Vt30']['4']['bnds'] = ((-0.13, 0.13), (None, None), (None, None), \
                                          (-0.5, 0.5), (-0.5, 0.5), (-0.5, 0.5),\
                                              (0.0, 1.0))
            
        return self.data
    
    
    def full_analysis(self, key1, key2):
        
        print("####################################################")
        print(" Trimming")
        Vt      = self.data[key1][key2]['Vt']
        mach    = self.data[key1][key2]['mach']
        phi     = self.data[key1][key2]['phi']
        lambdaf = self.data[key1][key2]['lambdaf']
        
        x0 = self.data[key1][key2]['x0']
        bnds = self.data[key1][key2]['bnds']
        
        dynamics = self.data[key1][key2]['type']
        
        if dynamics == 'transition':
            print(" Transition Dynamics at \n Mach: %s \n Bank Angle: %s \n Flight Path Angle: %s \n Case Number: %s" %(mach, phi, lambdaf, key2))
            print(" Trim Case: %s" %(dynamics).upper())
            xtrim = self.trim.optimize_generalized_transition_flight(mach, x0, phi=phi, bnds=bnds)
            Ft, Mt = self.trim.check_trim_states_transition(xtrim, Vt)
            trimmed_hover_dt, ch, ct = self.model.prop.get_hover_constants(xtrim[11]/4/self.model.g*1000)
            xdot, x, u, d = self.trim.convert_trim_states2model_states_transition(xtrim, mach, 0)
            E, Ap, Bp, Dp = self.lin.numerical_linearization(xdot, x, u, d, lin = 'transition')
            A = -np.linalg.inv(E) @ Ap
            B = -np.linalg.inv(E) @ Bp
            D = -np.linalg.inv(E) @ Dp
            C = np.eye(A.shape[0])
            
            self.data[key1][key2]['errorF'] = Ft
            self.data[key1][key2]['errorM'] = Mt
            self.data[key1][key2]['x'] = xtrim
            self.data[key1][key2]['A'] = A
            self.data[key1][key2]['B'] = B
            self.data[key1][key2]['C'] = C
            self.data[key1][key2]['D'] = D
            self.data[key1][key2]['hover_dt'] = trimmed_hover_dt
            self.data[key1][key2]['ch'] = ch
            self.data[key1][key2]['ct'] = ct
            self.data[key1][key2]['motor_input_velocity_matrix'] = np.linalg.inv(np.array([[ct, ct, ct, ct],
                                                                             [ct, -ct, -ct, ct],
                                                                             [ct, -ct, ct, -ct],
                                                                             [-ch, -ch, ch, ch]]))
            
            
            
        elif dynamics == 'simplified-flight':
            print(" Trimming Flight Dynamics at \n Mach: %s \n Bank Angle: %s \n Flight Path Angle: %s \n Case Number: %s" %(mach, phi, lambdaf, key2))
            print(" Trim case %s" %(dynamics).upper())
            xtrim = self.trim.optimize_simplified_straight_flight(mach, x0, phi = phi, bnds=bnds)
            Ft, Mt = self.trim.check_trim_states(xtrim, Vt, phi, lambdaf)
            xdot, x, u, d = self.trim.convert_trim_states2model_states_flight(xtrim, mach, phi)
            E, Ap, Bp, Dp = self.lin.numerical_linearization(xdot, x, u, d, 'flight')
            A = -np.linalg.inv(E) @ Ap
            B = -np.linalg.inv(E) @ Bp
            D = -np.linalg.inv(E) @ Dp
            C = np.eye(A.shape[0])
            
            self.data[key1][key2]['errorF'] = Ft
            self.data[key1][key2]['errorM'] = Mt
            self.data[key1][key2]['x'] = xtrim
            self.data[key1][key2]['A'] = A
            self.data[key1][key2]['B'] = B
            self.data[key1][key2]['C'] = C
            self.data[key1][key2]['D'] = D
            
            
        elif dynamics == 'generalized-flight':
            print(" Trimming Flight Dynamics at \n Mach: %s \n Bank Angle: %s \n Flight Path Angle: %s \n Case Number: %s" %(mach, phi, lambdaf, key2))
            print(" Trim case %s" %(dynamics).upper())
            xtrim = self.trim.optimize_generalized_straight_flight(mach, x0, phi = phi, bnds=bnds)
            Ft, Mt = self.trim.check_trim_states(xtrim, Vt, phi, lambdaf)
            xdot, x, u, d = self.trim.convert_trim_states2model_states_flight(xtrim, mach, phi)
            E, Ap, Bp, Dp = self.lin.numerical_linearization(xdot, x, u, d, 'flight')
            A = -np.linalg.inv(E) @ Ap
            B = -np.linalg.inv(E) @ Bp
            D = -np.linalg.inv(E) @ Dp
            C = np.eye(A.shape[0])
            
            self.data[key1][key2]['errorF'] = Ft
            self.data[key1][key2]['errorM'] = Mt
            self.data[key1][key2]['x'] = xtrim
            self.data[key1][key2]['A'] = A
            self.data[key1][key2]['B'] = B
            self.data[key1][key2]['C'] = C
            self.data[key1][key2]['D'] = D
            
            
        elif dynamics == 'wing-level-flight':
            print(" Trimming Flight Dynamics at \n Mach: %s \n Bank Angle: %s \n Flight Path Angle: %s \n Case Number: %s" %(mach, phi, lambdaf, key2))
            print(" Trim case %s" %(dynamics).upper())
            xtrim = self.trim.optimize_constant_altitude_straight_flight(mach, x0, phi = phi, bnds=bnds)
            Ft, Mt = self.trim.check_trim_states(xtrim, Vt, phi, lambdaf)
            xdot, x, u, d = self.trim.convert_trim_states2model_states_flight(xtrim, mach, phi)
            E, Ap, Bp, Dp = self.lin.numerical_linearization(xdot, x, u, d, 'flight')
            A = -np.linalg.inv(E) @ Ap
            B = -np.linalg.inv(E) @ Bp
            D = -np.linalg.inv(E) @ Dp
            C = np.eye(A.shape[0])
            
            self.data[key1][key2]['errorF'] = Ft
            self.data[key1][key2]['errorM'] = Mt
            self.data[key1][key2]['x'] = xtrim
            self.data[key1][key2]['A'] = A
            self.data[key1][key2]['B'] = B
            self.data[key1][key2]['C'] = C
            self.data[key1][key2]['D'] = D
            
            
        elif dynamics == 'steady-state-turn-flight':
            print(" Trimming Flight Dynamics at \n Mach: %s \n Bank Angle: %s \n Flight Path Angle: %s \n Case Number: %s" %(mach, phi, lambdaf, key2))
            print(" Trim case: %s" %(dynamics).upper())
            xtrim = self.trim.optimize_steady_state_turn(mach, x0, phi = phi, bnds=bnds)
            Ft, Mt = self.trim.check_steady_turn_trim_states(xtrim, Vt, phi)
            xdot, x, u, d = self.trim.convert_steadyturn_trim_states2model_states_flight(xtrim, mach, phi)
            E, Ap, Bp, Dp = self.lin.numerical_linearization(xdot, x, u, d, 'flight')
            A = -np.linalg.inv(E) @ Ap
            B = -np.linalg.inv(E) @ Bp
            D = -np.linalg.inv(E) @ Dp
            C = np.eye(A.shape[0])
                       
            self.data[key1][key2]['errorF'] = Ft
            self.data[key1][key2]['errorM'] = Mt
            self.data[key1][key2]['x'] = xtrim
            self.data[key1][key2]['A'] = A
            self.data[key1][key2]['B'] = B
            self.data[key1][key2]['C'] = C
            self.data[key1][key2]['D'] = D
            
            
    def flight_analysis(self):
        ts = self.trim.optimize_generalized_straight_flight()
        self.trim.check_trim_states(ts)
        return ts
    
    
    def main(self):
        self.data = self.cases()
        for i in self.data.keys():
            for j in self.data[i].keys():
                self.full_analysis(i,j)
        self.quadrotor_case()
                
        with open('state_space_database.pickle', 'wb') as handle:
            pickle.dump(self.data, handle, protocol=pickle.HIGHEST_PROTOCOL)        
        #np.save('state_space_database.npy', self.data)


def create_int_defaultdict():
    return defaultdict(dict)


if __name__ == '__main__':
    analysis = analysis()
    analysis.main()
    
