
"""
Created on Mon Aug  2 16:42:24 2021

@author: turan
"""

import numpy as np
import math
from utils import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import optimize

class dubinsPath():
    
    def __init__(self, config):
        self.config = config
        self.Vt = config['Vt']
        self.phi_bar = config['phi_bar']
        self.gamma_bar = config['gamma_bar']
        self.altClass = 'low'
        self.Rmin = self.minRadius()
        self.flag_req = 1
        self.step = config['step']
        
        solution = pathSolution(config)
                
        
    def minRadius(self):
        g = self.config['g']
        Rmin = self.Vt**2 / (g * math.tan(self.phi_bar))
        return Rmin
    
    
    def arccos(self, val):
        if val > 1:
            return 0
        elif val < -1:
            return np.pi
        else:
            return math.acos(val)
        
        
    def arcsin(self, val):
        if val > 1:
            return np.pi
        elif val < -1:
            return -np.pi
        else:
            return math.asin(val)
        
        
    def optimalRadius(self, R, start_node, end_node, circle_points, ind, k, gamma_bar):
        R1 = self.Rmin
        R2 = 2 * self.Rmin
        R  = (R1 + R2) / 2
        error = 1
        
        while abs(error) > 0.1:
            circle_points   = self.calcCircleLoc(start_node, end_node, R)
            pathLengths     = self.calcPathLenght(start_node, end_node, R, circle_points)
            L               = pathLengths[ind]
            
            error  = (L + 2 * np.pi * k * R) * math.tan(gamma_bar) - abs(start_node[2]-end_node[2]) 
            
            if error > 0:
                R2 = R
            else:
                R1 = R
            R = (R1 + R2) / 2
        
        R_Star  = R
        return R
    
    
    def optimizeSprial(self, R, start_node, end_node, circle_points, ind, gamma_bar, flag):
        delta_h = start_node[2]-end_node[2]
        psii = 0
        psi1 = 0
        psi2 = 2 * np.pi
        psi = (psi1 + psi2) / 2
        
        circle_points   = self.calcCircleLoc(start_node, end_node, R)
        pathLengths     = self.calcPathLenght(start_node, end_node, R, circle_points)
        L               = pathLengths[ind]
        
        error = L - abs(delta_h / math.tan(gamma_bar))
        if flag == True: #Means Ascent
            while abs(error) > 0.001:
                L, c_i, z_i, inter_point = self.calcInterPointsParametersDescent(R, start_node, end_node, circle_points, psi, ind)
                error = (L + abs(psi) * R) - abs(delta_h / tan(gamma_bar))
                if error > 0:
                    psi2 = (179*psi2+psi)/180
                else:
                    psi1 = (179*psi1+psi)/180
    
                psi = (psi1 + psi2) / 2
            
            L, c_i, z_i, inter_point = self.calcInterPointsParametersAscent(R, start_node, end_node, circle_points, psi, ind)
            L = L + abs(psi) * R
            psi_i = psi
            
            
            
        if flag == False: #Means Descent
            while abs(error) > 0.001:
                L, c_i, z_i, inter_point = self.calcInterPointsParametersDescent(R, start_node, end_node, circle_points, psi, ind)
                error = (L + fabs(psi) * R) - abs(delta_h / tan(gamma_bar))
                if error > 0:
                    psi2 = (179*psi2+psi)/180
                else:
                    psi1 = (179*psi1+psi)/180
    
                psi = (psi1 + psi2) / 2
            
            L, c_i, z_i, inter_point = self.calcInterPointsParametersDescent(R, start_node, end_node, circle_points, psi, ind)
            L = L + abs(psi) * R
            psi_i = psi
        
        return z_i, psi_i, c_i, inter_point, L
    
    
    def calcInterPointsParametersDescent(self, R, start_node, end_node, circle_points, psi, ind):
        if ind == 0:    
            z_i         = circle_points[0] + rotzB2E(psi) @ (start_node[0:3] - circle_points[0])
            angle_i     = start_node[3] + psi
            c_i         = z_i + R * rotzB2E(-np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcRSRPathLength(c_i, circle_points[2], inter_point, end_node[0:3], R)
            
        elif ind == 1:
            z_i         = circle_points[1] + rotzB2E(psi) @ (start_node[0:3] - circle_points[1])
            angle_i     = start_node[3] + psi
            c_i         = z_i + R * rotzB2E(-np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcRSLPathLength(c_i, circle_points[3], inter_point, end_node[0:3], R)
            
        elif ind == 2:
            z_i         = circle_points[2] + rotzB2E(-psi) @ (start_node[0:3] - circle_points[2])
            angle_i     = start_node[3] - psi
            c_i         = z_i + R * rotzB2E(np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcLSRPathLength(c_i, circle_points[2], inter_point, end_node[0:3], R)
            
        elif ind == 3:
            z_i         = circle_points[3] + rotzB2E(-psi) @ (start_node[0:3] - circle_points[3])
            angle_i     = start_node[3] - psi
            c_i         = z_i + R * rotzB2E(np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcLSLPathLength(c_i, circle_points[3], inter_point, end_node[0:3], R)
            
        return L, c_i, z_i, inter_point
    
    
    def calcInterPointsParametersAscent(self, R, start_node, end_node, circle_points, psi, ind):
        if ind == 0:    
            z_i         = circle_points[0] + rotzB2E(-psi) @ (end_node[0:3] - circle_points[0])
            angle_i     = end_node[3] - psi
            c_i         = z_i + R * rotzB2E(-np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcRSRPathLength( circle_points[0], c_i, start_node[0:3], inter_point, R)
            
        elif ind == 1:
            z_i         = circle_points[1] + rotzB2E(psi) @ (end_node[0:3] - circle_points[1])
            angle_i     = end_node[3] + psi
            c_i         = z_i + R * rotzB2E(np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcRSLPathLength( circle_points[0], c_i, start_node[0:3], inter_point, R)
            
        elif ind == 2:
            z_i         = circle_points[2] + rotzB2E(-psi) @ (end_node[0:3] - circle_points[2])
            angle_i     = end_node[3] - psi
            c_i         = z_i + R * rotzB2E(-np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcLSRPathLength( circle_points[1], c_i, start_node[0:3], inter_point, R)
            
        elif ind == 3:
            z_i         = circle_points[3] + rotzB2E(psi) @ (end_node[0:3] - circle_points[3])
            angle_i     = end_node[3] + psi
            c_i         = z_i + R * rotzB2E(np.pi/2) @ np.array([np.cos(angle_i), np.sin(angle_i), 0])
            inter_point = np.array([[c_i],[angle_i]])
            L, _, _, _  = self.calcLSLPathLength( circle_points[1], c_i, start_node[0:3], inter_point, R)
            
        return L, c_i, z_i, inter_point
            
    
    
    def calcCircleLoc(self, start_point, end_point, R):
        ksi_s = start_point[3,0]
        ksi_e = end_point[3,0]
        crs_p = start_point[0:3] + R * rotzB2E(np.pi/2) @  np.array([[math.cos(ksi_s)],[math.sin(ksi_s)],[0]])
        cls_p = start_point[0:3] + R * rotzB2E(-np.pi/2) @ np.array([[math.cos(ksi_s)],[math.sin(ksi_s)],[0]])
        cre_p = end_point[0:3] + R * rotzB2E(np.pi/2) @ np.array([[math.cos(ksi_e)],[math.sin(ksi_e)],[0]])
        cle_p = end_point[0:3] + R * rotzB2E(-np.pi/2) @ np.array([[math.cos(ksi_e)],[math.sin(ksi_e)],[0]])

        return (crs_p, cls_p, cre_p, cle_p)


    def calcLineAngle2D(self, point1, point2):
        angle = math.atan2(point2[1,0]-point1[1,0], point2[0,0]-point1[0,0])
        return angle
    
    
    def calcLineMagnitude(self, p1, p2):  
        dist = math.sqrt((p1[0,0]-p2[0,0])**2 + (p1[1,0]-p2[1,0])**2)
        return dist
    
    
    def calcRSRPathLength(self, crs_p, cre_p, start_point, end_point, R, ks=0, ke=0):
        angle1 = self.calcLineAngle2D(crs_p, cre_p)
        l1     = self.calcLineMagnitude(crs_p, cre_p)
        l2     = R * np.mod(2 * np.pi + np.mod(angle1-np.pi/2,2*np.pi) - np.mod(start_point[3,0]-np.pi/2,np.pi*2),np.pi*2) + 2 * np.pi * R * ks         
        l3     = R * np.mod(2 * np.pi - np.mod(angle1-np.pi/2,2*np.pi) + np.mod(end_point[3,0]-np.pi/2,np.pi*2),np.pi*2) + 2 * np.pi * R * ke
        return l1 + l2 + l3, l1, l2, l3


    def calcRSLPathLength(self, crs_p, cle_p, start_point, end_point, R, ks=0, ke=0):
        angle1 = self.calcLineAngle2D(crs_p, cle_p)
        l      = self.calcLineMagnitude(crs_p, cle_p)   
        angle2 = angle1 - np.pi/2 + self.arcsin(2*R/l)
        l1     = math.inf if (l**2 - 4*R**2) < 0 else math.sqrt(l**2 - 4*R**2)   
        l2     = R * np.mod(2*np.pi + np.mod(angle2, np.pi*2)-np.mod(start_point[3,0]-np.pi/2, np.pi*2),np.pi*2) + 2 * np.pi * R * ks 
        l3     = R * np.mod(2*np.pi + np.mod(angle2+np.pi, np.pi*2)-np.mod(end_point[3,0]+np.pi/2, np.pi*2),np.pi*2) + 2 * np.pi * R * ke
        return l1 + l2 + l3, l1, l2, l3
    
    
    def calcLSRPathLength(self, cls_p, cre_p, start_point, end_point, R, ks=0, ke=0):
        angle1 = self.calcLineAngle2D(cls_p, cre_p)
        l      = self.calcLineMagnitude(cls_p, cre_p)   
        angle2 = self.arccos(2*R/l)
        l1     = math.inf if (l**2 - 4*R**2) < 0 else math.sqrt(l**2 - 4*R**2)   
        l2     = R * np.mod(2*np.pi - np.mod(angle1+angle2, np.pi*2) + np.mod(start_point[3,0]+np.pi/2, np.pi*2),np.pi*2) + 2 * np.pi * R * ks 
        l3     = R * np.mod(2*np.pi - np.mod(angle1+angle2-np.pi, np.pi*2) + np.mod(end_point[3,0]-np.pi/2, np.pi*2),np.pi*2) + 2 * np.pi * R * ke
        return l1 + l2 + l3, l1, l2, l3
    
    
    def calcLSLPathLength(self, cls_p, cle_p, start_point, end_point, R, ks=0, ke=0):
        angle1 = self.calcLineAngle2D(cls_p, cle_p)
        l1     = self.calcLineMagnitude(cls_p, cle_p)
        l2     = R * np.mod(2*np.pi - np.mod(angle1+np.pi/2,2*np.pi) + np.mod(start_point[3,0]+np.pi/2,np.pi*2),np.pi*2) + 2 * np.pi * R * ks 
        l3     = R * np.mod(2*np.pi + np.mod(angle1+np.pi/2,2*np.pi) - np.mod(end_point[3,0]+np.pi/2,np.pi*2),np.pi*2) + 2 * np.pi * R * ke       
        return l1 + l2 + l3, l1, l2, l3
    
    
    def calcPathLenght(self, start_point, end_point, R, circle_p, ks=0, ke=0):
        (crs_p, cls_p, cre_p, cle_p) = circle_p
        L1, _, _, _ = self.calcRSRPathLength(crs_p, cre_p, start_point, end_point, R, ks, ke)
        L2, _, _, _ = self.calcRSLPathLength(crs_p, cle_p, start_point, end_point, R, ks, ke)
        L3, _, _, _ = self.calcLSRPathLength(cls_p, cre_p, start_point, end_point, R, ks, ke)
        L4, _, _, _ = self.calcLSLPathLength(cls_p, cle_p, start_point, end_point, R, ks, ke)
        return (L1, L2, L3, L4)
    
    
    def func(self, R, start_node, end_node, circle_p, ind, k, gamma_bar):
        circle_points   = self.calcCircleLoc(start_node, end_node, R)
        pathLengths     = self.calcPathLenght(start_node, end_node, R, circle_points)
        L = pathLengths[ind]
        return (L + 2 * np.pi * k * R)  - abs(start_node[2]-end_node[2]) / math.tan(gamma_bar)
    
        
    def test(self, start_node, end_node, R):
        return 
    
    
    def calcMinLength(self, L):
        ind  = np.argmin(L)
        minL = L[ind] 
        return minL, ind
    
    
    def checkRequirements(self, R, start_point, end_point):
        if R < self.Rmin:
            self.flag_req = False
        if np.linalg.norm(start_point-end_point) < 3*R:
            self.flag_req = False        
        return self.flag_req
    
    
    def findDubinsCarParameters(self):
        start_node  = self.config['start_node']
        end_node    = self.config['end_node']
        R           = self.Rmin
        e1          = np.array([[1],[0],[0]])
        if not self.checkRequirements(R, start_node, end_node):
            print("Start and End Node does not meet the reuqirements!")
            return
        
        circle_points   = self.calcCircleLoc(start_node, end_node, R)
        pathLengths     = self.calcPathLenght(start_node, end_node, R, circle_points)
        L, index        = self.calcMinLength(pathLengths)
        
        if index == 0:
            cs = circle_points[0]
            lambda_s = 1
            ce = circle_points[2]
            lambda_e = 1
            q1 = (ce - cs) / self.calcLineMagnitude(ce, cs)
            z1 = cs + R * rotzB2E(-np.pi/2) @ q1
            z2 = ce + R * rotzB2E(-np.pi/2) @ q1
        elif index == 1:
            cs = circle_points[0]
            lambda_s = 1
            ce = circle_points[3]
            lambda_e = -1
            l = self.calcLineMagnitude(ce - cs)
            angle1 = self.calcLineAngle2D(cs, ce)
            angle2 = angle1 - np.pi/2 + math.asin(2*R/l)
            q1 = rotzB2E(angle2 + np.pi/2) @ e1
            z1 = cs + R * rotzB2E(angle2) @ e1
            z2 = ce + R * rotzB2E(angle2 + np.pi) @ e1
        elif index == 2:
            cs = circle_points[1]
            lambda_s = -1
            ce = circle_points[2]
            lambda_e = 1
            l = self.calcLineMagnitude(ce - cs)
            angle1 = self.calcLineAngle2D(cs, ce)
            angle2 = math.acos(2*R/l)
            q1 = rotzB2E(angle1 + angle2 - np.pi/2) @ e1
            z1 = cs + R * rotzB2E(angle1 + angle2) @ e1
            z2 = ce + R * rotzB2E(angle1 + angle2 - np.pi) @ e1
        elif index == 3:
            cs = circle_points[1]
            lambda_s = -1
            ce = circle_points[3]
            lambda_e = -1
            q1 = (ce - cs) / self.calcLineMagnitude(ce - cs)
            z1 = cs + R * rotzB2E(-np.pi/2) @ q1
            z2 = ce + R * rotzB2E(-np.pi/2) @ q1
        z3 = end_node[0:3]
        q3 = rotzB2E(end_node[3]) @ e1
        return (L, cs, lambda_s, ce, lambda_e, z1, q1, z2, z3, q3)
    
    
    def findLowHighAltitudeParameters(self, R, gamma, circle_points, L, index, ks=0, ke=0):
        start_node  = self.config['start_node']
        end_node    = self.config['end_node']
        e1          = np.array([[1],[0],[0]])
        if not self.checkRequirements(R, start_node, end_node):
            print("Start and End Node does not meet the reuqirements!")
            return
        
        
        if index == 0:
            cs              = circle_points[0]
            lambda_s        = 1
            ce              = circle_points[2]
            lambda_e        = 1
            _, _, l1, l2    = self.calcRSRPathLength(cs, ce, start_node, end_node, R, ks, ke)  
            angle           = self.calcLineAngle2D(cs, ce)
            w1              = cs + R * rotzB2E(angle - np.pi/2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
            w2              = ce + R * rotzB2E(angle - np.pi/2) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
            q1              = (w2 - w1) / np.linalg.norm(w2-w1)
            psi_e           = angle - np.pi/2        
            ce              = ce  - np.array([[0], [0], [-l2*math.tan(gamma)]])
            psi_s           = start_node[3] - np.pi/2
        elif index == 1:
            cs              = circle_points[0]
            lambda_s        = 1
            ce              = circle_points[3]
            lambda_e        = -1
            _, _, l1, l2    = self.calcRSLPathLength(cs, ce, start_node, end_node, R, ks, ke)
            l               = self.calcLineMagnitude(ce, cs)
            angle1          = self.calcLineAngle2D(cs, ce)
            angle2          = angle1 - np.pi/2 + math.asin(2*R/l)
            w1              = cs + R * rotzB2E(angle2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
            w2              = ce + R * rotzB2E(angle2 + np.pi) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
            q1              = (w2 - w1) / np.linalg.norm(w2-w1)
            psi_e           = angle2 + np.pi 
            ce              = ce  - np.array([[0], [0], [-l2*math.tan(gamma)]])
            psi_s           = start_node[3] - np.pi/2
        elif index == 2:
            cs              = circle_points[1]
            lambda_s        = -1
            ce              = circle_points[2]
            lambda_e        = 1
            _, _, l1, l2    = self.calcLSRPathLength(cs, ce, start_node, end_node, R, ks, ke)     
            l               = self.calcLineMagnitude(ce, cs)
            angle1          = self.calcLineAngle2D(cs, ce)
            angle2          = self.arccos(2*R/l)
            w1              = cs + R * rotzB2E(angle1 + angle2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
            w2              = ce + R * rotzB2E(angle1 + angle2 - np.pi) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
            q1              = (w2 - w1) / np.linalg.norm(w2-w1)
            psi_e           = np.mod(angle1 + angle2 - np.pi, 2*np.pi)
            ce              = ce  - np.array([[0], [0], [-l2*math.tan(gamma)]])
            psi_s           = start_node[3] + np.pi/2
        elif index == 3:
            cs              = circle_points[1]
            lambda_s        = -1
            ce              = circle_points[3]
            lambda_e        = -1
            angle           = self.calcLineAngle2D(cs, ce)
            _, _, l1, l2    = self.calcLSLPathLength(cs, ce, start_node, end_node, R, ks, ke)
            w1              = cs + R * rotzB2E(angle + np.pi/2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
            w2              = ce + R * rotzB2E(angle + np.pi/2) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
            q1              = (w2 - w1) / np.linalg.norm(w2-w1)
            psi_e           = angle + np.pi/2    
            ce              = ce - np.array([[0], [0], [-l2*math.tan(gamma)]])
            psi_s           = start_node[3] + np.pi/2
            
        qs = q1.copy()   
        ws = w1.copy()
        wl = w2.copy()
        ql = q1.copy()
        we = end_node[0:3].copy()
        qe = rotzB2E(end_node[3]) @ e1
        
        psi_i = 0
        ci = 0
        lambda_i = 0
        wi = 0
        qi = 0  
        
        return (R, gamma, cs, psi_s, lambda_s, ws, qs, ks, ci, psi_i, lambda_i, wi, qi, wl, ql, ce, psi_e, lambda_e, we, qe, ke)
    
    
    def findMediumAltitudeParameters(self, R, gamma, circle_points, L, ci, zi, psii, inter_point, index, flag): 
        start_node  = self.config['start_node']
        end_node    = self.config['end_node']
        e1          = np.array([[1],[0],[0]])
        chi         = inter_point[3]
        
        if flag == True:
            #Ascent
            if index == 0:
                cs              = circle_points[0]
                lambda_s        = 1
                ce              = circle_points[2]
                lambda_e        = 1
                ks              = 0
                lambda_i        = -1
                _, _, l1, l2    = self.calcRSRPathLength(ci, ce, inter_point, end_node, R) 
                l               = self.calcLineMagnitude(ce, ci)
                angle1          = self.calcLineAngle2D(ci, ce)
                angle2          = self.arccos(2*R/l)
                w1              = ci + R * rotzB2E(angle - np.pi/2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
                w2              = ce + R * rotzB2E(angle - np.pi/2) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
                q1              = (w2 - w1) / np.linalg.norm(w2-w1)  
                c_i             = ci  + np.array([[0], [0], [-R_min*psii*math.tan(gamma)]])
                psi_s           = start_node[3] - np.pi/2
                psi_i           = chi + np.pi/2
                k_i             = 0
                c_e             = ce  - np.array([[0], [0], [-l2*math.tan(gamma)]])
                psi_e           = np.mod(angle1 + angle2 - np.pi, 2*np.pi)
                w_s             = zi - np.array([[0], [0], [-psii*self.Rmin*np.tan(gamma)]]).T
                q_s             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_i             = w1.copy()
                q_i             = q1.copy()
                w_l             = w2.copy()
                q_l             = q2.copy()
                w_e             = end_node[0:3].copy()
                q_e             = rotzB2E(end_node[3]) @ e1
                
            elif index == 1:
                ce              = circle_points[3]
                l               = self.calcLineMagnitude(ce, ci)
                angle1          = self.calcLineAngle2D(ci, ce)
                _, _, l1, l2    = self.calcRSLPathLength(ci, ce, inter_node, end_node, R)
                l3              = abs(psii * R)
                w1              = ci + R * rotzB2E(angle1 + np.pi/2) @ e1 + np.array([[0],[0],[-(l1+l3)*math.tan(gamma)]])
                w2              = ce + R * rotzB2E(angle1 + np.pi/2) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
                q1              = (w2-w1) / np.linalg.norm(w2-w1) 
                c_s             = circle_points[0]
                psi_s           = start_node[3] - np.pi/2
                lambda_s        = 1
                k_s             = 0
                c_i             = ci + np.array([[0],[0],[-l3*math.tan(gamma)]])
                psi_i           = chi + np.pi/2
                lambda_i        = -1
                k_i             = 0
                c_e             = ce - np.array([[0],[0],[-l2*math.tan(gamma)]])
                psi_e           = angle1 + np.pi/2
                lambda_e        = -1
                k_e             = 0
                w_s             = inter_point[0:3] - np.array([[0],[0],[-l3*math.tan(gamma)]])
                q_s             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_i             = w1
                q_i             = q1
                w_l             = w2
                q_l             = q1
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1
                
            elif index == 2:
                ce              = circle_points[2]
                l               = self.calcLineMagnitude(ce, ci)
                angle1          = self.calcLineAngle2D(ci, ce)
                _, _, l1, l2    = self.calcRSLPathLength(ci, ce, inter_node, end_node, R)
                l3              = abs(psii * R)
                w1              = ci + R * rotzB2E(angle1 - np.pi/2) @ e1 + np.array([[0],[0],[-(l1+l3)*math.tan(gamma)]])
                w2              = ce + R * rotzB2E(angle1 - np.pi/2) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
                q1              = (w2-w1) / np.linalg.norm(w2-w1) 
                c_s             = circle_points[1]
                psi_s           = start_node[3] + np.pi/2
                lambda_s        = -1
                k_s             = 0
                c_i             = ci + np.array([[0],[0],[-l3*math.tan(gamma)]])
                psi_i           = chi - np.pi/2
                lambda_i        = -1
                k_i             = 0
                c_e             = ce - np.array([[0],[0],[-l2*math.tan(gamma)]])
                psi_e           = angle1 - np.pi/2
                lambda_e        = 1
                k_e             = 0
                w_s             = inter_point[0:3] - np.array([[0],[0],[-l3*math.tan(gamma)]])
                q_s             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_i             = w1
                q_i             = q1
                w_l             = w2
                q_l             = q1
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1
                
            elif index == 3:
                ce              = circle_points[3]
                l               = self.calcLineMagnitude(ce, ci)
                angle1          = self.calcLineAngle2D(ci, ce)
                angle2          = angle1 - np.pi/2 + self.arccos(2*R/l)
                _, _, l1, l2    = self.calcLSLPathLength(ci, ce, inter_node, end_node, R)
                l3              = abs(psii * R)
                w1              = ci + R * rotzB2E(angle2) @ e1 + np.array([[0],[0],[-(l1+l3)*math.tan(gamma)]])
                w2              = ce + R * rotzB2E(angle2 + np.pi) @ e1 - np.array([[0],[0],[-l2*math.tan(gamma)]])
                q1              = (w2-w1) / np.linalg.norm(w2-w1) 
                c_s             = circle_points[1]
                psi_s           = start_node[3] + np.pi/2
                lambda_s        = -1
                k_s             = 0
                c_i             = ci + np.array([[0],[0],[-l3*math.tan(gamma)]])
                psi_i           = chi - np.pi/2
                lambda_i        = 1
                k_i             = 0
                c_e             = ce - np.array([[0],[0],[-l2*math.tan(gamma)]])
                psi_e           = angle2 + np.pi
                lambda_e        = -1
                k_e             = 0
                w_s             = inter_point[0:3] - np.array([[0],[0],[-l3*math.tan(gamma)]])
                q_s             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_i             = w1
                q_i             = q1
                w_l             = w2
                q_l             = q1
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1
                
                
                
        else:
            #Descent
            if index == 0:
                cs              = circle_points[0]
                ce              = circle_points[2]
                _, _, l1, l2    = self.calcRSRPathLength(cs, ci, start_node, inter_point, R) 
                l               = self.calcLineMagnitude(cs, ci)
                l3              = abs(psii * R)
                angle1          = self.calcLineAngle2D(cs, ci)
                angle2          = angle1 -np.pi/2 + self.arcsin(2*R/l)
                w1              = cs + R * rotzB2E(angle2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
                w2              = ci + R * rotzB2E(angle2 + np.pi) @ e1 - np.array([[0],[0],[-(l2+l3)*math.tan(gamma)]])
                q1              = (w2 - w1) / np.linalg.norm(w2-w1)  
                c_s             = cs
                psi_s           = start_node[3] - np.pi/2
                lambda_s        = 1
                k_s             = 0 
                c_i             = ci - np.array([[0], [0], [-(l2+l3)*math.tan(gamma)]])
                psi_i           = angle2 + np.pi
                lambda_i        = -1
                k_i             = 0
                c_e             = ce - np.array([[0], [0], [-l3*math.tan(gamma)]])
                psi_e           = end_node[0:3] - np.pi/2 - psii
                lambda_e        = 1
                k_e             = 0
                w_s             = w1
                q_s             = q1
                w_l             = w2
                q_l             = q1
                w_i             = inter_node[0:3] - np.array([[0], [0], [-l2*math.tan(gamma)]])
                q_i             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1
                
            elif index == 1:
                cs              = circle_points[0]
                ce              = circle_points[3]
                _, _, l1, l2    = self.calcRSLPathLength(cs, ci, start_node, inter_point, R) 
                l               = self.calcLineMagnitude(cs, ci)
                l3              = abs(psii * R)
                angle1          = self.calcLineAngle2D(cs, ci)

                w1              = cs + R * rotzB2E(angle1 - np.pi/2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
                w2              = ci + R * rotzB2E(angle1 - np.pi/2) @ e1 - np.array([[0],[0],[-(l2+l3)*math.tan(gamma)]])
                q1              = (w2 - w1) / np.linalg.norm(w2-w1)  
                c_s             = cs
                psi_s           = start_node[3] - np.pi/2
                lambda_s        = 1
                k_s             = 0 
                c_i             = ci - np.array([[0], [0], [-(l2+l3)*math.tan(gamma)]])
                psi_i           = angle1 - np.pi/2
                lambda_i        = 1
                k_i             = 0
                c_e             = ce - np.array([[0], [0], [-l3*math.tan(gamma)]])
                psi_e           = end_node[0:3] + np.pi/2 + psii
                lambda_e        = -1
                k_e             = 0
                w_s             = w1
                q_s             = q1
                w_l             = w2
                q_l             = q1
                w_i             = inter_node[0:3] - np.array([[0], [0], [-l3*math.tan(gamma)]])
                q_i             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1   
                
            elif index == 2:
                cs              = circle_points[1]
                ce              = circle_points[2]
                _, _, l1, l2    = self.calcLSRPathLength(cs, ci, start_node, inter_point, R) 
                l               = self.calcLineMagnitude(cs, ci)
                l3              = abs(psii * R)
                angle1          = self.calcLineAngle2D(cs, ci)
                
                w1              = cs + R * rotzB2E(angle1 + np.pi/2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
                w2              = ci + R * rotzB2E(angle1 + np.pi/2) @ e1 - np.array([[0],[0],[-(l2+l3)*math.tan(gamma)]])
                q1              = (w2 - w1) / np.linalg.norm(w2-w1)
                c_s             = cs
                psi_s           = start_node[3] + np.pi/2
                lambda_s        = -1
                k_s             = 0
                c_i             = ci - np.array([[0],[0],[-(l2+l3)*math.tan(gamma)]])
                psi_i           = angle1 + np.pi/2
                lambda_i        = -1
                k_i             = 0
                c_e             = ce - np.array([[0],[0],[-l3*math.tan(gamma)]])
                psi_e           = end_node[3] - np.pi/2 - psii
                lambda_e        = 1
                k_e             = 0
                w_s             = w1
                q_s             = q1
                w_l             = w2
                q_l             = q1
                z_i             = zi - np.array([[0],[0],[-l3*math.tan(gamma)]])
                q_i             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1  
                
            elif index == 3:
                cs              = circle_points[1]
                ce              = circle_points[3]
                _, _, l1, l2    = self.calcRSRPathLength(cs, ci, start_node, inter_point, R) 
                l               = self.calcLineMagnitude(cs, ci)
                l3              = abs(psii * R)
                angle1          = self.calcLineAngle2D(cs, ci)
                angle2          = self.arccos(2*R/l)
                w1              = cs + R * rotzB2E(angle1 + angle2) @ e1 + np.array([[0],[0],[-l1*math.tan(gamma)]])
                w2              = ci + R * rotzB2E(angle1 + angle2 - np.pi) @ e1 - np.array([[0],[0],[-(l2+l3)*math.tan(gamma)]])
                q1              = (w2 - w1) / np.linalg.norm(w2-w1)  
                c_s             = cs
                psi_s           = start_node[3] + np.pi/2
                lambda_s        = -1
                k_s             = 0 
                c_i             = ci - np.array([[0], [0], [-(l2+l3)*math.tan(gamma)]])
                psi_i           = np.mod(angle1 + angle2 - np.pi, np.pi*2)
                lambda_i        = 1
                k_i             = 0
                c_e             = ce - np.array([[0], [0], [-l3*math.tan(gamma)]])
                psi_e           = end_node[0:3] + np.pi/2 + psii
                lambda_e        = -1
                k_e             = 0
                w_s             = w1
                q_s             = q1
                w_l             = w2
                q_l             = q1
                w_i             = inter_node[0:3] - np.array([[0], [0], [-l3*math.tan(gamma)]])
                q_i             = np.array([[np.cos(chi)],[np.sin(chi)],[0]])
                w_e             = end_node[0:3]
                q_e             = rotzB2E(end_node[3]) @ e1
        
        return (R, gamma, c_s, psi_s, lambda_s, w_s, q_s, c_i, psi_i, lambda_i, w_i, q_i, w_l, q_l, c_e, psi_e, lambda_e, w_e, q_e)


    def DubinsAircraft(self):
        start_node  = self.config['start_node']
        end_node    = self.config['end_node']
        R           = self.Rmin
        e1          = np.array([[1],[0],[0]])
        gamma_bar   = self.config['gamma_bar']
        if not self.checkRequirements(R, start_node, end_node):
            print("Start and End Node does not meet the reuqirements!")
            return
        parameters = tuple()
        circle_points   = self.calcCircleLoc(start_node, end_node, R)
        pathLengths     = self.calcPathLenght(start_node, end_node, R, circle_points)
        L_car, index    = self.calcMinLength(pathLengths)
        delta_h         = -(end_node[2,0] - start_node[2,0])
        print(abs(delta_h) <= L_car * math.tan(gamma_bar))
        if abs(delta_h) <= L_car * math.tan(gamma_bar):
            gamma_star  = math.atan(delta_h / L_car)
            L_air       = L_car / math.cos(gamma_star)            
            params      = self.findLowHighAltitudeParameters(R, gamma_star, circle_points, L_air, index)
            r1          = self.generateSpiralPath(R, gamma_star, params[2], params[3], params[4], params[5], params[6], params[7], self.step)
            r2          = self.generateLinePath(params[6], params[14], params[5], params[13], self.step)
            r3          = self.generateSpiralPath(R, gamma_star, params[15], params[16], params[17], params[18], params[19], params[20], self.step)
            path        = np.hstack((np.hstack((r1, r2)), r3))
        elif abs(delta_h) >= math.tan(gamma_bar) * (L_car + 2 * np.pi * R):
            k = np.floor((abs(delta_h)/math.tan(gamma_bar) - L_car)/(2*np.pi*R))
            if delta_h < 0:
                ke = 0
                ks = k
            else:
                ks = 0
                ke = k
            gamma_star = np.sign(delta_h) * gamma_bar
            #R = optimize.bisect(self.func, R, 2*R, args = (start_node, end_node, circle_points, index, k, gamma_star))
            R = self.optimalRadius(R, start_node, end_node, circle_points, index, k, gamma_bar)
            circle_points   = self.calcCircleLoc(start_node, end_node, R) #Rmin control et
            pathLengths     = self.calcPathLenght(start_node, end_node, R, circle_points)
            L_car           = pathLengths[index]
            L_air           = (L_car + 2 * np.pi * k * R) / math.cos(gamma_bar)
            params      = self.findLowHighAltitudeParameters(R, gamma_star, circle_points, L_air, index, ks, ke)
            r1          = self.generateSpiralPath(R, gamma_star, params[2], params[3], params[4], params[5], params[6], params[7], self.step)
            r2          = self.generateLinePath(params[6], params[14], params[5], params[13], self.step)
            r3          = self.generateSpiralPath(R, gamma_star, params[15], params[16], params[17], params[18], params[19], params[20], self.step)

            path        = np.hstack((np.hstack((r1, r2)), r3))
        else:
            gamma_star = np.sign(delta_h) * gamma_bar
            if delta_h > 0:
                #Descending
                z_i, psi_i, c_i, inter_point, L = self.optimizeSprial(R, start_node, end_node, circle_points, index, gamma_bar, True)
                L = L / cos(gamma_bar)
                params = self.findMediumAltitudeParameters(R, gamma, circle_points, L, ci, zi, psii, inter_point, index, True)
            else:
                #Ascending
                z_i, psi_i, c_i, inter_point, L = self.optimizeSprial(R, start_node, end_node, circle_points, index, gamma_bar, False)
                L = L / cos(gamma_bar)
                params = self.findMediumAltitudeParameters(R, gamma, circle_points, L, ci, zi, psii, inter_point, index, False)
                
        return path, params
    
    
    def generateLinePath(self, q1, q2, z1, z2, step):
        path = z1.copy()
        s    = step
        ind  = 0    
        while (path[:,ind].reshape(3,1) - z2).T @ q2 <= 0:
            path    = np.hstack((path,  z1 + s * q1))
            s       += step
            ind     += 1            
        return path
    
    
    def generateSpiralPath(self, R, gamma, c, angle, lamda, w, q, k, step):
        path  = c + R * np.array([[math.cos(angle)], [math.sin(angle)], [0]])
        plane = np.dot((path[0:2]-w[0:2]).T, q[0:2])
        s     = step
        if (plane > 0).all():
            crossing = 2 * (k + 1)
        else:
            crossing = 2 * k + 1           
        
        while (crossing > 0) or ((plane <= 0).all()):           
            r = (c + R * np.array([[math.cos(lamda*s+angle)], [math.sin(lamda*s+angle)], [-s*math.tan(gamma)]]))
            path = np.hstack((path, r))
            if np.sign(plane) != np.sign(np.dot((r[0:2]-w[0:2]).T, q[0:2])):
                plane = np.dot((r[0:2] - w[0:2]).T, q[0:2])
                crossing = crossing - 1

            s = s + step
        return path
    
    
    def plotPath(self, path):
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot3D(path[0,:], path[1,:], -path[2,:], 'gray')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z');
        return
       

    
    
class pathSolution():
    
    def __init__(self, solution):
        self.solution = solution
    
    
    
if __name__ == '__main__':
    print('hello')
    a = (1,np.array([]),3,4)
    print(type(a[1]))
    q1 = np.array([[1],[0],[0]])
    q2 = np.array([[1],[0],[0]])
    z1 = np.array([[0],[0],[0]])
    z2 = np.array([[100],[0],[0]])
    config_var = dict() 
    dubins_case = 12
    
    if dubins_case == 1: # short climb RSR
        print ('### Path Type: short climb RSR' )
        config_var['start_node']  = np.array( [[0],   [0],   [-100],   [0*np.pi/180]])
        config_var['end_node']    = np.array( [[0], [200],   [-125], [270*np.pi/180]])
    elif dubins_case == 2: # short climb RSL
        print ('### Path Type: short climb RSL')
        config_var['start_node']  = np.array( [[0],   [0],    [-100], [-70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [100],  [-125], [-70*np.pi/180]])
    elif dubins_case == 3: # short climb LSR
        print ('### Path Type: short climb LSR')
        config_var['start_node']  = np.array( [[0],   [0],    [-100], [70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-125], [70*np.pi/180]])
    elif dubins_case == 4: # short climb LSL
        print ('### Path Type: short climb LSL')
        config_var['start_node']  = np.array( [[0],   [0],    [-100],  [70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-125], [-135*np.pi/180]])
    elif dubins_case == 5: # long climb RSR
        print ('### Path Type: long climb RSR')
        config_var['start_node']  = np.array( [[0],   [0],   [-100],   [0*np.pi/180]])
        config_var['end_node']    = np.array( [[0], [200],   [-250], [270*np.pi/180]])
    elif dubins_case == 6: # long climb RSL
        print ('### Path Type: long climb RSL')
        config_var['start_node']  = np.array( [[0],   [0],    [-100], [-70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [100],  [-350], [-70*np.pi/180]])
    elif dubins_case == 7: # long climb LSR
        print ('### Path Type: long climb LSR')
        config_var['start_node']  = np.array( [[0],   [0],    [-350], [70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-100], [70*np.pi/180]])
    elif dubins_case == 8: # long climb LSL
        print ('### Path Type: long climb LSL')
        config_var['start_node']  = np.array( [[0],   [0],    [-350],  [70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-100], [-135*np.pi/180]])
    elif dubins_case == 9: # intermediate climb RLSR (climb at beginning)
        print ('### Path Type: intermediate climb RLSR (climb at beginning)')
        config_var['start_node']  = np.array( [[0],  [ 0],   [-100],   [0*np.pi/180]])
        config_var['end_node']    = np.array( [[0], [200],   [-200], [270*np.pi/180]])
    elif dubins_case == 10: # intermediate climb RLSL (climb at beginning)
        print ('### Path Type: intermediate climb RLSL (climb at beginning)')
        config_var['start_node']  = np.array( [[0],   [0],   [-100], [0*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [100], [-200], [-90*np.pi/180]])
    elif dubins_case == 11: # intermediate climb LRSR (climb at beginning)
        print ('### Path Type: intermediate climb LRSR (climb at beginning)')
        config_var['start_node']  = np.array( [[0],   [0],   [-100], [0*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-200], [90*np.pi/180]])
    elif dubins_case == 12: # intermediate climb LRSL (climb at beginning)
        print ('### Path Type: intermediate climb LRSL (climb at beginning)')
        config_var['start_node']  = np.array( [[0],   [0],   [-100], [0*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-200], [-90*np.pi/180]])
    elif dubins_case == 13: # intermediate climb RSLR (descend at end)
        print ('### Path Type: intermediate climb RSLR (descend at end)')
        config_var['start_node']  = np.array( [[0],   [0],   [-200], [0*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [100], [-100], [90*np.pi/180]])
    elif dubins_case == 14: # intermediate climb RSRL (descend at end)
        print ('### Path Type: intermediate climb RSRL (descend at end)')
        config_var['start_node']  = np.array( [[0],   [0],   [-200], [0*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [100], [-100], [-90*np.pi/180]])
    elif dubins_case == 15: # intermediate climb LSLR (descend at end)
        print ('### Path Type: intermediate climb LSLR (descend at end)')
        config_var['start_node']  = np.array( [[0],   [0],   [-200], [70*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100], [-100], [90*np.pi/180]])
    elif dubins_case == 16: # intermediate climb LSRL (descend at end)
        print ('### Path Type: intermediate climb LSRL (descend at end)')
        config_var['start_node']  = np.array( [[0],   [0],   [-150], [0*np.pi/180]])
        config_var['end_node']    = np.array( [[100], [-100],[-100], [-90*np.pi/180]])
    elif dubins_case == 0: # for fixing errors
        print ('### Path Type: for fixing errors')
        config_var['start_node']  = np.array( [0,   0,   0, 0, VehiclePars.Vairspeed_0] )
        config_var['end_node']    = np.array( [40, -140,  100, 11*np.pi/9, VehiclePars.Vairspeed_0] ) # LSRL
        config_var['end_node']    = np.array( [60, 140,  -140, 0*np.pi/14, VehiclePars.Vairspeed_0] ) # RLSL       
        config_var['end_node']    = np.array( [40, -140,  -100, 10*np.pi/180, VehiclePars.Vairspeed_0] ) # LRSR    
                   
     
    config_var['Vt'] = 15
    config_var['g'] = 9.8065
    config_var['u_max'] = 1.0 
    config_var['gamma_bar'] = np.pi/6
    config_var['phi_bar'] = np.pi/4
    config_var['step'] = 0.1
    dub = dubinsPath(config_var)
    R_min = dub.Rmin
    circle_p = dub.calcCircleLoc(config_var['start_node'], config_var['end_node'], R_min)
    L = dub.calcPathLenght(config_var['start_node'], config_var['end_node'], R_min, circle_p, 0, 0)
    #parameters = dub.findDubinsCarParameters()
    dub.DubinsAircraft()
    #path = dub.generateLinePath(q1,q2,z1,z2,1)
    path, parameters = dub.DubinsAircraft()
    dub.plotPath(path)

    #print(np.argmin(L))
    
