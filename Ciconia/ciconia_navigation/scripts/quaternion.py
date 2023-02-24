# -*- coding: utf-8 -*-

import numpy as np



class Vector3:
    
    def __init__(self, i, j, k):
        self.i = i
        self.j = j
        self.k = k

    

class Quaternion:
    
    
    def __init__(self, qw=None, qx=None, qy=None, qz=None):
        
        self.qw = qw
        self.qx = qx
        self.qy = qy
        self.qz = qz



    def normalize(self):
        norm = np.linalg.norm(np.array([self.qw, self.qx, self.qy, self.qz]))
        return self.qw / norm, self.qx / norm, self.qy / norm, self.qz / norm


    def dot_product(self, quaternion):
        return self.qx * quaternion.qx, self.qy * quaternion.qy, self.qz * quaternion.qz
    
    
    def cross_product(self, quaternion):
        vi = self.qy * quaternion.qz - self.qz * quaternion.qy
        vj = self.qz * quaternion.qx - self.qx * quaternion.qz
        vk = self.qx * quaternion.qy - self.qy * quaternion.qx
        return vi, vj, vk
    
    
    def conjugate(self):
        return self.qw, -self.qx, -self.qy, -self.qz
    
    
    def multiplication(self, quaternion):
        qw = self.qw * quaternion.qw - self.qx * quaternion.qx - self.qy * quaternion.qy - self.qz * quaternion.qz
        qx = self.qx * quaternion.qw + self.qw * quaternion.qx - self.qz * quaternion.qy + self.qy * quaternion.qz
        qy = self.qy * quaternion.qw + self.qz * quaternion.qx + self.qw * quaternion.qy - self.qx * quaternion.qz
        qz = self.qz * quaternion.qw - self.qy * quaternion.qx + self.qx * quaternion.qy + self.qw * quaternion.qz
        return Quaternion(qw, qx, qy, qz).normalize()
    
    
    def transform_vector(self, vector):
        q_conjugate = Quaternion(self.conjugate())
        q1 = Quaternion(self.multiplication(Quaternion(0,vector.i, vector.j, vector.k)))
        q2 = q1.multiplication(q_conjugate)
        return q2.qw, q2.qx, q2.qy, q2.qz
    
    
    def reciprocal(self):
        qw, qx, qy, qz = self.conjugate()
        norm = np.linalg.norm(np.array([self.qw, self.qx, self.qy, self.qz]))
        return qw / norm, qx / norm, qy / norm, qz / norm
    
    
    def DCM(self):
        q1 = self.qx
        q2 = self.qy
        q3 = self.qz
        q4 = self.qw
        qnorm = np.linalg.norm(np.array([q1, q2, q3]))
    
        return np.array([[q1**2-q2**2-q3**2+q4**2, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)],
                        [2*(q1*q2-q3*q4), -q1**2+q2**2-q3**2+q4**2, 2*(q2*q3+q4*q1)],
                        [2*(q1*q3+q2*q4), 2*(q2*q3-q4*q1), -q1**2-q2**2+q3**2+q4**2]]) / np.sqrt(qnorm**2+q4**2)
    
    
    def quaternion_to_euler_angle(self):
         qysqr = self.qy * self.qy
    
         t0 = +2.0 * (self.qw * self.qx + self.qy * self.qz)
         t1 = +1.0 - 2.0 * (self.qx * self.qx + qysqr)
         phi = np.arctan2(t0, t1)
    
         t2 = +2.0 * (self.qw * self.qy - self.qz * self.qx)
         t2 = +1.0 if t2 > +1.0 else t2
         t2 = -1.0 if t2 < -1.0 else t2
         theta = np.arcsin(t2)
    
         t3 = +2.0 * (self.qw * self.qz + self.qx * self.qy)
         t4 = +1.0 - 2.0 * (qysqr + self.qz * self.qz)
         psi = np.arctan2(t3, t4)
    
         return phi, theta, psi
     
        
    def generate_quaternion_with_angle(self, xx, yy, zz, angle):
        
        factor = np.sin(angle / 2.0)
        x = xx * factor
        y = yy * factor
        z = zz * factor
    
        w = np.cos(angle / 2.0)

        return Quaternion(w, x, y, z).normalize()
    
    
    def euler_to_quaternion(self, phi, theta, psi):

        qx = np.sin(phi/2) * np.cos(theta/2) * np.cos(psi/2) - np.cos(phi/2) * np.sin(theta/2) * np.sin(psi/2)
        qy = np.cos(phi/2) * np.sin(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.cos(theta/2) * np.sin(psi/2)
        qz = np.cos(phi/2) * np.cos(theta/2) * np.sin(psi/2) - np.sin(phi/2) * np.sin(theta/2) * np.cos(psi/2)
        qw = np.cos(phi/2) * np.cos(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.sin(theta/2) * np.sin(psi/2)
 
        return Quaternion(qw, qx, qy, qz).normalize()

   


     