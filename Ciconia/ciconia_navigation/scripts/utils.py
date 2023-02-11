
"""
Created on Tue Aug  3 12:02:31 2021

@author: turan

"""

import numpy as np
import scipy




def rotzB2E(angle):
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


def rotzE2B(angle):
    return np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])


def euler_transformation_x(phi, vector):
    rotx = np.array([[1, 0, 0], [0, np.cos(phi), np.sin(phi)], [0, -np.sin(phi), np.cos(phi)]])
    return rotx @ vector


def euler_transformation_y(theta, vector):
    roty = np.array([[np.cos(theta), 0, -np.sin(theta)], [0, 1, 0], [np.sin(theta), 0, np.cos(theta)]])
    return roty @ vector


def euler_transformation_z(psi, vector):
    rotz = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    return rotz @ vector


def euler_inverse_transformation_x(phi, vector):
    rotx = np.array([[1, 0, 0], [0, np.cos(phi), -np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    return rotx @ vector


def euler_inverse_transformation_y(theta, vector):
    roty = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-np.sin(theta), 0, np.cos(theta)]])
    return roty @ vector


def euler_inverse_transformation_z(psi, vector):
    rotz = np.array([[np.cos(psi), -np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    return rotz @ vector


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


def gazebo2earth_transformation(psi, vector):
    rotz = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    return  rotz @ vector


def earth2gazebo_transformation(psi, vector):
    rotz = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    return  rotz.T @ vector


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


def euler_rate2body_rate(angles, rates):
    vec1 = np.array([[1, 0,  -np.sin(angles[1,0])]])

    vec2 = np.array([[0, np.cos(angles[0,0]), np.sin(angles[0,0]) * np.cos(angles[1,0])]])

    vec3 = np.array([[0, -np.sin(angles[0,0]), np.cos(angles[0,0]) * np.cos(angles[1,0])]])

    transformation_matrix = np.concatenate((vec1, vec2, vec3), axis = 0)

    vec = transformation_matrix @ rates

    p = vec[0,0]
    q = vec[1,0]
    r = vec[2,0]

    return np.array([[p],[q],[r]])
    
    
def quaternion_to_euler_angle(w, x, y, z):
     ysqr = y * y

     t0 = +2.0 * (w * x + y * z)
     t1 = +1.0 - 2.0 * (x * x + ysqr)
     X = np.arctan2(t0, t1)

     t2 = +2.0 * (w * y - z * x)
     t2 = +1.0 if t2 > +1.0 else t2
     t2 = -1.0 if t2 < -1.0 else t2
     Y = np.arcsin(t2)

     t3 = +2.0 * (w * z + x * y)
     t4 = +1.0 - 2.0 * (ysqr + z * z)
     Z = np.arctan2(t3, t4)

     return X, Y, Z
     
     
def euler_to_quaternion(phi, theta, psi):

    qx = np.sin(phi/2) * np.cos(theta/2) * np.cos(psi/2) - np.cos(phi/2) * np.sin(theta/2) * np.sin(psi/2)
    qy = np.cos(phi/2) * np.sin(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.cos(theta/2) * np.sin(psi/2)
    qz = np.cos(phi/2) * np.cos(theta/2) * np.sin(psi/2) - np.sin(phi/2) * np.sin(theta/2) * np.cos(psi/2)
    qw = np.cos(phi/2) * np.cos(theta/2) * np.cos(psi/2) + np.sin(phi/2) * np.sin(theta/2) * np.sin(psi/2)
 
    return qx, qy, qz, qw


def quaternion_to_gravity(qw, qx, qy, qz):
    x = 2 * (qx * qz - qw * qy)
    y = 2 * (qw * qx + qy * qz)
    z = qw * qw - qx * qx - qy * qy + qz * qz
    return x, y, z


def earth_radiuses(ref_latitude):
    ##
    ## https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/blob/melodic-devel/hector_gazebo_plugins/src/gazebo_ros_gps.cpp
    ##
    
    equatorial_radius = 6378137.0
    flattening = 1.0/298.257223563
    excentrity = 2*flattening - flattening*flattening
    tmp = 1.0 / (1.0 - excentrity * np.sin(ref_latitude * np.pi/180.0) * np.sin(ref_latitude * np.pi/180.0))
    prime_vertical_radius = equatorial_radius * np.sqrt(tmp)

    radius_north = prime_vertical_radius * (1 - excentrity) * tmp
    radius_east  = prime_vertical_radius * np.cos(ref_latitude * np.pi/180.0)
    return radius_north, radius_east


def omega(w):
    x = np.zeros((4,4))
    cross = np.array([[0, -w[2,0], w[1,0]],
                     [ w[2,0], 0, -w[0,0]],
                     [-w[1,0], w[0,0], 0]])
    x[0:3,0:3] = cross
    x[3,0:3]   = -np.transpose(w)
    x[0:3,3]   = w.reshape(3,)
    return x/2


def dq(q, w):
    return omega(w) @ q


def DCM_NB(q):
    q1 = q[0,0]
    q2 = q[1,0]
    q3 = q[2,0]
    q4 = q[3,0]
    qnorm = np.linalg.norm(q[0:3])

    return np.array([[q1**2-q2**2-q3**2+q4**2, 2*(q1*q2+q3*q4), 2*(q1*q3-q2*q4)],
                    [2*(q1*q2-q3*q4), -q1**2+q2**2-q3**2+q4**2, 2*(q2*q3+q4*q1)],
                    [2*(q1*q3+q2*q4), 2*(q2*q3-q4*q1), -q1**2-q2**2+q3**2+q4**2]]) * 1/np.sqrt(qnorm**2+q4**2)

     
def qk(qk, w, dt):
    return scipy.linalg.expm(omega(w)*dt) @ qk
     
