
"""
Created on Tue Aug  3 12:02:31 2021

@author: turan

"""

import numpy as np



def rotzB2E(angle):
    return np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])


def rotzE2B(angle):
    return np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])


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
