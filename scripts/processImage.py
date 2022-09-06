
"""
Created on Wed Aug 18 12:01:49 2021

@author: turan
"""

import cv2
import cv2.aruco as aruco
import numpy as np
import math
import os
from utils import *

class readImage:
    
    def __init__(self, _IM_ID, _IM_SIZE):
        
        self.__id                = _IM_ID
        self.__size              = _IM_SIZE
        self.__path              = os.path.dirname(os.path.abspath(__file__))
        self.__path              = self.__path + "/camera_config/"
        self.__camMatrix         = np.loadtxt(self.__path+'cameraMatrix_webcam.txt', delimiter=',')
        self.__camDistortion     = np.loadtxt(self.__path+'cameraDistortion_webcam.txt', delimiter=',')
        self.__RFlip             = np.array([[1,0,0],[0,-1,0],[0,0,-1]])
        self.__R_PLANE_MARKER    = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        #self.__RFlip             = np.array([[0,-1,0],[-1,0,0],[0,0,-1]])
        self.__dict              = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.__params            = aruco.DetectorParameters_create()
        self.__cap               = cv2.VideoCapture(0)
        self.__cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.__cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 800)        
        self.__font = cv2.FONT_HERSHEY_PLAIN
        
        self.pos = np.array([[],[],[]])
        self.ori = np.array([[],[],[]])



    #------------------------------------------------------------------------------
    #------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
    #------------------------------------------------------------------------------
    # Checks if a matrix is a valid rotation matrix.
    def __isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6
    
    
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def __rotationMatrixToEulerAngles(self, R):
        assert (self.__isRotationMatrix(R))
    
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    
        singular = sy < 1e-6
    
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
    
        return np.array([x, y, z])
        
        
    def process_image(self, img, ori_vec=None):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=img, dictionary=self.__dict, parameters=self.__params)
        
        if ids is not None and ids[0] == self.__id:
            ret = aruco.estimatePoseSingleMarkers(corners, self.__size, self.__camMatrix, self.__camDistortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0].T
            aruco.drawDetectedMarkers(img, corners)
            aruco.drawAxis(img, self.__camMatrix, self.__camDistortion, rvec, tvec, self.__size)

            if ori_vec is None:
                R_ct    = np.array(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T
                
                roll_marker, pitch_marker, yaw_marker = self.__rotationMatrixToEulerAngles(self.__RFlip@R_tc)
                
                str_attitude = "MARKER Attitude phi=%f  theta=%f  psi=%f"%(roll_marker,pitch_marker,yaw_marker)          
                cv2.putText(img, str_attitude, (0, 150), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.ori = np.array([[-roll_marker], [-pitch_marker], [-yaw_marker]])
                
                #tvec = self.__R_PLANE_MARKER @ earth2body_transformation(self.ori, tvec)
                str_position = "MARKER Position x=%f  y=%f  z=%f"%(tvec[0]/100, tvec[1]/100, tvec[2]/100)
                cv2.putText(img, str_position, (0, 100), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
                
                pos_camera = -self.__R_PLANE_MARKER @ earth2body_transformation(self.ori, tvec)
                
                str_position = "CAMERA Position x=%f  y=%f  z=%f"%(pos_camera[0]/100, pos_camera[1]/100, pos_camera[2]/100)
                cv2.putText(img, str_position, (0, 200), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.pos = np.array([[pos_camera[0]], [pos_camera[1]], [pos_camera[2]]])            
                
                roll_camera, pitch_camera, yaw_camera = self.__rotationMatrixToEulerAngles(self.__RFlip@R_tc)
                str_attitude = "CAMERA Attitude r=%f  p=%f  y=%f"%(roll_camera, pitch_camera, yaw_camera)
                cv2.putText(img, str_attitude, (0, 250), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
            else:
                ########################
                R_ct    = np.array(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T
                
                roll_marker, pitch_marker, yaw_marker = self.__rotationMatrixToEulerAngles(self.__RFlip@R_tc)
                
                str_attitude = "MARKER Attitude phi=%f  theta=%f  psi=%f"%(roll_marker,pitch_marker,yaw_marker)          
                #cv2.putText(img, str_attitude, (0, 150), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.ori = np.array([[-roll_marker], [-pitch_marker], [-yaw_marker]])
                print(self.ori)
                
                ########################
                R_ct    = np.array(cv2.Rodrigues(self.__R_PLANE_MARKER.T@ori_vec)[0])
                R_tc    = R_ct.T
                
                roll_marker, pitch_marker, yaw_marker = self.__rotationMatrixToEulerAngles(R_tc)
                
                str_attitude = "MARKER Attitude phi=%f  theta=%f  psi=%f"%(roll_marker,pitch_marker,yaw_marker)          
                cv2.putText(img, str_attitude, (0, 150), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.ori = np.array([[roll_marker], [pitch_marker], [yaw_marker]])
                print(self.ori)

                #tvec = self.__R_PLANE_MARKER @ earth2body_transformation(self.ori, tvec)
                str_position = "MARKER Position x=%f  y=%f  z=%f"%(tvec[0]/100, tvec[1]/100, tvec[2]/100)
                cv2.putText(img, str_position, (0, 100), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    
                #pos_camera = -R_tc @ tvec
                pos_camera = -self.__R_PLANE_MARKER @ earth2body_transformation(self.ori, tvec)
                
                str_position = "CAMERA Position x=%f  y=%f  z=%f"%(pos_camera[0]/100, pos_camera[1]/100, pos_camera[2]/100)
                cv2.putText(img, str_position, (0, 200), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                self.pos = np.array([[pos_camera[0]], [pos_camera[1]], [pos_camera[2]]])            
                
                roll_camera, pitch_camera, yaw_camera = self.__rotationMatrixToEulerAngles(R_tc)
                str_attitude = "CAMERA Attitude r=%f  p=%f  y=%f"%(roll_camera, pitch_camera, yaw_camera)
                cv2.putText(img, str_attitude, (0, 250), self.__font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                
            
            
        cv2.imshow('frame', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.__cap.release()
            cv2.destroyAllWindows()
    
        return img, self.pos, self.ori

        
    
    
