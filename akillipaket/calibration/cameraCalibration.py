

"""
Created on Wed Aug 18 12:01:49 2021

@author: turan
"""

import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math
import os
import time
from utils import *
from sensor_msgs.msg import Image
import rclpy
from rclpy.node import Node
import glob


class cameraCalibration(Node):

    def __init__(self):
        self.height = 0
        self.width  = 0
        self.cal = True
        self.image_data = 0
        super().__init__('Camera_Calibration_Node')
        self.image_subs = self.create_subscription(Image,'/image',self.image_callback, 10)
        self.abs_path = os.path.abspath(os.getcwd()) + '/src/akillipaket/calibration/calibrationFiles/'
        self.get_logger().info(self.abs_path)
        self.filename  = self.abs_path
        #self.img_file  = self.abs_path + '/image_files'
        #self.filename2 = self.abs_path + 'chess_image.png'
        #self.filename3 = self.abs_path + 'undistorted_image.png'
        #self.filename4 = self.abs_path + 'remapped_image.png'
        #self.subscription  # prevent unused variable warning

    def image_callback(self, msg):
        self.get_logger().info('Image has been captured!')
        self.height = msg.height
        self.widht  = msg.width
        self.image_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)


    def calibrate(self):
        self.get_logger().info('Calibration Process')
        ncor_c = 9
        ncor_r = 6

        objp = np.zeros((ncor_r*ncor_c, 3), np.float32)
        objp[:,:2] = np.mgrid[0:ncor_r, 0:ncor_c].T.reshape(-1,2)

        objpoints = []
        imgpoints = []

        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        images = glob.glob(self.filename + '*.png')
        self.get_logger().info(str(images))
        k = 0
        for fname in images:
            image = cv.imread(fname)
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            self.get_logger().info(fname)
            ret, corners = cv.findChessboardCorners(gray, (ncor_r,ncor_c), None)
            self.get_logger().info(str(ret))
            k += 1
            if ret == True:
                objpoints.append(objp)

                corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners)
                cv.drawChessboardCorners(gray, (ncor_r, ncor_c), corners2, ret)
                #cv.imwrite(self.filename + 'chess_image' + str(k) + '.png', gray)
                self.get_logger().info('Converting Image to Gray...')

        if len(objpoints) != 0:

            ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            self.get_logger().info('\n Camera Matrix: \n "%s"' %mtx)
            self.get_logger().info('\n Distortion Coefficients: \n "%s"' %dist)
            #self.get_logger().info('\n Rotation Vector: \n "%s"' %(str(rvecs)))
            #self.get_logger().info('\n Translation Vector: \n "%s"' %(str(tvecs)))
            np.savetxt('src/akillipaket/calibration/calibrationFiles/camera_distortion.csv', dist, delimiter=',')
            np.savetxt('src/akillipaket/calibration/calibrationFiles/camera_matrix.csv', mtx, delimiter=',')


            self.get_logger().info('Calibrating Camera...')
            h,  w = gray.shape[:2]
            newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
            self.get_logger().info('Getting Optimal Camera Matrix...')
            self.get_logger().info('Optimal Camera Matrix: \n "%s"' %newcameramtx)
            dst = cv.undistort(gray, mtx, dist, None, newcameramtx)
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            #cv.imwrite(self.filename3, dst)
            self.get_logger().info('Writing Undistorted Image...')
            mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
            dst = cv.remap(gray, mapx, mapy, cv.INTER_LINEAR)
            self.get_logger().info('Remapping and Writing Image...')
            x, y, w, h = roi
            dst = dst[y:y+h, x:x+w]
            #cv.imwrite(self.filename4, dst)

            mean_error = 0
            for i in range(len(objpoints)):
                imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
                error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
                mean_error += error
            self.get_logger().info( "total error: {}".format(mean_error/len(objpoints)) )
        self.get_logger().info('Calibration has been performed with "%s" images' %str(len(objpoints)))

        return



def main(args=None):
    rclpy.init(args=args)

    cc = cameraCalibration()


    for k in range(1,31):
        cc.get_logger().info('Taking Image'+str(k)+'...')
        for i in range(1, 4):
            cc.get_logger().info(str(i))
            time.sleep(1)
        rclpy.spin_once(cc, timeout_sec=1)
        cv.imwrite(cc.filename + 'image_' + str(k) + '.png', cc.image_data)



    cc.calibrate()
    cc.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
