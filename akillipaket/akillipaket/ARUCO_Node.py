import cv2 as cv
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import math
from std_msgs.msg import Float64MultiArray


class ArucoNode(Node):

    def __init__(self):
        super().__init__('aruco_node')
        self.subscription = self.create_subscription(Image, '/akillipaket/image', self.arucoCallback,  4)

        self.camera_pose_publisher = self.create_publisher(Float64MultiArray, '/akillipaket/Aruco/Camera/relativePose', 4)
        #self.camera_attitude_publisher = self.create_publisher(Float64MultiArray, '/akillipaket/Aruco/Camera/relativeAttitude', 4)
        #self.marker_position_publisher = self.create_publisher(Float64MultiArray, '/akillipaket/Aruco/Marker/relativePosition', 4)
        self.marker_pose_publisher = self.create_publisher(Float64MultiArray, '/akillipaket/Aruco/Marker/relativePose', 4)

        self.__path = 'src/akillipaket/calibration/calibrationFiles/'
        self.__camera_matrix = np.loadtxt(self.__path + 'camera_matrix.csv', delimiter=',')
        self.__camera_distortion = np.loadtxt(self.__path + 'camera_distortion.csv', delimiter=',')

        self.get_logger().info('\n Camera Matrix: ')
        self.get_logger().info(str(self.__camera_matrix))
        self.get_logger().info('\n Camera Distortion: ')
        self.get_logger().info(str(self.__camera_distortion))


        self.__id = 0
        self.__size = 0.095

        self.__dict   = aruco.Dictionary_get(aruco.DICT_6X6_100)
        self.__params = aruco.DetectorParameters_create()
        self.__font   = cv.FONT_HERSHEY_PLAIN

        self.__RFlip = np.array([[0,1,0],[1,0,0],[0,0,1]])
        self.subscription

        self.marker_pose_msg = Float64MultiArray()
        #self.marker_attitude_msg = Float64MultiArray()
        self.camera_pose_msg = Float64MultiArray()
        #self.camera_attitude_msg = Float64MultiArray()


    def wrap(self, angle, offset):
        yaw = angle + offset
        if yaw > offset - np.pi and yaw < -np.pi:
            yaw = 3*np.pi/2 + angle
        return yaw


    def __isRotationMatrix(self, R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6


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


    def arucoCallback(self, msg):
        self.height = msg.height
        self.widht  = msg.width
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(image=img, dictionary=self.__dict, parameters=self.__params)
        if ids is not None and ids[0] == self.__id:
            self.get_logger().info('Marker Detected!')
            ret = aruco.estimatePoseSingleMarkers(corners, self.__size, self.__camera_matrix, self.__camera_distortion)
            rvec, tvec = ret[0][0,0,:], ret[1][0].T


            R_ct    = np.array(cv.Rodrigues(rvec)[0])
            R_tc    = R_ct.T

            roll_marker, pitch_marker, yaw_marker = self.__rotationMatrixToEulerAngles(self.__RFlip@R_tc)
            yaw_marker = self.wrap(yaw_marker, -np.pi/2)
            #Pose of camera
            pos_camera = -R_tc@tvec
            roll_camera, pitch_camera, yaw_camera = self.__rotationMatrixToEulerAngles(self.__RFlip@R_tc)

            tvec = self.__RFlip @ tvec

            #self.marker_attitude_msg.data = [roll_marker, pitch_marker, yaw_marker*180/np.pi]
            self.marker_pose_msg.data = [tvec[0,0], -tvec[1,0], tvec[2,0], yaw_marker*180/np.pi]
            #self.camera_attitude_msg.data = [roll_camera, pitch_camera, yaw_camera]
            self.camera_pose_msg.data = [pos_camera[0,0], pos_camera[1,0], pos_camera[2,0], yaw_camera]


            #self.marker_attitude_publisher.publish(self.marker_attitude_msg)
            self.marker_pose_publisher.publish(self.marker_pose_msg)
            #self.camera_attitude_publisher.publish(self.camera_attitude_msg)
            self.camera_pose_publisher.publish(self.camera_pose_msg)




def main(args=None):
    rclpy.init(args=args)

    aruco = ArucoNode()

    rclpy.spin(aruco)

    aruco.destroy_node()
    rclpy.shutdown()
