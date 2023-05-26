#! /usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import tf
import sys
from scipy.optimize import minimize
import numpy as np
import time
from math import atan, pi
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from quadrotor_msgs.msg import TakeoffLand

class Algorithm:
    
    def __init__(self):
        rospy.init_node("arcuo_det")
        self.debug = rospy.get_param('~is_debug')
        self.state = Odometry()

        # for Aruco
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.detect = False

        # self.camera_Matrix = np.array([[320.0, 0.0, 320.0],
        #                                [0.0, 320.0, 240.0],
        #                                [0.0, 0.0, 1.0]])
        self.camera_Matrix = np.array([[404.035450, 0., 329.188263],
                                       [0.,403.653476, 248.486988,],
                                       [0., 0., 1. ]])
        self.distortion_Matrix = np.array([0.043602, -0.064718,
                                    -0.000313, -0.000471,
                                    0.])

        self.points = []
        self.time_stamp = rospy.Time.now().to_sec()
        self.degree = -100

        # Subscriber
        pose_sub = rospy.Subscriber('/vins_fusion/imu_propagate', Odometry, self.pose_callback)

        # Publisher 
        self.camera_pub = rospy.Publisher('/aruco_det/vis', Image, queue_size=1)
        self.detect_state_pub = rospy.Publisher('/aruco_det/state', Bool, queue_size=1)
        self.detect_result = rospy.Publisher('/aruco_det/target_loc', PoseStamped, queue_size=1)
    
    def pose_callback(self, msg):
        self.state = msg
    
    def camera_callback(self, msg):
        time_received = rospy.Time.now()
        cv_image = msg
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.arucoDict,
            parameters=self.arucoParams)
        
        flag = Bool()

        if len(corners) > 0:
            print('ids: ', ids)
            if ids[0][0] == 19:
                length = 0.14
            else:
                length = 0.018
            self.detect_state_pub.publish(True)
            draw_det_marker_img = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            tmp = np.array([[-960, -540],[-960, -540],[-960, -540],[-960, -540]])

            # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners+tmp, length, self.camera_Matrix,np.array([]))
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, length, self.camera_Matrix, self.distortion_Matrix)

            for i in range(rvec.shape[0]):
                draw_det_marker_img = cv2.aruco.drawAxis(draw_det_marker_img, self.camera_Matrix, self.distortion_Matrix,
                                                         rvec[i, :, :], tvec[i, :, :], 0.03)
                
                target_loc = PoseStamped()
                target_loc.header.stamp = time_received
                # print(tvec)
                # print(tvec[0])
                # print(tvec[0][0])
                target_loc.pose.position.x = tvec[0][0][0]
                target_loc.pose.position.y = tvec[0][0][1]
                target_loc.pose.position.z = tvec[0][0][2]
                # target_loc.pose.orientation.z = self.degree

                # pose_now.sight_angle[0] = atan(tvecs[i][0] / tvecs[i][2]);
                # pose_now.sight_angle[1] = atan(tvecs[i][1] / tvecs[i][2]);

                target_loc.pose.orientation.x = atan(tvec[0][0][0] / tvec[0][0][2])
                target_loc.pose.orientation.y = atan(tvec[0][0][1] / tvec[0][0][2])

                self.detect_result.publish(target_loc)

                flag.data = True
                self.detect_state_pub.publish(flag)

                break
            
        else:
            flag.data = False
            self.detect_state_pub.publish(flag)


if __name__ == "__main__":

    main_algorithm = Algorithm()
    rate = rospy.Rate(20)
    cap = cv2.VideoCapture(0)
    while not rospy.is_shutdown():
        retval, frame = cap.read()
        main_algorithm.camera_callback(frame)
        if retval:
            # print(retval)
            # cv2.imshow('Live', frame)
            main_algorithm.camera_callback(frame)
        if cv2.waitKey(5) >= 0:
            break
        rate.sleep()