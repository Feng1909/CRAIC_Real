#! /usr/bin/env python
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import tf
import sys
from scipy.optimize import minimize
import numpy as np
from pyquaternion import Quaternion
import time
from math import atan2, pi

class Algorithm:
    
    def __init__(self):
        rospy.init_node("arcuo_det")
        self.debug = rospy.get_param('~is_debug')
        self.state = {'pose': {}, 'ori':{}, 'twist':{}}
        self.state['pose'] = PoseStamped()
        self.state['twist'] = TwistStamped()

        # for Aruco
        self.bridge = CvBridge()
        self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters_create()
        self.detect = False
        self.camera_Matrix = np.array([[320.0, 0.0, 320.0],
                                       [0.0, 320.0, 240.0],
                                       [0.0, 0.0, 1.0]])
        
        self.points = []
        self.time_stamp = rospy.Time.now().to_sec()
        self.degree = -100


        # Subscriber
        pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        vel_sub = rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.vel_callback)
        camera_sub = rospy.Subscriber('/rflysim/sensor1/img_rgb', Image, self.camera_callback)

        # Publisher 
        self.camera_pub = rospy.Publisher('aruco_det/vis', Image, queue_size=1)
        self.detect_state_pub = rospy.Publisher('aruco_det/state', Bool, queue_size=1)
        self.detect_result = rospy.Publisher('aruco_det/target_loc', PoseStamped, queue_size=1)
    
    def pose_callback(self, msg):
        self.state['pose'] = msg

    def vel_callback(self, msg):
        self.state['twist'] = msg
    
    def camera_callback(self, msg):
        time_received = rospy.Time.now()
        t_1 = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, self.arucoDict,
            parameters=self.arucoParams)
        if len(corners) > 0:
            self.detect_state_pub.publish(True)
            draw_det_marker_img = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.19, self.camera_Matrix,np.array([]))

            for i in range(rvec.shape[0]):
                draw_det_marker_img = cv2.aruco.drawAxis(draw_det_marker_img, self.camera_Matrix, np.array([]),
                                                         rvec[i, :, :], tvec[i, :, :], 0.03)
                
                
                code_matrix = self.quat_to_pos_matrix_hm(tvec[0][0][0], tvec[0][0][1], tvec[0][0][2], 0, 0, 0, 1)
                camera_matrix = self.quat_to_pos_matrix_hm(0, 0, 0, 0.7071068, -0.7071068, 0, 0)
                drone_matrix = self.quat_to_pos_matrix_hm(self.state['pose'].pose.position.x, self.state['pose'].pose.position.y, self.state['pose'].pose.position.z, self.state['pose'].pose.orientation.x, self.state['pose'].pose.orientation.y, self.state['pose'].pose.orientation.z, self.state['pose'].pose.orientation.w)
                global_matrix = drone_matrix*camera_matrix*code_matrix
                
                self.points.append([global_matrix[0,3], global_matrix[1,3], global_matrix[2,3]])
                if len(self.points) >= 102:
                    self.points = self.points[1:]
                if len(self.points) > 100:
                    radius = 0.7
                    circle_center = self.fit_circle(self.points[::3], radius)

                    self.degree = atan2(global_matrix[1,3]-circle_center[1], global_matrix[0,3]-circle_center[0]) \
                                    + (time.time()-t_1)*0.7
                    if self.degree > pi:
                        self.degree -= 2*pi
                    if self.degree < -pi:
                        self.degree += 2*pi
                    
                    self.time_stamp = rospy.Time.now().to_sec()
                    
                    target_loc = PoseStamped()
                    target_loc.header.stamp = time_received
                    target_loc.pose.position.x = circle_center[0]
                    target_loc.pose.position.y = circle_center[1]
                    target_loc.pose.position.z = circle_center[2]
                    target_loc.pose.orientation.z = self.degree
                    self.detect_result.publish(target_loc)
                  
            if self.debug:
                image_message = self.bridge.cv2_to_imgmsg(draw_det_marker_img, encoding="bgr8")
                self.camera_pub.publish(image_message)

        else:
            self.detect_state_pub.publish(False)
    
    def error_function(self, params, points, radius):
        a, b, c = params
        error = 0
        for point in points:
            x, y, z = point
            error += ((x - a)**2 + (y - b)**2 + (z - c)**2 - radius**2)**2
        return error

    def fit_circle(self, points, radius):
        initial_guess = np.array([0,8,0.5])
        result = minimize(self.error_function, initial_guess, args=(points, radius))
        return result.x  # 返回最优解

    def quat_to_pos_matrix_hm(self, p_x, p_y, p_z, x, y, z, w):
        # 创建位姿矩阵，写入位置
        T = np.matrix([[0, 0, 0, p_x], [0, 0, 0, p_y], [0, 0, 0, p_z], [0, 0, 0, 1]])
        T[0, 0] = 1 - 2 * pow(y, 2) - 2 * pow(z, 2)
        T[0, 1] = 2 * (x * y - w * z)
        T[0, 2] = 2 * (x * z + w * y)

        T[1, 0] = 2 * (x * y + w * z)
        T[1, 1] = 1 - 2 * pow(x, 2) - 2 * pow(z, 2)
        T[1, 2] = 2 * (y * z - w * x)

        T[2, 0] = 2 * (x * z - w * y)
        T[2, 1] = 2 * (y * z + w * x)
        T[2, 2] = 1 - 2 * pow(x, 2) - 2 * pow(y, 2)
        return T


if __name__ == "__main__":

    main_algorithm = Algorithm()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()