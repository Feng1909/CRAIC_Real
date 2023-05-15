#! /usr/bin/env python3
#coding=utf-8

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int32
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from quadrotor_msgs.msg import TakeoffLand
from threading import Thread
import os

class run_ego_planner(Thread):
    def __init__(self):
        super().__init__()

    def run(self):  # 固定名字run ！！！必须用固定名
        os.system('roslaunch ego_planner single_run_in_exp.launch')

class Algorithm:
    
    def __init__(self):
        rospy.init_node("state_machine")
        self.debug = rospy.get_param('~is_debug')

        self.state_machine = Int32()
        self.state_machine.data = 0
        # 0: take off
        # 1: ego planner
        # 2: fast perching
        # 3: land and disarm

        self.aruco_detected = False
        self.ego_planner = run_ego_planner()
        
        self.set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.land_set_mode = SetModeRequest()
        self.land_set_mode.custom_mode = 'AUTO.LAND'


        self.arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)  
        self.dis_arm_cmd = CommandBoolRequest()
        self.dis_arm_cmd.value = False

        self.takeoff_land = rospy.Publisher('/px4ctrl/takeoff_land', TakeoffLand, queue_size=1)


        # Subscriber
        pose_sub = rospy.Subscriber('/vins_fusion/imu_propagate', Odometry, self.pose_callback)
        target_sub = rospy.Subscriber('/aruco_det/target_loc', PoseStamped, self.target_callback)
# rostopic pub -1  /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
        # Publisher 
        self.state_machin_pub = rospy.Publisher('/state_machine', Int32, queue_size=1)
    
    def pose_callback(self, msg):
        self.state = msg
        if self.state_machine.data == 0:
            land = TakeoffLand()
            land.takeoff_land_cmd = 1
            self.takeoff_land.publish(land)
            if self.state.pose.pose.position.z >= 0.8:
                self.state_machine.data = 1
                self.ego_planner.start()
                print('launch ego planner')
                return
        
        if self.state_machine.data == 1 and self.aruco_detected == True:
            # self.state_machine.data = 2
            print('launch fast perching')
            return
        
        if (self.state_machine.data == 2 and self.state.pose.pose.position.z <= 0.6) or self.state_machine.data == 3:
            print('landing')
            self.state_machine.data = 3
            if(self.set_mode_client.call(self.land_set_mode).mode_sent == True):
                rospy.loginfo("land enabled")
            if(self.arming_client.call(self.dis_arm_cmd).success == True):
                rospy.loginfo("Vehicle disarmed")
            land = TakeoffLand()
            land.takeoff_land_cmd = 2
            self.takeoff_land.publish(land)
            return
    
    def target_callback(self, msg):
        print('aruco detected')
        self.aruco_detected = True

    def send_msgs(self):
        self.state_machin_pub.publish(self.state_machine.data)


if __name__ == "__main__":

    main_algorithm = Algorithm()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        main_algorithm.send_msgs()
        rate.sleep()