import rospy
import ObjectDetect
#from mavros_msgs.msg import
from nav_msgs.msg import Odometry
import sensor_msgs.msg
from offboard_pkg.msg import Lines
from offboard_pkg.msg import Line
from offboard_pkg.msg import Distance
import math
import time



obj_detect = ObjectDetect.ObjectDetected(0.5,math.pi) 

rospy.init_node("demo", anonymous=True)

dist_pub = rospy.Publisher("/objs_dist",Distance, queue_size=10)
objs_pub  = rospy.Publisher("/objs_line",Lines,queue_size=10)

def LaserCallBack(data): 
    global obj_detect
    obj_detect.InputLidarData(data) 
    dist = Distance()
    dist.front_dist = obj_detect.GetYawObjDistance(0) 
    dist.left_dist = obj_detect.GetYawObjDistance(math.pi/2) 
    dist.right_dist = obj_detect.GetYawObjDistance(-math.pi/2) 
    dist.behind_dist = obj_detect.GetYawObjDistance(math.pi) 

def DronePoseCallBack(msg): 
    '''
    
    '''
    # position = msg.pose.pose.position

    
    # lines = obj_detect.GetObjMsg()  
    # if(len(lines)==0):
    #     return

    
    # print("lines",lines)
    # x = msg.pose.pose.orientation.x
    # y = msg.pose.pose.orientation.y
    # z = msg.pose.pose.orientation.z
    # w = msg.pose.pose.orientation.w

    # position = msg.pose.pose.position
    # yaw = math.atan2(2*(w*z + x*y),1-2*(z*z+ y *y))
    
    # sin_t = math.sin(yaw)
    # cos_t = math.cos(yaw)
    # for line in lines:
    #     start_x = line[0][0]
    #     start_y = line[0][1]
    #     end_x = line[1][0]
    #     end_y = line[1][1]
    #     line[0][0] = start_x * cos_t + start_y *sin_t + position.x
    #     line[0][1] = start_y * cos_t - start_x*sin_t + position.y
    #     line[1][0] = end_x * cos_t + end_y *sin_t + position.x
    #     line[1][1] = end_y * cos_t - end_x*sin_t + position.y
    

time.sleep(4)


rospy.Subscriber("/scan",sensor_msgs.msg.LaserScan,LaserCallBack) 
rospy.Subscriber("/mavros/local_position/odom",Odometry,DronePoseCallBack)

rate = rospy.Rate(10)

ros_lines = Lines()
ros_line = Line()
while not rospy.is_shutdown():

    lines = obj_detect.GetObjMsg()  
    if(len(lines)==0):
        rate.sleep()
        continue

    ros_lines.lines=[]
    for line in lines:
        ros_line.begin_p.x = line[0][0]
        ros_line.begin_p.y = line[0][1]
        ros_line.end_p.x = line[1][0]
        ros_line.end_p.y = line[1][1]
        ros_lines.lines.append(ros_line)
    objs_pub.publish(ros_lines)
    rate.sleep()


rospy.spin()
