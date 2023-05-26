#ObjectDetected 类说明
- 输入飞机坐标下的任意角度，输出障碍物点到飞机的距离
- 同时障碍物可以以直线(点对)的形式获取障碍物数据，该类输出数据皆以飞机坐标系（FLU）为基准，demo.py中有全局坐标系下的坐标转换


#接口说明
``` python
object_detect.py

def init(objs_dist, offset_ang,...) #初始化类，objs_dist：为目标最小间距(相邻两条线段的相邻两端最近距离) offset_ang: 激光雷达X方向（设备上箭头指向）与飞机X方向的夹角


def InputLidarData(self,data) #数据ros激光雷达数据，数据类型为 sensor_msgs/LaserScan


def GetYawObjDistance(self,yaw) #获取相对无人机坐标系（FLU）yaw角度方向的障碍物距离，yaw:为飞机FLU坐标系下X方向（X方向为0度，左正右负）角度

def GetObjMsg(self)->list: #获取相对飞机的障碍物数据(障碍物封装程线段形式)
    #线段封装[
    #         [a1.x,a1.y],[b1.x,b1.y],
    #         [a2.x,a2.y],[b2.x,b2.y],
    #           .... 
    #           ]

```

``` python
demo.py

def DronePoseCallBack(msg:Odometry): #订阅无人机在global中的位姿
    ''' 获取飞机在全局坐标系的位姿(位置和姿态),进行坐标转换'''
    position = msg.pose.pose.position

    #lines 点是相对飞机FLU坐标系的，
    # lines = obj_detect.GetObjMsg()  #获取到激光点云数据中障碍物数据，障碍物数据封装成线两个端点数据，因此lines中n个点对表示n条直线，该接口里面涉及到循环遍历，对于python而言计算会比较慢
    # if(len(lines)==0):
    #     return
    # print("lines",lines)
    # x = msg.pose.pose.orientation.x
    # y = msg.pose.pose.orientation.y
    # z = msg.pose.pose.orientation.z
    # w = msg.pose.pose.orientation.w

    # position = msg.pose.pose.position
    # yaw = math.atan2(2*(w*z + x*y),1-2*(z*z+ y *y))
    # #需要转换成global坐标系下的坐标
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
    # pass
dist_pub = rospy.Publisher("/objs_dist",Distance, queue_size=10)  #发布 离飞机前后左右障碍物的距离
objs_pub  = rospy.Publisher("/objs_line",Lines,queue_size=10) #发布障碍物提取（线的点对集）

rospy.Subscriber("/scan",LaserScan,LaserCallBack) #订阅激光雷达数据
rospy.Subscriber("/mavros/global_position/local",Odometry,DronePoseCallBack)#订阅飞机在全局坐标系下的坐标系

#下
rate = rospy.Rate(10)

ros_lines = Lines
ros_line = Line
while not rospy.is_shutdown():

    lines = obj_detect.GetObjMsg()  
    if(len(lines)==0):
        rate.sleep()
        continue

    ros_lines.lines.clear()
    for line in lines:
        ros_line.begin_p.x = line[0][0]
        ros_line.begin_p.y = line[0][1]
        ros_line.end_p.x = line[1][0]
        ros_line.end_p.y = line[1][1]
        ros_lines.lines.append(ros_line)
    objs_pub.publish(ros_lines)
    rate.sleep()


rospy.spin()



```

# 环境搭建 
（若使用的是飞思提供的虚拟机),环境已经搭建好了的
```shell
安装MAVROS
sudo apt-get install ros-melodic-mavros #ubuntu 18.04
or 
sudo apt-get install ros-neotic-mavros #ubuntu 20.04

```
## 编译源码
飞思实验室提同的镜像，源码不需要编译，若使用自己的环境，需要编译源码以及生成自定义消息文件对应的头文件
```shell
#编译 ponitcloud_to_laserscan
sudo apt-get install ros-melodic-tf2-sensor-msgs #ubuntu 18.04
or
sudo apt-get install ros-neotic-tf2-sensor-msgs #ubuntu 20.04

#编译工程
cd competition_ws/
catkin_make
```

