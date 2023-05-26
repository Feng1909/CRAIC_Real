#include <ros/ros.h>
#include "ros/package.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ActuatorControl.h>
#include <vector>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "yaml-cpp/yaml.h"
#include <iostream>
//#include <zbar.h>

//global params
int achieve_counts = 20;
double achieve_threshold = 0.2;

ros::Time last_request;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_goal;
mavros_msgs::SetMode auto_land_setmode;
std::vector<geometry_msgs::PoseStamped> goals;
geometry_msgs::PoseStamped four_goal[4];  
// A* 
//float astar_path_pos_set[3]={0,0,0};
//int astar_pos_cmd_count = 0;
 
int number[6];      // 六个点位对应手写数字的值
bool PlaceToLand;   // right = 0 left = 1
int waypoint0,waypoint1,waypoint2;    // 依次前往的三个目标点
mavros_msgs::ActuatorControl claw_control_mix;

enum State {
  STATE_OFFFBOARD,
  STATE_TAKEOFF,
  STATE_GOALS,
  STATE_POINT0,
  STATE_POINT1,
  STATE_POINT2,
  STATE_UNLOAD,
  STATE_POINTLAND,
  STATE_LAND,
  STATE_FINAL
  
};

void loadYAML();
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
void image_cb(const sensor_msgs::ImageConstPtr &msg);
void parse_qr_code(const std::string& data);
bool judgeAchieve();
//void drone_control_command_cb(const offboard_pkg::ControlCommand::ConstPtr& msg); 
//
//void drone_control_command_cb(const offboard_pkg::ControlCommand::ConstPtr& msg)
//{
//    astar_path_pos_set[0] = msg->Reference_State.position_ref[0];
//    astar_path_pos_set[1] = msg->Reference_State.position_ref[1];
//    astar_path_pos_set[2] = msg->Reference_State.position_ref[2];
//
//    astar_pos_cmd_count = astar_pos_cmd_count + 1;
//}


void loadYAML() {
    std::string path = ros::package::getPath("example");
    path += "/param/points.yaml";
    printf("yaml path:%s\n", path.c_str());
    YAML::Node config = YAML::LoadFile(path);
    int num = config["num"].as<int>();
    std::cout << "target pose number :" << num << std::endl;
    std::string str = "point";
    for (int i = 1; i <= num; i++) {
        std::string now = str + std::to_string(i);
        auto x = config[now]["x"].as<double>();
        auto y = config[now]["y"].as<double>();
        auto z = config[now]["z"].as<double>();
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = x;
        temp.pose.position.y = y;
        temp.pose.position.z = z;
        goals.push_back(temp);
        ROS_INFO("\033[32mgoals x:%lf y:%lf z:%lf loaded", x, y, z);
    }
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg){
    current_pose = *msg;
}


// QR code
cv::Mat image;
void image_cb(const sensor_msgs::ImageConstPtr &msg)
{
    static cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
    try
    {
        image = cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    cv::Mat bbox, rectifiedImage;
    std::string data = qrDecoder.detectAndDecode(image, bbox, rectifiedImage);
    
    if (data.length() > 0)
    {
        std::cout << "Decoded QR Code: " << data << std::endl;
        ROS_INFO("Decoded QR Code: %s", data.c_str()); // 输出识别到的二维码内容
        parse_qr_code(data);
    }
    cv::imshow("camera", image);
    cv::waitKey(10);

}

// 二维码解析  1,2,3,right
void parse_qr_code(const std::string& data) {
    // 如果二维码数据为空，则直接返回
    if (data.empty()) {
        ROS_WARN("Empty QR code data.");
        return;
    }
    
    // 将字符串按逗号分割，获取多个子串
    std::vector<std::string> substrings;
    std::istringstream iss(data);
    std::string substring;
    while (std::getline(iss, substring, ',')) {
        substrings.push_back(substring);
    }

    // 根据不同的子串内容，设置不同的全局变量
    for (int i = 0; i < substrings.size(); ++i) {
        if (substrings[i] && i == 0) {
            waypoint0 = substrings[i];
        } else if (substrings[i]  && i == 1) {
            waypoint1 = substrings[i];
        } else if (substrings[i] && i == 2) {
            waypoint2= substrings[i];
        } else if (substrings[i] == "left" ) {
            PlaceToLand = true;
            ROS_INFO("left");
        } else if (substrings[i] == "right") {
            PlaceToLand = false;
            ROS_INFO("right");
        }
    }
    ROS_INFO("Point0:%d Point1:%d Point2:%d",waypoint0,waypoint1,waypoint2);
}

// 判断是否抵达目标点
bool judgeAchieve(){
    static int count = 0;
    if (abs(current_goal.pose.position.x-current_pose.pose.position.x)<achieve_threshold && 
    	abs(current_goal.pose.position.y-current_pose.pose.position.y)<achieve_threshold &&
	abs(current_goal.pose.position.z-current_pose.pose.position.z)<achieve_threshold){
        count++;
        ROS_INFO("Point0:%d Point1:%d Point2:%d",waypoint0,waypoint1,waypoint2);
        if (count > achieve_counts) return !(count=0);
    }
    else count = 0;
    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fly_with_qr");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber camera_sub = nh.subscribe<sensor_msgs::Image>("iris/usb_cam/image_raw", 1, image_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");	
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::Publisher claw_pub_mix = nh.advertise<mavros_msgs::ActuatorControl>("/mavros/actuator_control", 1);
    //ros::Subscriber drone_control_command_sub = nh.subscribe<offboard_pkg::ControlCommand>("/drone/control_command", 100, drone_control_command_cb);
    
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);
    //LOAD Waypoint0
    loadYAML();
     auto goal_it = goals.begin();   //获取目标点位信息
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    last_request = ros::Time::now();
    claw_control_mix.group_mix = 2;//-1是最小转动角度 1是最大角度
     //-1是最小转动角度 1是最大角度
	for (int i = 0; i < 8; ++i) {
		claw_control_mix.controls[i] = -1;
	}

    State currentState = STATE_OFFFBOARD;
    // 投放时无人机的高度不得高于 0.8m。
    // 无人机的飞行高度不得低于 1.2m
    while(ros::ok()){
        switch (currentState) {
            case STATE_OFFFBOARD:
                ROS_INFO("waiting for take off");
                if (current_state.mode != "OFFBOARD") {
                    if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
                         {
                             ROS_INFO("Offboard enabled");
                         }
                        last_request = ros::Time::now();
                        }
                         local_pos_pub.publish(pose);
                    } else {
                        currentState = STATE_TAKEOFF;
                    }
                break;
            case STATE_TAKEOFF:
                ROS_INFO("going to take off");
                if (!current_state.armed) {
                    if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                            ROS_INFO("Vehicle armed");
        	    			claw_pub_mix.publish(claw_control_mix);
                        }
                        last_request = ros::Time::now();
                    }
                    local_pos_pub.publish(pose);
                } else {
                   currentState = STATE_GOALS;
                    //初始化舵机角度
        	    	claw_pub_mix.publish(claw_control_mix);
                }
                break;ik
            case STATE_GOALS:
              if (goal_it != goals.end()) {
                    current_goal = *goal_it;
                    local_pos_pub.publish(current_goal);
                    if (judgeAchieve()) {
                        goal_it++;
                        ROS_INFO("Fly to X:%lf Y:%lf Z:%lf",current_goal.pose.position.x,current_goal.pose.position.y,current_goal.pose.position.z);
                    }
              } else {
                // 在这里完成三个点位的匹配
                for(int i = 0, i <= 5; i++){
                    if(waypoint0 == number[i])      // waypoint0 = 3 number[1] = 3 代表1号位置为3
                    {
                        four_goal[0].pose.position.x =  
                        four_goal[0].pose.position.y = 
                        four_goal[0].pose.position.z = 
                        continue;
                    }
                    if(waypoint1 == number[i])
                    {

                       continue; 
                    }
                    if(waypoint2 == number[i]i)
                    {

                        continue;
                    }

                }
                int number[6]; 
                waypoint0 = 3;
                currentState = STATE_POINT0;    // 最后一个点位是第六个点 飞完后前往第一个投放地点
              }
              break;
            case STATE_UNLOAD:
                ROS_INFO("down to 0.5 unload up to 1.2 fly");z

            case STATE_POINT0:
                // fly to point 1 and 
                pose.pose.position.x = 0.3;
                pose.pose.position.y = 0.3;
                pose.pose.position.z = 0.3;
                ROS_INFO("go to (1,0,0.3)");
                local_pos_pub.publish(pose);
                if(pose_uav.pose.position.z < 0.4){
                    for (int i = 0; i < 8; ++i) {
                        //转动舵机 放下货物
                        claw_control_mix.controls[i] = 1;
                        claw_pub_mix.publish(claw_control_mix);
                    }
                    currentState =  STATE_POINT2;
                }
                break;
            case STATE_POINT1:
                // fly to point 1 and 
                pose.pose.position.x = 0.3;
                pose.pose.position.y = 0.3;
                pose.pose.position.z = 0.3;
                ROS_INFO("go to (1,0,0.3)");
                local_pos_pub.publish(pose);
                if(pose_uav.pose.position.z < 0.4){
                    for (int i = 0; i < 8; ++i) {
                        //转动舵机 放下货物
                        claw_control_mix.controls[i] = 1;
                        claw_pub_mix.publish(claw_control_mix);
                    }
                    currentState =  STATE_POINT2;
                }
                break;
            case STATE_POINT2:
                   // fly to point 1 and 
                pose.pose.position.x = 0.3;
                pose.pose.position.y = 0.3;
                pose.pose.position.z = 0.3;
                ROS_INFO("go to (1,0,0.3)");
                local_pos_pub.publish(pose);
                if(pose_uav.pose.position.z < 0.4){
                    for (int i = 0; i < 8; ++i) {
                        //转动舵机 放下货物
                        claw_control_mix.controls[i] = 1;
                        claw_pub_mix.publish(claw_control_mix);
                    }
                    currentState =  STATE_LAND;
                }
                break;
            case STATE_POINTLAND:
                break;    


            case STATE_LAND:
                ROS_INFO("land");
                if (current_state.mode != "AUTO.LAND") {
                    if (ros::Time::now() - last_request > ros::Duration(1.0)) {
                        mavros_msgs::SetMode land_set_mode;
                        land_set_mode.request.custom_mode = "AUTO.LAND";
                        if (set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent) {
                            ROS_INFO("Vehicle landed");
                        }
                        last_request = ros::Time::now();
                    }
                } else {
                    currentState = STATE_FINAL;
                }
                break;
            case STATE_FINAL:
                return 0;
             }
                ros::spinOnce();
                rate.sleep();
            }
        return 0;
}


