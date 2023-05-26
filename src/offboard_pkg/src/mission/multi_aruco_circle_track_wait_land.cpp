/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include "ros/package.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <offboard_pkg/DetectionInfo.h>
#include <offboard_pkg/ArucoInfo.h>
#include <vector>
#include "yaml-cpp/yaml.h"

#include "math_utils.h"

//global params
double fly_altitude = 1.0;
int achieve_counts = 20;
double achieve_threshold = 0.2;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_goal;
std::vector<geometry_msgs::PoseStamped> goals;

double cur_position[3]={0,0,0};
double position_det[3]={0,0,0};
double circle_center_pos_enu[3]={0,0,0};
double circle_center_pos_enu_store[3]={0,0,0};
double circle_center_pos_enu_store_avr[3]={0,0,0};

double cur_position_tempstore[3]={0,0,0};

double aruco_pos_enu[3]={0,0,0};

double aruco_preland_pos_enu[3]={0,0,0};

bool detected_1;
bool IF_SIMULATION;

int circle_center_pos_enu_count = 0;

float current_roll;
float current_pitch;
float current_yaw;
float THRUST;

float yawratefromsim;

// int state_flag = 0;
int state_flag = 0;

int num_regain = 0;
int num_lost = 0;
bool is_detected;
int VISION_THRES = 13;

int state5_count = 0;


void loadYAML() {
    std::string path = ros::package::getPath("fly_demo");
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
        geometry_msgs::PoseStamped temp;
        temp.pose.position.x = x;
        temp.pose.position.y = y;
        temp.pose.position.z = fly_altitude;
        goals.push_back(temp);
        ROS_INFO("\033[32mgoals x:%lf y:%lf loaded", x, y);
    }
}


void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
            current_pose = *msg;
            // std::cout<<current_pose<<std::endl;
            cur_position[0] = msg->pose.position.x;
            cur_position[1] = msg->pose.position.y;
            cur_position[2] = msg->pose.position.z;
    
            //排序是 roll,pitch,yaw
            Eigen::Quaterniond q_fcu = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);

            Eigen::Vector3d euler_fcu = quaternion_to_euler(q_fcu);
            current_roll = euler_fcu[0];
            current_pitch = euler_fcu[1];
            current_yaw = euler_fcu[2];

        }

bool judgeAchieve(){
    static int count = 0;
    std::cout<<abs(current_goal.pose.position.x-current_pose.pose.position.x)<<" "<<abs(current_goal.pose.position.y-current_pose.pose.position.y)<<" "<<abs(current_goal.pose.position.z-current_pose.pose.position.z)<<std::endl;
    if (abs(current_goal.pose.position.x-current_pose.pose.position.x)<achieve_threshold &&
            abs(current_goal.pose.position.y-current_pose.pose.position.y)<achieve_threshold &&
            abs(current_goal.pose.position.z-current_pose.pose.position.z)<achieve_threshold)
    {
      std::cout<<"coming"<<std::endl;
        count++;
        if (count > achieve_counts) return !(count=0);
    }
    else {cout<<"error"<<std::endl;count = 0;}

    std::cout<<"count: "<<count<<std::endl;

    return false;
}

void multi_aruco_det_cb(const offboard_pkg::DetectionInfo::ConstPtr &msg)
        {   
            detected_1=msg->detected;
            position_det[0] = msg->position[0];
            position_det[1] = msg->position[1];
            position_det[2] = msg->position[2];

            if(detected_1)
            {
               num_regain++;
               num_lost = 0;
            }else
            {
               num_regain = 0;
               num_lost++;
            }

            // 当连续一段时间无法检测到目标时，认定目标丢失
            if(num_lost > VISION_THRES)
            {
               is_detected = false;
               //存储无人机丢失二维码时的当前位置，后面作为悬停位置
               //之所以后来不放在aruco_det_cb回调函数里面是防止最开始的时候就没有检测到二维码的时候这个初始的位置记录值可能和无人机当前位置偏差较远
               if(num_lost == VISION_THRES+1)
               {
                 //cur_position_tempstore[0] = cur_position[0];
                 //cur_position_tempstore[1] = cur_position[1];
                 //cur_position_tempstore[2] = cur_position[2];
               }
            }

            // 当连续一段时间检测到目标时，认定目标得到
            if(num_regain > VISION_THRES)
            {
               is_detected = true;
            }


        }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "aruco_land_static");
    ros::NodeHandle nh;
    ros::NodeHandle nh1("~");

    float TRAS;
    

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher setpoint_raw_local_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);


    ros::Subscriber multi_aruco_det_sub = nh.subscribe<offboard_pkg::DetectionInfo>("/drone/object_detection/multi_aruco_det", 10, multi_aruco_det_cb);


    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    nh1.param<float>("Tras", TRAS, 31.4);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    loadYAML();

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
       ros::spinOnce();
       rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

//真机上应该不用再先发点了，因为手动切offboard嘛，等程序先运行起来已经发点了，再手动切offboard就够了。
    //send a few setpoints before starting
   for(int i = 100; ros::ok() && i > 0; --i){
      local_pos_pub.publish(pose);
       ros::spinOnce();
        rate.sleep();
   }

    //mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool disarm_cmd;
    //arm_cmd.request.value = true;
    disarm_cmd.request.value = false; //这样就是上锁

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";


    ros::Time last_time = ros::Time::now();

    ros::Time state1_first_time;

    ros::Time time_snap1;
    ros::Time time_snap11;

    ros::Time aruco_preland_enupos_record_time;

    auto goal_it = goals.begin();

    while(ros::ok()){

    //参考自：Prometheus/master/Modules/control/include/command_to_mavros.h

    geometry_msgs::PoseStamped pos_setpoint;

    if(cur_position[2] < 0.3)
    {
       last_time = ros::Time::now();
    }

    //fly to goal

    if (state_flag == 0) 
    {
      if (goal_it != goals.end()) {
          current_goal = *goal_it;
          local_pos_pub.publish(current_goal);
          if (judgeAchieve()) goal_it++;
          ROS_INFO("Fly to X:%lf Y:%lf",current_goal.pose.position.x,current_goal.pose.position.y);
      }
      else {
        state_flag = 1;
        time_snap11 = ros::Time::now();
      }
    }

    if(state_flag == 1)
    {
    ROS_INFO("state 1");
       //如果丢失二维码，则进入悬停状态
       if(is_detected == false)
        {
           state_flag = 5;
           continue;
        }    
    //坐标系确认好
    aruco_pos_enu[0] = cur_position[0] - position_det[1];
    aruco_pos_enu[1] = cur_position[1] - position_det[0]; 
    aruco_pos_enu[2] = cur_position[2] - position_det[2]; 
    //printf("circle_center_pos_enu[0] is %f\n", circle_center_pos_enu[0] + 2.2);
    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

    pos_setpoint.pose.position.x = aruco_pos_enu[0];
    pos_setpoint.pose.position.y = aruco_pos_enu[1];
    //pos_setpoint.pose.position.z = 0;
    pos_setpoint.pose.position.z = aruco_pos_enu[2] + 0.3;
    // pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);

    if((ros::Time::now() - time_snap11 > ros::Duration(5.0))&&(abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05))
     {
       //记录下当前的时间戳和二维码的ENU坐标
       aruco_preland_enupos_record_time = ros::Time::now();
       aruco_preland_pos_enu[0] = aruco_pos_enu[0];
       aruco_preland_pos_enu[1] = aruco_pos_enu[1];
       aruco_preland_pos_enu[2] = aruco_pos_enu[2];

       state_flag = 8;
     }
  
    }
    
    
    //切land
    if(state_flag == 2)
    {
    printf("aruco_pos_enu[0] is %f\n", aruco_pos_enu[0]);
    printf("aruco_pos_enu[1] is %f\n", aruco_pos_enu[1]);
    printf("aruco_pos_enu[2] is %f\n", aruco_pos_enu[2]);

 
         set_mode_client.call(land_set_mode);
         if(land_set_mode.response.mode_sent)
           {
             ROS_INFO("land enabled");          
           }  
    }


    //没有检测到二维码时保持悬停
    if(state_flag == 5)
    {
    ROS_INFO("state 5");
    state5_count = state5_count + 1;
    if(state5_count == 1)
    {
       //记录刚丢失二维码时的无人机位置的值，作为悬停的位置点。
       //之所以不放在aruco_det_cb回调函数里面是防止最开始的时候就没有检测到二维码的时候这个初始的位置记录值可能和无人机当前位置偏差较远
       cur_position_tempstore[0] = cur_position[0];
       cur_position_tempstore[1] = cur_position[1];
       cur_position_tempstore[2] = cur_position[2];
    }
    //这个悬停比较严谨
    pos_setpoint.pose.position.x = cur_position_tempstore[0];
    pos_setpoint.pose.position.y = cur_position_tempstore[1];
    //pos_setpoint.pose.position.z = 0;
    pos_setpoint.pose.position.z = cur_position_tempstore[2];
    // pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);


    if(is_detected == true)
     {
       state_flag = 1;
       state5_count = 0;
     }

    }

    //通过service进行上锁。
    if(state_flag == 7)
    {
      //此处切换会manual模式是因为:PX4默认在offboard模式且有控制的情况下没法上锁,所以先切为manual再上锁
       //前面加上判断可以避免反复一直切模式
         if(current_state.mode == "OFFBOARD")
         {
         set_mode_client.call(manual_set_mode);
         if(manual_set_mode.response.mode_sent)
           {
             ROS_INFO("manual enabled");          
           }  
         }

            //前面加上判断可以避免反复一直切上锁
            //if(current_state.armed){
            if((current_state.armed)&&(current_state.mode == "MANUAL")){ 
                if( arming_client.call(disarm_cmd) &&
                    disarm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
             }
    }


    //提前飞到二维码上方
    if(state_flag == 8)
    {
    ROS_INFO("state 8");
    // pos_setpoint.type_mask = 0b100111111000;  // 100 111 111 000  xyz + yaw
    // pos_setpoint.coordinate_frame = 1;


    pos_setpoint.pose.position.x = aruco_preland_pos_enu[0];
    pos_setpoint.pose.position.y = aruco_preland_pos_enu[1];
    pos_setpoint.pose.position.z = aruco_preland_pos_enu[2] + 0.15;
    // pos_setpoint.yaw = 0;
    setpoint_raw_local_pub.publish(pos_setpoint);


    //if(ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(20.9))
    //if(ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(32.9))
    //if((ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(32))&&(abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05)&&(is_detected == true))
    // if((ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(TRAS - 1))&&(abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05)&&(is_detected == true))
    if((ros::Time::now() - aruco_preland_enupos_record_time > ros::Duration(TRAS - 1))&&(abs(position_det[0]) < 0.05)&&(abs(position_det[1]) < 0.05)&&(num_regain>2))
    {
       state_flag = 7;
    }

    }


    ros::spinOnce();
    rate.sleep();
    }

    return 0;
}


