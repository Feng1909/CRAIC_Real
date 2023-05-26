#include <ros/ros.h>
#include "ros/package.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include "yaml-cpp/yaml.h"


//global params
double fly_altitude = 1.0;
int achieve_counts = 100;
double achieve_threshold = 0.1;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_goal;
mavros_msgs::SetMode auto_land_setmode;
std::vector<geometry_msgs::PoseStamped> goals;

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

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

bool judgeAchieve(){
    static int count = 0;
        std::cout<<abs(current_goal.pose.position.x-current_pose.pose.position.x)<<" "<<abs(current_goal.pose.position.y-current_pose.pose.position.y)<<" "<<abs(current_goal.pose.position.z-current_pose.pose.position.z)<<std::endl;

    if (abs(current_goal.pose.position.x-current_pose.pose.position.x)<achieve_threshold &&
            abs(current_goal.pose.position.y-current_pose.pose.position.y)<achieve_threshold &&
            abs(current_goal.pose.position.z-current_pose.pose.position.z)<achieve_threshold)
    {
        count++;
        if (count > achieve_counts) return !(count=0);
    }
    else count = 0;

    return false;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "fly_demo_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

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

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    auto goal_it = goals.begin();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
//            if( set_mode_client.call(offb_set_mode) &&
//                offb_set_mode.response.mode_sent){
//                ROS_INFO("Offboard enabled");
//            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        //fly to goal
        if (goal_it != goals.end()) {
            current_goal = *goal_it;
            local_pos_pub.publish(current_goal);
            if (judgeAchieve()) goal_it++;
            ROS_INFO("Fly to X:%lf Y:%lf",current_goal.pose.position.x,current_goal.pose.position.y);
        }
        else {
            ROS_INFO("Flght finished");
            //land
            static ros::Time last_request = ros::Time::now();
            if (current_state.mode != "AUTO.LAND" &&
                (ros::Time::now() - last_request > ros::Duration(1.0))) {
                if (set_mode_client.call(auto_land_setmode) &&
                    auto_land_setmode.response.mode_sent) {
                    ROS_INFO("AUTO.LAND");
                    ros::shutdown();
                }
                last_request = ros::Time::now();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

