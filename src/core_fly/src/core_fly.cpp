#include "core_fly.hpp"

namespace ns_core_fly {
double points[100][2];
double cha_points[100][2];
int NUM=0, cha_NUM=0;

CORE_FLY::CORE_FLY(ros::NodeHandle &nh):nh_(nh) {
    index = 0;
    pose.pose.position.x = points[index][0];
    pose.pose.position.y = points[index][1];
    pose.pose.position.z = 0.7;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 0.0;

    freopen("path.txt","r", stdin);
    std::cout<<"opening"<<std::endl;
    double a, b;
    while(std::cin>>a){
        std::cout<<"a: "<<a<<std::endl;
        if(a <= -999) {
            break;
        }
        std::cin>>b;
        std::cout<<"a: "<<a<<" b: "<<b<<std::endl;
        points[NUM][0] = a;
        points[NUM][1] = b;
        NUM++;
    }
    std::cout<<"finish load path"<<std::endl;
    std::cin>>a;
    freopen("path.txt","r", stdin);
    bool flag = false;
    while(std::cin>>a){
        if (a > -999 && flag == false) {
            continue;
        }
        else
        if (flag == false){
            flag = true;
            continue;
        }
        std::cin>>b;
        std::cout<<"a: "<<a<<" b: "<<b<<std::endl;
        cha_points[cha_NUM][0] = a;
        cha_points[cha_NUM][1] = b;
        cha_NUM++;
    }
    std::cout<<"finish load change path"<<std::endl;
}

void CORE_FLY::runAlgorithm() {

}

geometry_msgs::PoseStamped CORE_FLY::get_goal_pose() {
    return pose;
}

void CORE_FLY::update_state(nav_msgs::Odometry msg) {
    state = msg;
    if (hypot(state.pose.pose.position.x-cha_points[index][0],
                 state.pose.pose.position.y-cha_points[index][1]) <= 0.1 && abs(state.pose.pose.position.z - 0.7)<=0.1)
    index += 1;
    if (index > 1) 
        index = 1;
    // std::cout<<"index: "<<index<<std::endl;
    pose.pose.position.x = points[index][0];
    pose.pose.position.y = points[index][1];
    
}

}