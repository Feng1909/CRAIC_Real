#ifndef CORE_FLY_HPP
#define CORE_FLY_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>

#include "quadrotor_msgs/PositionCommand.h"

namespace ns_core_fly {

class CORE_FLY {

public:
    CORE_FLY(ros::NodeHandle &nh);
    void runAlgorithm();
    geometry_msgs::PoseStamped get_goal_pose();
    void update_state(nav_msgs::Odometry msgs);

private:
    ros::NodeHandle &nh_;
    
    geometry_msgs::PoseStamped pose;
    nav_msgs::Odometry state;
    int index;
};

}

#endif