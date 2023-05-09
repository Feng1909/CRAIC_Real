#ifndef FAST_PERCHING_HPP
#define FAST_PERCHING_HPP

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <ctime>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <Eigen/Core>
#include <traj_opt/traj_opt.h>

#include "quadrotor_msgs/PositionCommand.h"

namespace ns_fast_perching {

class FAST_PERCHING {

public:
    FAST_PERCHING(ros::NodeHandle &nh);
    void runAlgorithm();
    void set_target(geometry_msgs::PoseStamped msg);
    void set_drone_pose(geometry_msgs::PoseStamped msg);
    quadrotor_msgs::PositionCommand get_next_point();
    // geometry_msgs::PoseStamped get_goal_pose();
    // void update_state(geometry_msgs::PoseStamped msgs);

private:
    ros::NodeHandle &nh_;
    
    geometry_msgs::PoseStamped target;
    geometry_msgs::PoseStamped drone_pose;
    std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
    std::shared_ptr<vis_utils::VisUtils> visPtr_;
    double velocity_omega;
    quadrotor_msgs::PositionCommand next_point;
};

}

#endif