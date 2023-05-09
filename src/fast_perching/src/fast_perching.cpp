#include "fast_perching.hpp"

namespace ns_fast_perching {

FAST_PERCHING::FAST_PERCHING(ros::NodeHandle &nh):nh_(nh) {
    nh_.getParam("velocity_omega", velocity_omega);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh_);
    visPtr_ = std::make_shared<vis_utils::VisUtils>(nh_);
}

void FAST_PERCHING::set_target(geometry_msgs::PoseStamped msg) {
    target = msg;
}

void FAST_PERCHING::set_drone_pose(geometry_msgs::PoseStamped msg) {
    drone_pose = msg;
}


void FAST_PERCHING::runAlgorithm() {
    double time_duration = (ros::Time::now()-target.header.stamp).toSec();
    // if (time_duration > 1000)
    //     return;
    double degree = target.pose.orientation.z;
    degree += velocity_omega * time_duration;
    Eigen::Vector3d target_p, target_v, perching_axis_;
    Eigen::MatrixXd iniState;

    target_p(0) = target.pose.position.x + 0.7 * cos(degree);
    target_p(1) = target.pose.position.y + 0.7 * sin(degree);
    target_p(2) = target.pose.position.z;
    target_v(0) = 0.0;
    target_v(1) = 0.0;
    target_v(2) = 0.0;
    iniState.setZero(3, 4);
    iniState.col(0).x() = drone_pose.pose.position.x;
    iniState.col(0).y() = drone_pose.pose.position.y;
    iniState.col(0).z() = drone_pose.pose.position.z;
    perching_axis_.x() = 0.0;
    perching_axis_.y() = 0.0;
    perching_axis_.z() = 0.0;

    Eigen::Vector3d axis = perching_axis_.normalized();
    double theta = -1.5708;
    Eigen::Quaterniond target_q, land_q(1, 0, 0, 0);
    Trajectory traj;
    target_q.x() = 0.0;
    target_q.y() = 0.0;
    target_q.z() = 2.0;
    target_q.w() = 1.0;
    land_q.w() = cos(theta);
    land_q.x() = axis.x() * sin(theta);
    land_q.y() = axis.y() * sin(theta);
    land_q.z() = axis.z() * sin(theta);
    land_q = target_q * land_q;
    bool generate_new_traj_success = false;
    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    std::cout<<"finish"<<std::endl;
    visPtr_->visualize_traj(traj, "traj");








}

}