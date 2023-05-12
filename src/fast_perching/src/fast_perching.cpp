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

void FAST_PERCHING::set_drone_pose(nav_msgs::Odometry msg) {
    drone_pose = msg;
}

quadrotor_msgs::PositionCommand FAST_PERCHING::get_next_point() {
    return next_point;
}


void FAST_PERCHING::runAlgorithm() {
    double time_duration = (ros::Time::now()-target.header.stamp).toSec();
    // if (time_duration > 1000)
    //     return;
    double degree = target.pose.orientation.z;
    degree += velocity_omega * time_duration;
    Eigen::Vector3d target_p, target_v, perching_axis_;
    Eigen::MatrixXd iniState;

    // target_p(0) = target.pose.position.x + 0.7 * cos(degree);
    // target_p(1) = target.pose.position.y + 0.7 * sin(degree);
    // target_p(0) = target.pose.position.x;
    // target_p(1) = target.pose.position.y;
    // target_p(2) = target.pose.position.z;
    target_p(0) = 0.1;
    target_p(1) = -0.1;
    target_p(2) = 0.18;
    target_v(0) = 0.0;
    target_v(1) = 0.0;
    target_v(2) = 0.0;
    iniState.setZero(3, 4);
    iniState.col(0).x() = drone_pose.pose.pose.position.x;
    iniState.col(0).y() = drone_pose.pose.pose.position.y;
    iniState.col(0).z() = drone_pose.pose.pose.position.z;
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
    std::cout<<"init state: "<<iniState<<std::endl;
    std::cout<<"target p: "<<target_p<<std::endl;
    generate_new_traj_success = trajOptPtr_->generate_traj(iniState, target_p, target_v, land_q, 10, traj);
    std::cout<<"time duration: "<<traj.getDurations()<<std::endl;
    visPtr_->visualize_traj(traj, "traj");

    double select_time = 0.1;
    Eigen::Vector3d next_pose = traj.getPos(select_time);
    Eigen::Vector3d next_vel  = traj.getVel(select_time);
    Eigen::Vector3d next_acc  = traj.getAcc(select_time);
    Eigen::Vector3d next_jerk = traj.getJer(select_time);
    
    next_point.header.stamp = ros::Time::now();
    next_point.header.frame_id = "world";
    next_point.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    next_point.trajectory_id = 0;
    next_point.position.x = next_pose(0);
    next_point.position.y = next_pose(1);
    next_point.position.z = next_pose(2);

    next_point.velocity.x = next_vel(0);
    next_point.velocity.y = next_vel(1);
    next_point.velocity.z = next_vel(2);
    
    next_point.acceleration.x = next_acc(0);
    next_point.acceleration.y = next_acc(1);
    next_point.acceleration.z = next_acc(2);

    next_point.jerk.x = next_jerk(0);
    next_point.jerk.y = next_jerk(1);
    next_point.jerk.z = next_jerk(2);

    next_point.yaw = 0;
    next_point.yaw_dot = 0;
    
}

}