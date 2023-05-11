#include "fast_perching_handle.hpp"

namespace ns_fast_perching {

FAST_PERCHING_Handle::FAST_PERCHING_Handle(ros::NodeHandle &nodeHandle):nodeHandle_(nodeHandle), fast_perching_(nodeHandle) {
    ROS_INFO("Constructing Handle");
    subscribeToTopics();
    publishToTopics();
    
    aruco_det_state = false;
}

void FAST_PERCHING_Handle::aruco_det_state_callback(const std_msgs::Bool::ConstPtr& msg) {
    aruco_det_state = msg->data; // 1 detected 0 not detected
}

void FAST_PERCHING_Handle::aruco_det_result_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    target_loc = *msg;
    fast_perching_.set_target(target_loc);
}

void FAST_PERCHING_Handle::drone_pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    drone_pose = *msg;
    fast_perching_.set_drone_pose(drone_pose);
}

int FAST_PERCHING_Handle::getNodeRate() const {
    return 5;
}

void FAST_PERCHING_Handle::subscribeToTopics() {
    detect_state_sub = nodeHandle_.subscribe<std_msgs::Bool>("/aruco_det/state", 10, &FAST_PERCHING_Handle::aruco_det_state_callback, this);
    detect_result_sub = nodeHandle_.subscribe<geometry_msgs::PoseStamped>("/aruco_det/target_loc", 10, &FAST_PERCHING_Handle::aruco_det_result_callback, this);
    drone_pose_sub = nodeHandle_.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 10, &FAST_PERCHING_Handle::drone_pose_callback, this);
    // state_sub = nodeHandle_.subscribe<mavros_msgs::State>("/mavros/state", 10, &FAST_PERCHING_Handle::state_cb, this);
    // pose_sub = nodeHandle_.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, &FAST_PERCHING_Handle::set_pose_callback, this);
    // aruco_sub = nodeHandle_.subscribe<geometry_msgs::PoseStamped>("/aruco_det/target_loc", 10, &FAST_PERCHING_Handle::aruco_result_callback, this);
}

void FAST_PERCHING_Handle::publishToTopics() {
    cmd_pub = nodeHandle_.advertise<quadrotor_msgs::PositionCommand>("/cmd", 10);
    // local_pos_pub = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
}

// void FAST_PERCHING_Handle::actService() {
    // arming_client = nodeHandle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    // set_mode_client = nodeHandle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
// }

void FAST_PERCHING_Handle::run() {
    fast_perching_.runAlgorithm();
    sendMsg();
}

void FAST_PERCHING_Handle::sendMsg() {
//     if (!is_land)

    cmd_pub.publish(fast_perching_.get_next_point());
}

}