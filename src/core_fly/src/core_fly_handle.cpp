#include "core_fly_handle.hpp"

namespace ns_core_fly {

CORE_FLY_Handle::CORE_FLY_Handle(ros::NodeHandle &nodeHandle):nodeHandle_(nodeHandle), core_fly_(nodeHandle) {
    ROS_INFO("Constructing Handle");
    subscribeToTopics();
    publishToTopics();
    actService();
    ros::Rate rate(20.0);
    while(ros::ok() && !current_state.connected) {
        ROS_INFO("waiting for connection");
        ros::spinOnce();
        rate.sleep();
    }
    last_request = ros::Time::now();
    offb_set_mode.request.custom_mode = "OFFBOARD";
    land_set_mode.request.custom_mode = "AUTO.LAND";
    mannual_set_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = true;
    dis_arm_cmd.request.value = false;
    is_land = false;
    land_time = ros::Time().fromSec(0);
    aruco_result_time = ros::Time::now();
}

void CORE_FLY_Handle::state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void CORE_FLY_Handle::set_pose_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    state = *msg;
    core_fly_.update_state(state);
}

void CORE_FLY_Handle::aruco_result_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if((state.pose.pose.position.x - 0.7)<0.1) {
        is_land = true;
        aruco_result_time = ros::Time::now();
    }
}

int CORE_FLY_Handle::getNodeRate() const {
    return 5;
}

void CORE_FLY_Handle::subscribeToTopics() {
    state_sub = nodeHandle_.subscribe<mavros_msgs::State>("/mavros/state", 10, &CORE_FLY_Handle::state_cb, this);
    pose_sub = nodeHandle_.subscribe<nav_msgs::Odometry>("/vins_fusion/imu_propagate", 10, &CORE_FLY_Handle::set_pose_callback, this);
    aruco_sub = nodeHandle_.subscribe<geometry_msgs::PoseStamped>("/aruco_det/target_loc", 10, &CORE_FLY_Handle::aruco_result_callback, this);
}

void CORE_FLY_Handle::publishToTopics() {
    // local_pos_pub = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    local_pos_pub = nodeHandle_.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
}

void CORE_FLY_Handle::actService() {
    arming_client = nodeHandle_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client = nodeHandle_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
}

void CORE_FLY_Handle::run() {
    core_fly_.runAlgorithm();
    if (!is_land && current_state.mode != "OFFBOARD" && ros::Time::now() - last_request > ros::Duration(5.0)) {
        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("Offboard enabled");
        }
        last_request = ros::Time::now();
    }
    else {
        if (!is_land && !current_state.armed && ros::Time::now() - last_request > ros::Duration(5.0)) {
            if(arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
    }
    sendMsg();
}

void CORE_FLY_Handle::sendMsg() {
    quadrotor_msgs::PositionCommand next_point;
    geometry_msgs::PoseStamped next_point_;
    next_point_ = core_fly_.get_goal_pose();


    next_point.header.stamp = ros::Time::now();
    next_point.header.frame_id = "world";
    next_point.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    next_point.trajectory_id = 0;
    next_point.position.x = next_point_.pose.position.x;
    next_point.position.y = next_point_.pose.position.y;
    next_point.position.z = next_point_.pose.position.z;
    if (!is_land)
        local_pos_pub.publish(next_point);
}

}