#ifndef FAST_PERCHING_HANDLE_HPP
#define FAST_PERCHING_HANDLE_HPP

#include "fast_perching.hpp"

namespace ns_fast_perching {

class FAST_PERCHING_Handle{
public:
    FAST_PERCHING_Handle(ros::NodeHandle &nodeHandle);

    int getNodeRate() const;

    void subscribeToTopics();
    void publishToTopics();
    // void actService();
    void run();
    void sendMsg();

private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber detect_state_sub;
    ros::Subscriber detect_result_sub;
    ros::Subscriber drone_pose_sub;
    ros::Subscriber state_machine_sub;

    ros::Publisher cmd_pub;
    // ros::Publisher local_pos_pub;

    // ros::ServiceClient arming_client;
    // ros::ServiceClient set_mode_client;
    
    void aruco_det_state_callback(const std_msgs::Bool::ConstPtr& msg);
    void aruco_det_result_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void drone_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void state_callback(const std_msgs::Int32::ConstPtr& msg);
    // void state_cb(const mavros_msgs::State::ConstPtr& msg);
    // void set_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    // void aruco_result_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    geometry_msgs::PoseStamped target_loc;
    nav_msgs::Odometry drone_pose;
    bool is_pub;
    // mavros_msgs::State current_state;
    // mavros_msgs::SetMode offb_set_mode;
    // mavros_msgs::SetMode land_set_mode;
    // mavros_msgs::SetMode mannual_set_mode;
    // mavros_msgs::CommandBool arm_cmd;
    // mavros_msgs::CommandBool dis_arm_cmd;
    // ros::Time last_request;
    // bool is_land;
    // ros::Time land_time;
    bool aruco_det_state;

    FAST_PERCHING fast_perching_;


};

}

#endif