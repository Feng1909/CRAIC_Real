#ifndef CORE_FLY_HANDLE_HPP
#define CORE_FLY_HANDLE_HPP

#include "core_fly.hpp"

namespace ns_core_fly {

class CORE_FLY_Handle{
public:
    CORE_FLY_Handle(ros::NodeHandle &nodeHandle);

    int getNodeRate() const;

    void subscribeToTopics();
    void publishToTopics();
    void actService();
    void run();
    void sendMsg();

private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber state_sub;
    ros::Subscriber pose_sub;
    ros::Subscriber aruco_sub;

    ros::Publisher local_pos_pub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void set_pose_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void aruco_result_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    mavros_msgs::State current_state;
    mavros_msgs::SetMode offb_set_mode;
    mavros_msgs::SetMode land_set_mode;
    mavros_msgs::SetMode mannual_set_mode;
    mavros_msgs::CommandBool arm_cmd;
    mavros_msgs::CommandBool dis_arm_cmd;
    ros::Time last_request;
    nav_msgs::Odometry state;
    bool is_land;
    ros::Time aruco_result_time;
    ros::Time land_time;

    CORE_FLY core_fly_;


};

}

#endif