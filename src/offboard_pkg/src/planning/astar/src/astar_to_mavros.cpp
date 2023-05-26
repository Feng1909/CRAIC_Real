/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/PositionTarget.h>

#include "offboard_pkg/ControlCommand.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>


Eigen::Vector3d pos_drone_t265;
Eigen::Quaterniond q_t265;

mavros_msgs::PositionTarget Command_Now;

float position_set[3]={0,0,0};

float velocity_set[3]={0,0,0};
int offboard_flag = 0;
bool auto_takeoff = true;    
bool is_takeoff = false;
mavros_msgs::State current_state;
int drone_control_command_count = 0;
float height = 0.3;
bool have_pose_pub = false;
ros::Publisher setpoint_raw_local_pub;

void t265_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    //if (msg->header.frame_id == "t265_odom_frame")
    {
        pos_drone_t265 = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        // pos_drone_t265[0] = msg->pose.pose.position.x + pos_offset[0];
        // pos_drone_t265[1] = msg->pose.pose.position.y + pos_offset[1];
        // pos_drone_t265[2] = msg->pose.pose.position.z + pos_offset[2];

        q_t265 = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        //Euler_t265 = quaternion_to_euler(q_gazebo);
        // Euler_t265[2] = Euler_t265[2] + yaw_offset;
        // q_t265 = quaternion_from_rpy(Euler_t265);
    }
    //else
    //{
        
    //}
}



//仿照https://gitee.com/maxibooksiyi/ego_ws/blob/for_vinsfusion_without-devel-build/src/px4_com/src/px4_replan_sender.cpp
void setpoint_raw_cb(const mavros_msgs::PositionTarget::ConstPtr& msg)
{
    Command_Now = *msg;
}

void drone_control_command_cb(const offboard_pkg::ControlCommand::ConstPtr& msg)
{
    position_set[0] = msg->Reference_State.position_ref[0];
    position_set[1] = msg->Reference_State.position_ref[1];
    position_set[2] = msg->Reference_State.position_ref[2];

    velocity_set[0] = msg->Reference_State.velocity_ref[0];
    velocity_set[1] = msg->Reference_State.velocity_ref[1];
    velocity_set[2] = msg->Reference_State.velocity_ref[2];

    drone_control_command_count = drone_control_command_count + 1;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
        {
	    if(!is_takeoff && abs(msg->pose.position.z - height) < 0.1)
	     {
                is_takeoff = true;
	     }
        }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_to_mavros");
    ros::NodeHandle nh;

    nh.param<bool>("auto_takeoff", auto_takeoff, true);
    nh.param<float>("height", height, 0.3);
        //  【订阅】t265估计位置
    //ros::Subscriber t265_sub = nh.subscribe<nav_msgs::Odometry>("/t265/odom/sample", 100, t265_cb);

    //ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    setpoint_raw_local_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pos_cb);

    //ros::Subscriber setpoint_raw_local_pre_sub = nh.subscribe<mavros_msgs::PositionTarget>("/setpoint_raw/local_pre", 100, setpoint_raw_cb);

    ros::Subscriber drone_control_command_sub = nh.subscribe<offboard_pkg::ControlCommand>("/drone/control_command", 100, drone_control_command_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(30.0);

    
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
   mavros_msgs::PositionTarget pos_setpoint;

    
    while(ros::ok())
    {
    	if(auto_takeoff && offboard_flag == 0 && !is_takeoff)
   	 {
       		pos_setpoint.type_mask =  0b100111111000;     //x y z+ yaw

       		pos_setpoint.coordinate_frame = 1;

       		pos_setpoint.position.x = 0;
       		pos_setpoint.position.y = 0;
       		pos_setpoint.position.z = height;

       		pos_setpoint.yaw = 1.57;
      		setpoint_raw_local_pub.publish(pos_setpoint);
      		ros::spinOnce();
               // ROS_INFO("takeoff !");
		if(ros::Time::now() - last_request < ros::Duration(1))
		{
                    rate.sleep();
		    
		    continue;
		}
		else
		{
			last_request = ros::Time::now();
		}
    	}


        //setpoint_raw_local_pub.publish(Command_Now);

	if(auto_takeoff)
        {
            if( current_state.mode != "OFFBOARD")
            {
               if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
                 {
                	ROS_INFO("Offboard enabled");
                }
            }
          else 
          {
            if( !current_state.armed)
            {
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                 {
                    ROS_INFO("Vehicle armed");
                 }         
            }
          } 
        }
    

    if(drone_control_command_count > 0 && is_takeoff )
       {
       /*******************************************************************************************
       // 速度作为前馈项， 参见FlightTaskOffboard.cpp
       // 2. position setpoint + velocity setpoint (velocity used as feedforward)
       // 控制方法请见 PositionControl.cpp
       //pos_setpoint.type_mask = 0b100111000000;   // 100 111 000 000  vx vy　vz x y z+ yaw

       //pos_setpoint.type_mask =  0b100111111000;     //x y z+ yaw

       pos_setpoint.coordinate_frame = 1;

       pos_setpoint.position.x = position_set[0];
       pos_setpoint.position.y = position_set[1];
       pos_setpoint.position.z = position_set[2];
       pos_setpoint.velocity.x = velocity_set[0];
       pos_setpoint.velocity.y = velocity_set[1];
       pos_setpoint.velocity.z = velocity_set[2];

       pos_setpoint.yaw = 0;
       **********************************************************************************************/
       pos_setpoint.type_mask =  0b100111111000;     //x y z+ yaw

       pos_setpoint.coordinate_frame = 1;

       pos_setpoint.position.x = position_set[0];
       pos_setpoint.position.y = position_set[1];
       pos_setpoint.position.z = position_set[2];
       //pos_setpoint.velocity.x = velocity_set[0];
       //pos_setpoint.velocity.y = velocity_set[1];
       //pos_setpoint.velocity.z = velocity_set[2];

       pos_setpoint.yaw = 1.57;
       }
     
      setpoint_raw_local_pub.publish(pos_setpoint);
      ros::spinOnce();
      have_pose_pub = true;

        rate.sleep();
    }

    return 0;
}



