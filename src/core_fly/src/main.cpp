#include <ros/ros.h>
#include "core_fly_handle.hpp"

typedef ns_core_fly::CORE_FLY_Handle CORE_FLY_Handle;

int main(int argc, char **argv){
    ros::init(argc, argv, "core_fly_node");
    ros::NodeHandle nodeHandle("~");

    CORE_FLY_Handle myCORE_FLY_Handle(nodeHandle);
    
    ros::Rate loop_rate(20);

    while(ros::ok()) {
        myCORE_FLY_Handle.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;    
}