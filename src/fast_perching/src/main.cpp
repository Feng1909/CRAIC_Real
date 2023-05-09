#include <ros/ros.h>
#include "fast_perching_handle.hpp"

typedef ns_fast_perching::FAST_PERCHING_Handle FAST_PERCHING_Handle;

int main(int argc, char **argv){
    ros::init(argc, argv, "fast_perching");
    ros::NodeHandle nodeHandle("~");

    FAST_PERCHING_Handle myFAST_PERCHING_Handle(nodeHandle);
    
    ros::Rate loop_rate(5);

    while(ros::ok()) {
        myFAST_PERCHING_Handle.run();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;    
}