#include <iostream>
#include "../include/Tracking/tracking.hpp"
#include <ros/ros.h>

bool check_line_state = false;
bool First_flag = true;
bool submit_roi_flag = false;
bool end_loop_flag = false;

int main( int argc, char **argv ){
    ros::init(argc, argv, "Tracking");
    ros::NodeHandle n;
    //RoboticArm::RoboticArmNode * myArmNode = new RoboticArm::RoboticArmNode(n);

    Tracking::TrackingNode *myTrackingNode = new Tracking::TrackingNode(n);
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    delete myTrackingNode;
    myTrackingNode = nullptr;
    
    return 0;

}
