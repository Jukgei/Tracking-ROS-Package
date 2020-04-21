#include <iostream>
#include "../include/Tracking/tracking.hpp"
#include <ros/ros.h>


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
