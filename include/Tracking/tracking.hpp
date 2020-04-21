#ifndef TRACKING_H
#define TRACKING_H

#define WIDTH   640
#define HEIGHT  360

//ros include
#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

//DJI include 
#include "../../../../devel/include/dji_sdk/DroneTaskControl.h"
#include "../../../../devel/include/dji_sdk/SDKControlAuthority.h"

#include "../../../dji_sdk/include/dji_sdk/dji_sdk.h"

//#include "Tracking/tracking.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <librealsense2/rs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <exception>
//#include "cv-helpers.hpp"

#include "Tracking/setpoint.h"
#include "Tracking/tracking.h"

using namespace std;
using namespace cv;


extern bool check_line_state;
extern bool First_flag;
extern bool submit_roi_flag; 
extern bool end_loop_flag;

cv::Mat frame_to_mat(const rs2::frame& f);
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f);
namespace Tracking{

typedef struct{
    CvRect ROI_rect;
    CvPoint p_start;
    CvPoint p_end;
}ROI_zone;

void mouse_pick_roi(int event, int x, int y,int flags, void * param);

class TrackingNode{

public:
    TrackingNode(ros::NodeHandle &n);
    void InitSubcribers(ros::NodeHandle &n);
    void InitPublishers(ros::NodeHandle &n);
    void InitTrackingThread();
    void TrackingThread();   //run tracking algorithm 
    void SetPointPublish();
    
    //void Publish();


private:
    ros::Publisher TrackingPublisher;
    ros::Publisher SetPointPublisher;
   
    
    rs2::pipeline p;
    rs2::colorizer color_map;
    rs2::config config;
    
    Ptr<cv::TrackerKCF> tracker; 
    ROI_zone target;
    Rect2d roi;
    
    bool isDisplay;
    
    std::string windowName; 
    
    float l;
    float targetHeight;

};
}

#endif
