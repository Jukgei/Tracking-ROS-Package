#include "../include/Tracking/tracking.hpp"
#include <thread>
#include <unistd.h>
#include <iostream>

Tracking::TrackingNode::TrackingNode(ros::NodeHandle &n){

    
    isDisplay = true;
    windowName = "RealSense";

    l = 666.66;
    targetHeight = 666.66;

   this->InitPublishers(n);

   this->InitSubcribers(n);

   this->InitTrackingThread();

}


void Tracking::TrackingNode::InitSubcribers(ros::NodeHandle &n){
    //ArmPosSubscriber = n.subscribe<RoboticArm::state>
    //    ("state",10,&RoboticArmNode::GetArmPosCallBack,this);

    //ArmSetPointSubscriber = n.subscribe<RoboticArm::setpoint>
    //    ("setpoint",10,&RoboticArmNode::GetSetPointCallBack,this);
}

void Tracking::TrackingNode::InitPublishers(ros::NodeHandle &n){
    this->TrackingPublisher = n.advertise<Tracking::tracking>("trackingFlag",10);
    
    this->SetPointPublisher = n.advertise<Tracking::setpoint>("setpoint",10);
    
    std::thread pub(std::bind(&TrackingNode::SetPointPublish,this));

    pub.detach();
}

void Tracking::TrackingNode::InitTrackingThread(){
    std::thread track(std::bind(&TrackingNode::TrackingThread,this));
    track.detach();
}

void Tracking::mouse_pick_roi(int event, int x, int y,int flags, void * param)
{
    ROI_zone * zone = (ROI_zone *)param;
    if(event == CV_EVENT_LBUTTONDOWN)
    {
        zone->ROI_rect.x = x;
        zone->ROI_rect.y = y;
        check_line_state = true;
    }
    else if((event == CV_EVENT_MOUSEMOVE) && (check_line_state == true))
    {
       // p1 = cvPoint(ROI_rect.x,ROI_rect.y);
       // p2 = cvPoint(x,y);
        zone->p_start = cvPoint(zone->ROI_rect.x,zone->ROI_rect.y);
        zone->p_end = cvPoint(x,y);
       // rectangle((*picture),p1,p2,CV_RGB(0,255,0),2,CV_AA,0);
//        imshow(windowName,(*picture));
    }
    else if(event == CV_EVENT_LBUTTONUP)
    {
        zone->p_start = cvPoint(zone->ROI_rect.x,zone->ROI_rect.y);
        zone->p_end = cvPoint(x,y);
        check_line_state = false;
        waitKey(20);
    }
}

void Tracking::TrackingNode::TrackingThread(){
    tracker = TrackerKCF::create();
    
    target.ROI_rect.x = 0;
    target.ROI_rect.y = 0;
    target.p_start = cvPoint(0,0);
    target.p_end = cvPoint(0,0);

    config.enable_stream(RS2_STREAM_COLOR,WIDTH,HEIGHT,RS2_FORMAT_BGR8, 30);
    config.enable_stream(RS2_STREAM_DEPTH,WIDTH,HEIGHT,RS2_FORMAT_Z16,30);
    p.start(config);
    
    rs2::align align(RS2_STREAM_COLOR);

    namedWindow("RealSense",WINDOW_AUTOSIZE);

    while( ros::ok())
    {  

        Mat Display;
        rs2::frameset data  = p.wait_for_frames();

        data = align.process(data);  //align depth to color


        //end = clock();
        rs2::frame depth = data.get_depth_frame().apply_filter(color_map);
        rs2::frame depth1 = data.get_depth_frame();      
        //rs2::frame depth1 = depth;
        rs2::frame color = data.get_color_frame();

        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();
        Mat depth_opencv(Size(WIDTH,HEIGHT),CV_8UC3,(void *)depth.get_data(),Mat::AUTO_STEP);
             

        auto color_opencv = frame_to_mat(color);
        auto distance_matrix = depth_frame_to_meters(p,depth1); //double
        //cout<<"Distance is:"<<distance_matrix.at<double>(WIDTH/2,HEIGHT/2)<<"\r";
       
        //hconcat(color_opencv,color_opencv,Display);
        if(First_flag)
        {
            setMouseCallback(windowName,mouse_pick_roi,&target);
            First_flag = false;
        }
        if(submit_roi_flag == false && isDisplay)
        {
            rectangle(color_opencv,target.p_start,target.p_end,CV_RGB(0,255,0),2,CV_AA,0);
            rectangle(depth_opencv,target.p_start,target.p_end,CV_RGB(0,255,0),2,CV_AA,0);
        
            vconcat(color_opencv,depth_opencv,Display);
            imshow(windowName,Display);
        }


        switch(waitKey(1))
        {
            case 'q':
            {
                end_loop_flag = true;
                break;
            }
                    
            case '\n':
            {
                submit_roi_flag = true;
                cout<<"Submit"<<endl;
                roi =  Rect2d(target.p_start,target.p_end);
                tracker->init(color_opencv,roi);
                break;
            }

            default: break;
        }
        if(end_loop_flag) {break;}
        if(submit_roi_flag)
        {
            Tracking::tracking flag;
            tracker->update(color_opencv,roi);
            rectangle(color_opencv,roi,Scalar(0,255,0),2,1);
            rectangle(depth_opencv,roi,Scalar(0,255,0),2,1);
            //cout<<roi<<"\r";
            flag.isTracking = true;
            this->TrackingPublisher.publish(flag);
            std::cout<<"Positon(x,y):"<<roi.x +roi.width/2<<","<<roi.y + roi.height/2  <<"\r";
            l = roi.x +roi.width/2;
            targetHeight = roi.y +roi.height/2;
            if(isDisplay){
                vconcat(color_opencv,depth_opencv,Display);
                imshow(windowName,Display);
            }
        }
    }

}

void Tracking::TrackingNode::SetPointPublish(){
    Tracking::setpoint s;
    ros::Rate LoopRate(50);

    while(ros::ok()){

        std::vector<float> pos;
        std::vector<float> att;
        att.push_back(0);
        att.push_back(0);
        att.push_back(0);
           
        pos.push_back(l);
        pos.push_back(targetHeight);
        pos.push_back(0);

        s.attitude = att;
        s.position = pos;
        this->SetPointPublisher.publish(s);
        LoopRate.sleep();
    }
    
    std::vector<float> pos;
    std::vector<float> att;
    att.push_back(0);
    att.push_back(0);
    att.push_back(0);

}

cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

// Converts depth frame to a matrix of doubles with distances in meters
cv::Mat depth_frame_to_meters(const rs2::pipeline& pipe, const rs2::depth_frame& f)
{
    using namespace cv;
    using namespace rs2;

    Mat dm = frame_to_mat(f);
    dm.convertTo(dm, CV_64F);
    auto depth_scale = pipe.get_active_profile()
        .get_device()
        .first<depth_sensor>()
        .get_depth_scale();
    dm = dm * depth_scale;
    return dm;
}
