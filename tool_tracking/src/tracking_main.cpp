#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <tool_tracking/particle_filter.h>
// #include <vesselness_image_filter_cpu/vesselness_lib.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>


bool freshImage;
bool freshCameraInfo;
bool freshVelocity;
using namespace cv_projective;
double Arr[6];


std::vector<cv::Mat> trackingImgs;  ///this should be CV_32F


void newImageCallback(const sensor_msgs::ImageConstPtr& msg, cv::Mat* outputImage)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //cv::Mat src =  cv_bridge::toCvShare(msg,"32FC1")->image;
        //outputImage[0] = src.clone();
        cv_ptr = cv_bridge::toCvCopy(msg);
        outputImage[0] = cv_ptr->image;
        freshImage = true;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg ->encoding.c_str());
    }

}

// receive body velocity
void arrayCallback(const std_msgs::Float64MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<double>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        Arr[i] = *it;
        i++;
    }

    freshVelocity = true;

}

void timerCB(const ros::TimerEvent&)
{

    std::vector<cv::Mat> disp;
    disp.resize(2);

   // for (int j(0); j<2; j++)
   // {
   //     convertSegmentImageCPUBW(trackingImgs[j],disp[j]);  //what is this?
   // }

   //  cv::imshow( "Trancking Image: LEFT", disp[0]);
   //  cv::imshow( "Trancking Image: RIGHT", disp[1]);

   //  cv::waitKey(10);

}


cv::Mat segmentation(cv::Mat &InputImg){

    //cout<<"in segmentation"<<endl;
    cv::Mat src, src_gray;
    cv::Mat grad;

    src = InputImg;

    resize(src, src, cv::Size(), 1, 1);

    double lowThresh = 43;
    cv::cvtColor( src, src_gray, CV_BGR2GRAY );

    blur( src_gray, src_gray, cv::Size(3,3) );

    Canny( src_gray, grad, lowThresh, 4*lowThresh, 3 );

    return grad;

}

int main(int argc, char** argv){

    ros::init(argc, argv, "tracking_node");

    ros::NodeHandle nh;

    /******  initialization  ******/
    ParticleFilter Particles(&nh);
    freshCameraInfo = false;
    freshVelocity = false;

    cv::Mat bodyVel = cv::Mat::zeros(6,1,CV_64FC1);

    trackingImgs.resize(2);

    // Camera intrinsic matrices
    cv::Mat P_l, P_r;
    P_l.setTo(0);
    P_r.setTo(0);

    cv::Mat seg_left;
    cv::Mat seg_right;

    const std::string leftCameraTopic("/stereo_example/left/camera_info");
    const std::string rightCameraTopic("/stereo_example/right/camera_info");
    cameraProjectionMatrices cameraInfoObj(nh, leftCameraTopic, rightCameraTopic);
    ROS_INFO("---- Connected to camera info -----");

    /*** timer set up ***/
    //ros::Rate loop_rate(50);

    ros::Timer timer = nh.createTimer(ros::Duration(0.01), timerCB); //show images

    /*** subscribers, velocity, stream images ***/
    ros::Subscriber sub3 = nh.subscribe("/bodyVelocity", 100, arrayCallback);

    cv::Mat rawImage_left = cv::Mat::zeros(640, 920, CV_32FC1);
    cv::Mat rawImage_right = cv::Mat::zeros(640, 920, CV_32FC1);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1,
                                                         boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));
    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1,
                                                         boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    ROS_INFO("---- done subscribe -----");



    while(nh.ok()){
        ros::spinOnce();

        seg_left = segmentation(rawImage_left);
        seg_right = segmentation(rawImage_right);

        /*** make sure camera information is ready ***/
        if(freshCameraInfo == false)
        {
            //retrive camera info
            P_l = cameraInfoObj.getLeftProjectionMatrix();
            P_r = cameraInfoObj.getRightProjectionMatrix();
            if(P_l.at<double>(0,0) != 0 && P_r.at<double>(0,0))
            {
                ROS_INFO("obtained camera info");
                freshCameraInfo = true;
            }


        }

        /*** if camera is ready, doing the tracking based on segemented image***/
        if(freshImage && freshCameraInfo && freshVelocity)
        {
            //Stage body velocity
            for(int i(0);i<6;i++)
            {
                bodyVel.at<double>(i,0) = Arr[i];
            }

            trackingImgs = Particles.trackingTool(bodyVel, seg_left, seg_right, P_l, P_r); //with rendered tool and segmented img

            freshImage = false;
            freshVelocity = false;

        }


    }



}