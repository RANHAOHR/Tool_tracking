/*
*  Copyright (c) 2016
*  Ran Hao <rxh349@case.edu>
*
*  All rights reserved.
*
*  @The functions in this file create the random Tool Model (loaded OBJ files) and render tool models to Images
*/

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <tool_model_lib/tool_model.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

int main(int argc, char **argv) {
    ROS_INFO("---- In main node -----");
    ros::init(argc, argv, "tool_tracking");
    ros::NodeHandle nh;

    cv::Mat Cam(4, 4, CV_64FC1);

    Cam = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0.0,
    0, 0, -1, 0.0,
    0, 1, 0, 0.2,
    0, 0, 0, 1);  ///should be camera extrinsic parameter relative to the tools

    ToolModel newToolModel;

    ROS_INFO("After Loading Model and Initialization, please press ENTER to go on");
    cin.ignore();

    cv::Mat testImg = cv::Mat::zeros(480, 640, CV_8UC1); //CV_8UC3
    cv::Mat P(3, 4, CV_64FC1);

//    cv::Size size(640, 480);
//    cv::Mat segImg = cv::imread("/home/rxh349/ros_ws/src/Tool_tracking/tool_tracking/left.png",CV_LOAD_IMAGE_GRAYSCALE );
//    cv::resize(segImg, segImg,size );

    /******************magic numbers*************/

    /*actual Projection matrix*/
    P.at<double>(0, 0) = 893.7852590197848;
    P.at<double>(1, 0) = 0;
    P.at<double>(2, 0) = 0;

    P.at<double>(0, 1) = 0;
    P.at<double>(1, 1) = 893.7852590197848;
    P.at<double>(2, 1) = 0;

    P.at<double>(0, 2) = 288.4443244934082; // horiz
    P.at<double>(1, 2) = 259.7727756500244; //verticle
    P.at<double>(2, 2) = 1;

    P.at<double>(0, 3) = 0.0;//4.732;
    P.at<double>(1, 3) = 0;
    P.at<double>(2, 3) = 0;


    ToolModel::toolModel initial;

    initial.tvec_cyl(0) = 0.01;// +0.4  //left and right (image frame)
    initial.tvec_cyl(1) = 0.0;  //up and down
    initial.tvec_cyl(2) = 0.04;
    initial.rvec_cyl(0) = 0.0;
    initial.rvec_cyl(1) = 1.4;
    initial.rvec_cyl(2) = 0.2;

    newToolModel.computeEllipsePose(initial, 0.0, 0.0, 0.0 );
    cv::imshow("tool image: ",testImg );

    cv::waitKey(0);
/***********testing below***************/
//    for (int i = 0; i < 20; ++i) {
//        double genrater = newToolModel.randomNumber(0.01, 0);
//        ROS_INFO_STREAM("genrater" << genrater);
//    }
//    ROS_INFO_STREAM("setRandomConfig time is: " << sec1);
//    ROS_INFO_STREAM("render time is: " << sec);
       //double result = newToolModel.calculateMatchingScore(testImg, segImg);
//
//        t2 = clock();
        //double chamfer_result = newToolModel.calculateChamferScore(testImg, segImg);
//        t2 = clock() - t2;
       //ROS_INFO_STREAM("THE MATCHING SCORE IS: " << result);
//        ROS_INFO_STREAM("THE chamfer SCORE IS: " << chamfer_result);

//    float sec2 = (float) t2 / CLOCKS_PER_SEC;
//    ROS_INFO_STREAM("MATCHING TIME: " << sec2);

    return 0;

}

