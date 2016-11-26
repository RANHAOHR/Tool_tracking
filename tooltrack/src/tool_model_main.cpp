/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *    
 *    Orhan Ozguner <oxo31@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

 #include <ros/ros.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <image_transport/image_transport.h>
 #include <cwru_opencv_common/projective_geometry.h>
 //#include <cwru_needle_tracking/particle_filter.h>
 //#include <vesselness_image_filter_cpu/vesselness_lib.h>
 #include <tool_tracking_lib/tool_model.h>
 #include "std_msgs/MultiArrayLayout.h"
 #include "std_msgs/MultiArrayDimension.h"
 #include "std_msgs/Float64MultiArray.h"
 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #include <time.h> 
using namespace std;
int main(int argc, char** argv)
{
	ROS_INFO("---- In main node -----");
	ros::init(argc, argv, "tool_tracking");
	ros::NodeHandle nh;	

    cv::Mat Cam(4,4,CV_64FC1);
    Cam.at<double>(0,0) = 1;
    Cam.at<double>(1,0) = 0;
    Cam.at<double>(2,0) = 0;
    Cam.at<double>(3,0) = 0;    

    Cam.at<double>(0,1) = 0;
    Cam.at<double>(1,1) = -1;
    Cam.at<double>(2,1) = 0;
    Cam.at<double>(3,1) = 0;

    Cam.at<double>(0,2) = 0;
    Cam.at<double>(1,2) = 0;
    Cam.at<double>(2,2) = -1;
    Cam.at<double>(3,2) = 0;

    Cam.at<double>(0,3) = 0.0;   //should be in meters
    Cam.at<double>(1,3) = 0.1;
    Cam.at<double>(2,3) = 0.2;  // cannot have z = 0 for reprojection, camera_z must be always point to object
    Cam.at<double>(3,3) = 1;

    //cv::Mat Inv = Cam.inv(); use the inv od cam_mat to transform the body coord to the cam coord
    //ROS_INFO_STREAM("cam inv is: " << Inv );

	ToolModel newToolModel(Cam);

	ROS_INFO("After Loading Model and Initializetion, please press ENTER to go on");
	cin.ignore();

	clock_t t;
	clock_t t1;

	cv::Mat testImg = cv::Mat::zeros(480, 640, CV_64FC1); //
	cv::Mat P(3,4,CV_64FC1);
/******************magic numbers!!!!!!!!!!!!!!*************/
    // P.at<double>(0,0) = 500;
    // P.at<double>(1,0) = 0;
    // P.at<double>(2,0) = 0;

    // P.at<double>(0,1) = 0;
    // P.at<double>(1,1) = 500;
    // P.at<double>(2,1) = 0;

    // P.at<double>(0,2) = 320; // horiz
    // P.at<double>(1,2) = 200; //verticle
    // P.at<double>(2,2) = 1;

    // P.at<double>(0,3) = 0;
    // P.at<double>(1,3) = 0;
    // P.at<double>(2,3) = 0;

/*GENERAL P CONFIGS for DVRK*/
    P.at<double>(0,0) = 1000;
    P.at<double>(1,0) = 0;
    P.at<double>(2,0) = 0;

    P.at<double>(0,1) = 0;
    P.at<double>(1,1) = 1000;
    P.at<double>(2,1) = 0;

    P.at<double>(0,2) = 320; // horiz
    P.at<double>(1,2) = 240; //verticle
    P.at<double>(2,2) = 1;

    P.at<double>(0,3) = 0;
    P.at<double>(1,3) = 0;
    P.at<double>(2,3) = 0;

	ToolModel::toolModel initial;

	initial.tvec_cyl(0) = 0.0;
	initial.tvec_cyl(1) = 0.0;
	initial.tvec_cyl(2) = 0.0;
	initial.rvec_cyl(0) = 0.0;
	initial.rvec_cyl(1) = M_PI/2;
	initial.rvec_cyl(2) = 0.0;


	ToolModel::toolModel newTool;

	
	t1 = clock();
	newTool = newToolModel.setRandomConfig(initial, 1, 0);
	t1 = clock() - t1;

	//newTool = initial;
	t = clock();
	cv::Rect testROI = newToolModel.renderTool(testImg, newTool, Cam, P );
	t = clock() - t;

	float sec1 = (float)t1/CLOCKS_PER_SEC;
	float sec = (float)t/CLOCKS_PER_SEC;


	ROS_INFO_STREAM("setRandomConfig time is: " << sec1);
	ROS_INFO_STREAM("render time is: " << sec);

	if(!testImg.empty()){
        imshow("test", testImg);
    }
	

	cv::waitKey(0);





	return 0;

}

