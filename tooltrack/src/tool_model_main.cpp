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
    Cam.at<double>(1,1) = 1;
    Cam.at<double>(2,1) = 0;
    Cam.at<double>(3,1) = 0;

    Cam.at<double>(0,2) = 0;
    Cam.at<double>(1,2) = 0;
    Cam.at<double>(2,2) = 1;
    Cam.at<double>(3,2) = 0;

    Cam.at<double>(0,3) = 0.0;   //should this be in inch
    Cam.at<double>(1,3) = 0.0;
    Cam.at<double>(2,3) = 0.10;
    Cam.at<double>(3,3) = 1;

	ToolModel newToolModel(Cam);

	ToolModel::toolModel initial;

	initial.tvec_cyl(0) = 0.0;
	initial.tvec_cyl(1) = 0.0;
	initial.tvec_cyl(2) = 0.0;
	initial.rvec_cyl(0) = 0.0;
	initial.rvec_cyl(1) = 0.0;
	initial.rvec_cyl(2) = 0.0;

/*	initial.tvec_elp(0) = 0.0;
	initial.tvec_elp(1) = 0.0;
	initial.tvec_elp(2) = 0.0;
	initial.rvec_elp(0) = 0.0;
	initial.rvec_elp(1) = 0.0;
	initial.rvec_elp(2) = 0.0;*/

	initial.theta_ellipse = 0.0;
	initial.theta_grip_1 = 0.0;
	initial.theta_grip_2 = 0.0;  //9 DOF


	cv::Mat testImg = cv::Mat::zeros(480, 640, CV_64FC1); //
	cv::Mat P(3,4,CV_64FC1);

    P.at<double>(0,0) = 1000;
    P.at<double>(1,0) = 0;
    P.at<double>(2,0) = 0;

    P.at<double>(0,1) = 0;
    P.at<double>(1,1) = 1000;
    P.at<double>(2,1) = 0;

    P.at<double>(0,2) = 320;
    P.at<double>(1,2) = 240;
    P.at<double>(2,2) = 1;

    P.at<double>(0,3) = 0;
    P.at<double>(1,3) = 0;
    P.at<double>(2,3) = 0;



	ToolModel::toolModel newTool;

	ROS_INFO("after loading");
	//newTool = newToolModel.setRandomConfig(initial, 1, 0);
	newTool = initial;
	ROS_INFO("after setRandomconfig");
	cv::Rect testROI = newToolModel.renderTool(testImg, newTool, P, Cam);
	ROS_INFO("after render");
	ROS_INFO_STREAM("p: " << P);

	// cv::Point3d input_v1;
	// input_v1.x = -1;
	// input_v1.y = 0;
	// input_v1.z = 0;

	// cv::Point3d input_v2;
	// input_v2.x = 0;
	// input_v2.y = -1;
	// input_v2.z = 0;

	// cv::Point3d input_v3;
	// input_v3.x = 0;
	// input_v3.y = 0;
	// input_v3.z = -1;

	// cv::Point3d input_n1;
	// input_n1.x = 0;
	// input_n1.y = 0;
	// input_n1.z = 1;

	// cv::Point3d input_n2;
	// input_n2.x = 0;
	// input_n2.y = 0;
	// input_n2.z = 1;

	// cv::Point3d input_n3;
	// input_n3.x = 0;
	// input_n3.y = 0;
	// input_n3.z = 1;

	// cv::Point3d normal = newToolModel.FindFaceNormal(input_v1, input_v2, input_v3,
 //                                     input_n1, input_n2, input_n3);

	// cout<<"normal.x "<< normal.x <<endl;
	// cout<<"normal.y "<< normal.y <<endl;
	// cout<<"normal.z "<< normal.z <<endl;

	if(!testImg.empty()){
        imshow("test", testImg);
    }
	

	cv::waitKey(0);





	return 0;

}

