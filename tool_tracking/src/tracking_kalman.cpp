#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <tool_tracking/kalman_filter.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/Float64MultiArray.h"

bool freshImage;
bool freshCameraInfo;
bool freshVelocity;

using namespace std;
using namespace cv_projective;

std::vector<cv::Mat> trackingImgs;  ///this should be CV_32F

void newImageCallback(const sensor_msgs::ImageConstPtr &msg, cv::Mat *outputImage) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		//cv::Mat src =  cv_bridge::toCvShare(msg,"32FC1")->image;
		//outputImage[0] = src.clone();
		cv_ptr = cv_bridge::toCvCopy(msg);
		outputImage[0] = cv_ptr->image;
		freshImage = true;
	}
	catch (cv_bridge::Exception &e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

cv::Mat segmentation(cv::Mat &InputImg) {

	cv::Mat src, src_gray;
	cv::Mat grad;

	cv::Mat res;
	src = InputImg;

	cv::resize(src, src, cv::Size(), 1, 1);

	double lowThresh = 43;

	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	cv::blur(src_gray, src_gray, cv::Size(3, 3));

	cv::Canny(src_gray, grad, lowThresh, 4 * lowThresh, 3); //use Canny segmentation

	grad.convertTo(res, CV_32FC1);

	return res;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "tracking_node");

	ros::NodeHandle nh;
	/******  initialization  ******/
	KalmanFilter UKF(&nh);

	freshCameraInfo = false;
	freshImage = false;
	//freshVelocity = false;//Moving all velocity-related things inside of the kalman.

	cv::Mat seg_left;
	cv::Mat seg_right;

	trackingImgs.resize(2);

	//TODO: get image size from camera model, or initialize segmented images,

	cv::Mat rawImage_left = cv::Mat::zeros(480, 640, CV_32FC1);
	cv::Mat rawImage_right = cv::Mat::zeros(480, 640, CV_32FC1);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber img_sub_l = it.subscribe(
		"/davinci_endo/left/image_raw",
		1,
		boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left))
	);
	image_transport::Subscriber img_sub_r = it.subscribe(
		"/davinci_endo/right/image_raw",
		1,
		boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right))
	);

	ROS_INFO("---- done subscribe -----");

//	/***testing segmentation images***/
//	cv::Size size(640, 480);
//	std::string package = ros::package::getPath("tool_tracking");
//	seg_left = cv::imread(package + "/left.png", CV_LOAD_IMAGE_GRAYSCALE);
//	//seg_left = cv::imread(package + "/particle_test.png", CV_LOAD_IMAGE_GRAYSCALE );  //testing image
//	seg_right = cv::imread(package + "/right.png", CV_LOAD_IMAGE_GRAYSCALE);
//
//    //this is to avoid the white line above
//    cv::Mat new_seg_left = seg_left.rowRange(5,480);
//    cv::Mat new_seg_right = seg_right.rowRange(5,480);
//
//    cv::resize(new_seg_left, new_seg_left,size);
//    cv::resize(new_seg_right, new_seg_right,size);


//	ToolModel newtest;
//
//	ToolModel::toolModel initial_test;
//
//	initial_test.tvec_elp(0) = 0;  //left and right (image frame)
//	initial_test.tvec_elp(1) = 0;  //up and down
//	initial_test.tvec_elp(2) = -0.0037;
//	initial_test.rvec_elp(0) = 1.11383;
//	initial_test.rvec_elp(1) = 1.11383;
//	initial_test.rvec_elp(2) = 2.24769;
//
//	newtest.computeModelPose(initial_test, 0.0, 0.0, 0.0 );

	/*** Timer set up ***/
	ros::Rate loop_rate(50);

	ros::Duration(2).sleep();
	while (nh.ok()) {
		ros::spinOnce();

		if (freshImage && UKF.freshCameraInfo){

			seg_left = segmentation(rawImage_left);  //or use image_vessselness
			seg_right = segmentation(rawImage_right);
			//ROS_INFO("AFTER SEG");
//			cv::imshow("Cam L", rawImage_left);
//			cv::imshow("Cam R", rawImage_right);`
//			cv::imshow("Seg L", seg_left);
//			cv::imshow("Seg R", seg_right);
//			cv::waitKey(10);

            UKF.update(seg_left, seg_right);
			//UKF.measureFunc(UKF.toolImage_left_arm_1, UKF.toolImage_right_arm_1, initial_test, seg_left, seg_right, UKF.Cam_left_arm_1, UKF.Cam_right_arm_1);

			freshImage = false;
			freshVelocity = false;

            ros::Duration(3).sleep();
		}

		//We want to update our filter whenever the robot is doing anything, not just when we are getting images.

        //	ToolModel::toolModel currentToolModel;
        //	convertToolModel(current_mu, currentToolModel,1);
        //	measureFunc(currentToolModel, segmented_left, segmented_right, zt);


		/*** make sure camera information is ready ***/
		//This does not seem useful at all.
		//if(!freshCameraInfo){
////			ROS_INFO("---- inside get cam info -----");
////				freshCameraInfo = true;
////		}



//		/*** if camera is ready, doing the tracking based on segemented image***/
		//if (freshImage /*&& freshVelocity && freshCameraInfo*/){
/*
////
//			cv::imshow("Rendered Image: Left", trackingImgs[0]);
//			cv::imshow("Rendered Image: Right", trackingImgs[1]);
//			cv::waitKey(50);
//
			freshImage = false;
			freshVelocity = false;
		}*/
		loop_rate.sleep();  //or cv::waitKey(10);
	}
}
