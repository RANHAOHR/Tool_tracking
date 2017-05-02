#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <tool_tracking/kalman_filter.h>

bool freshImage;

using namespace std;
using namespace cv_projective;

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

int main(int argc, char **argv) {

	ros::init(argc, argv, "tracking_node");

	ros::NodeHandle nh;
	/******  initialization  ******/
	KalmanFilter UKF(&nh);

	//freshVelocity = false;//Moving all velocity-related things inside of the kalman.

	cv::Mat rawImage_left = cv::Mat::zeros(480, 640, CV_32FC1);
	cv::Mat rawImage_right = cv::Mat::zeros(480, 640, CV_32FC1);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1,boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));

	image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

	ROS_INFO("---- done subscribe -----");

	/*** Timer set up ***/
	ros::Rate loop_rate(50);

	ros::Duration(2).sleep();

	int tracking_iteration = 0;
	while (nh.ok()) {
		ros::spinOnce();

		if (freshImage){

			UKF.tool_rawImg_left = rawImage_left.clone();
			UKF.tool_rawImg_right = rawImage_right.clone();

            UKF.UKF_double_arm();

			freshImage = false;
			tracking_iteration +=1;
		}

		//get ready for dynamic tracking:temp solution or can use client goal
		if(tracking_iteration == 5){  //takes 5 iterations to converge
			ROS_INFO("INSIDE dynamic ");
			UKF.getCourseEstimation();
			tracking_iteration = 0;
            //cv::waitKey();
		}

		//loop_rate.sleep();  //or cv::waitKey(10);
	}
}
