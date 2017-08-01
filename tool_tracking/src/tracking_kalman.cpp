#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_rect", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));

    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_rect", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

	ROS_INFO("---- done subscribe -----");

	ros::Duration(3).sleep();

//	cv::Size size(640, 475);
//	std::string package = ros::package::getPath("tool_tracking"); ////warning: do not have one package with the same name
//	//seg_left = cv::imread(package + "/left.png", CV_LOAD_IMAGE_GRAYSCALE );
//	seg_left = cv::imread(package + "/new.png", CV_LOAD_IMAGE_GRAYSCALE );  //testing image
//	seg_right = cv::imread(package + "/right.png", CV_LOAD_IMAGE_GRAYSCALE );
//
//	cv::Mat new_seg_left = seg_left.rowRange(5,480);
//	cv::Mat new_seg_right = seg_right.rowRange(5,480);
//
//	cv::resize(new_seg_left, new_seg_left,size );
//	cv::resize(new_seg_right, new_seg_right,size );
	while (nh.ok()) {

		ros::spinOnce();

		if (freshImage){

			UKF.tool_rawImg_left = rawImage_left.clone();
			UKF.tool_rawImg_right = rawImage_right.clone();

            UKF.UKF_double_arm();

			freshImage = false;
		}

	}
}
