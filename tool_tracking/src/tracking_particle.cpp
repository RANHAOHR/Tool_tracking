#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>
#include <tool_tracking/particle_filter.h>


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

cv::Mat segmentation(cv::Mat &InputImg) {

	cv::Mat src, src_gray;
	cv::Mat grad;

	cv::Mat res;
	src = InputImg;

	resize(src, src, cv::Size(), 1, 1);

	double lowThresh = 20;

	cv::cvtColor(src, src_gray, CV_BGR2GRAY);

	blur(src_gray, src_gray, cv::Size(3, 3));

	Canny(src_gray, grad, lowThresh, 4 * lowThresh, 3); //use Canny segmentation

	grad.convertTo(res, CV_32FC1);

	return res;

}

int main(int argc, char **argv) {

	ros::init(argc, argv, "tracking_node");

	ros::NodeHandle nh;
	//Initialize particles
	ParticleFilter Particles(&nh);

    freshImage = false;

    cv::Mat seg_left  = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat seg_right  = cv::Mat::zeros(480, 640, CV_8UC1);


    //get image size from camera model, or initialize segmented images,
    cv::Mat rawImage_left = cv::Mat::zeros(480, 640, CV_8UC3);//CV_32FC1
    cv::Mat rawImage_right = cv::Mat::zeros(480, 640, CV_8UC3);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe(
            "/davinci_endo/left/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));

    image_transport::Subscriber img_sub_r = it.subscribe(
            "/davinci_endo/right/image_raw", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    ROS_INFO("---- done subscribe -----");
    ros::Duration(2).sleep();

	while (nh.ok()) {
		ros::spinOnce();

		// if camera is ready, track segmented image
		if (freshImage) {

			Particles.raw_image_left = rawImage_left.clone();
			Particles.raw_image_right = rawImage_right.clone();

			seg_left = segmentation(rawImage_left);
			seg_right = segmentation(rawImage_right);

			cv::imshow("seg_left ", seg_left);
			cv::imshow("seg_right ", seg_right);

			ROS_INFO("---- START TRACKING -----");
			Particles.trackingTool(seg_left, seg_right);
//            cv::waitKey(10);
			freshImage = false;

			}

	}

//	// if camera is ready, track segmented image
//	if (freshImage) {
//
//		Particles.raw_image_left = rawImage_left.clone();
//		Particles.raw_image_right = rawImage_right.clone();
//
//		seg_left = segmentation(rawImage_left);
//		seg_right = segmentation(rawImage_right);
//
//		Particles.dataCollection(seg_left, seg_right);
//
//		freshImage = false;
//
//	}

	return 0;
}
