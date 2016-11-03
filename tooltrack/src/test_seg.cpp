/*test the segmentation method in a vedio*/

#include <ros/ros.h>
 #include <cv_bridge/cv_bridge.h>
 #include <sensor_msgs/image_encodings.h>
 #include <image_transport/image_transport.h>

#include <time.h> 

 #include "std_msgs/MultiArrayLayout.h"
 #include "std_msgs/MultiArrayDimension.h"
 #include "std_msgs/Float64MultiArray.h"

 #include <vector>
 #include <iostream>

#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>

 #include <cv_bridge/cv_bridge.h>
#include <stdlib.h>
#include <stdio.h>
#include "opencv/cv.hpp"
using namespace cv;

bool freshImage;
bool freshCameraInfo;
// bool freshVelocity;
// using namespace cv_projective;
// double Arr[6];
using namespace std;



// updates the local image.
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

    // char* original_window_name = const_cast<char*>("raw_img");
    // imshow( window_name, outputImage[0] );

}

void segmentation(cv::Mat InputImg, int code ){

	cout<<"in segmentation"<<endl;
	Mat src, src_gray;
	Mat grad;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;	

	src = InputImg;
	if( !src.data )
    { return ; }
	
	resize(src, src, Size(), 0.5, 0.5);

	if (code == 1)
	{	
		char* window_name = const_cast<char*>("Left_Segmented");
		char* original_window_name = const_cast<char*>("left_raw_img");
		namedWindow( window_name, CV_WINDOW_AUTOSIZE );

		GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

	  /// Convert it to gray
	  cvtColor( src, src_gray, CV_BGR2GRAY );

	  /// Create window


	  /// Generate grad_x and grad_y
	  Mat grad_x, grad_y;
	  Mat abs_grad_x, abs_grad_y;

	  /// Gradient X
	  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	  Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	  convertScaleAbs( grad_x, abs_grad_x );

	  /// Gradient Y
	  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	  Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	  convertScaleAbs( grad_y, abs_grad_y );

	  /// Total Gradient (approximate)
	  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad ); //grad = 0.5grad_x + 0.5grad_y
	  //cout<<"before imshow"<<endl;
	  imshow( original_window_name, src);
	  imshow( window_name, grad );
	  //cout<<"after imshow"<<endl;
	  waitKey(10);

	}

	if(code == 2){
		char* window_name = const_cast<char*>("Right_Segmented");
		char* original_window_name = const_cast<char*>("right_raw_img");
		namedWindow( window_name, CV_WINDOW_AUTOSIZE );

		GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );

	  /// Convert it to gray
	  cvtColor( src, src_gray, CV_BGR2GRAY );

	  /// Create window


	  /// Generate grad_x and grad_y
	  Mat grad_x, grad_y;
	  Mat abs_grad_x, abs_grad_y;

	  /// Gradient X
	  //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
	  Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	  convertScaleAbs( grad_x, abs_grad_x );

	  /// Gradient Y
	  //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
	  Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
	  convertScaleAbs( grad_y, abs_grad_y );

	  /// Total Gradient (approximate)
	  addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad ); //grad = 0.5grad_x + 0.5grad_y
	  //cout<<"before imshow"<<endl;
	  imshow( original_window_name, src);
	  imshow( window_name, grad );
	  //cout<<"after imshow"<<endl;
	  waitKey(10);
	}



}

 // This node takes as an input, the rectified camera image and performs stereo calibration.
int main(int argc, char** argv)
{
	ROS_INFO("---- In main node -----");

	ros::init(argc, argv, "test_segmentation");

	ros::NodeHandle nh;

	ros::Rate timer(50);

	ROS_INFO("---- Initialized ROS -----");

    freshCameraInfo = false;
    freshImage = false;

    clock_t t;
	// const std::string leftCameraTopic("/davinci_endo/left/image_raw");
	// const std::string rightCameraTopic("/davinci_endo/right/image_raw");
	// cameraProjectionMatrices cameraInfoObj(nh, leftCameraTopic, rightCameraTopic);
	// ROS_INFO("---- Connected to camera info -----");

	//TODO: get image size from camera model
    // initialize segmented images

    Mat rawImage_left = cv::Mat::zeros(640, 920, CV_32FC1);
    Mat rawImage_right = cv::Mat::zeros(640, 920, CV_32FC1);
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_raw", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left))); 
    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_raw", 1,
      boost::function< void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    ROS_INFO("---- done subscribe -----");


    while (nh.ok())
    {
        ros::spinOnce();

    	if (freshImage){
    		t = clock();
    		segmentation(rawImage_left, 1);
    		segmentation(rawImage_right, 2);
    		t = clock() - t;
    		cout<<"each segmentation peroid takes: "<<t<<endl;
    		freshImage = false;
    	}

    	timer.sleep();

	}

	if (!freshImage)
	{
		return 1;
	}
	
	
}
