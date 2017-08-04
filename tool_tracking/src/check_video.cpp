//
// Created by ranhao on 7/19/17.
//
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <cwru_opencv_common/projective_geometry.h>

bool freshImage;

using namespace std;
using namespace cv_projective;

//get image size from camera model, or initialize segmented images
cv::Mat rawImage_left = cv::Mat::zeros(480, 640, CV_8UC3);//CV_32FC1
cv::Mat rawImage_right = cv::Mat::zeros(480, 640, CV_8UC3);

void newImageCallback(const sensor_msgs::ImageConstPtr &msg, cv::Mat *outputImage) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);
        outputImage[0] = cv_ptr->image;
        freshImage = true;
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "check_video");

    ros::NodeHandle nodeHandle;

    freshImage = false;

    image_transport::ImageTransport it(nodeHandle);
    image_transport::Subscriber img_sub_l = it.subscribe("/davinci_endo/left/image_rect", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_left)));

    image_transport::Subscriber img_sub_r = it.subscribe("/davinci_endo/right/image_rect", 1, boost::function<void(const sensor_msgs::ImageConstPtr &)>(boost::bind(newImageCallback, _1, &rawImage_right)));

    ROS_INFO("---- done subscribe -----");

/*    cv::Mat cam_mat = (cv::Mat_<double>(4,4) << -0.795366110925932, 0.598910033205905, 0.09327122662354001, -0.14643016064445,
    0.5800314064129208, 0.7967225045097124, -0.1696962532951757, -0.05682715421036513,
    -0.1759440739701311, -0.0808704082657756, -0.9810727087742884, 0.0180363085622405,
    0, 0, 0, 1);

    cv::Mat rot(3, 3, CV_64FC1);
    rot = cam_mat.colRange(0, 3).rowRange(0, 3);
    cv::Mat rot_vec(3,1,CV_64FC1);
    cv::Rodrigues(rot, rot_vec);
    ROS_INFO_STREAM("rot_vec " << rot_vec);

    cv::Mat P = cam_mat.colRange(3, 4).rowRange(0, 3);
    ROS_INFO_STREAM("P " << P); */


    while (nodeHandle.ok()) {
        ros::spinOnce();

        if (freshImage) {
            cv::imshow("raw image left: ", rawImage_left);
            cv::imshow("raw image right: ", rawImage_right);
            cv::waitKey(10);
        }
        freshImage = false;
    }

}

