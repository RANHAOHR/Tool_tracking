/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *	Ran Hao <rxh349@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *	 copyright notice, this list of conditions and the following
 *	 disclaimer in the documentation and/or other materials provided
 *	 with the distribution.
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *	 contributors may be used to endorse or promote products derived
 *	 from this software without specific prior written permission.
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

#include <tool_tracking/kalman_filter.h>  //everything inside here
#include <opencv2/calib3d/calib3d.hpp>
#include <cwru_davinci_interface/davinci_interface.h>

using namespace std;

KalmanFilter::KalmanFilter(ros::NodeHandle *nodehandle) :
        nh_(*nodehandle), L(12) {

    ROS_INFO("Initializing UKF...");

    //initializeParticles(); Where we're going, we don't need particles.

    // initialization, just basic black image ??? how to get the size of the image
    toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);
    toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);

    toolImage_left_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);
    toolImage_right_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);

    //Set up forward kinematics.
    /***motion model params***/
    unsigned int state_dimension = 6;
    cmd_green = cv::Mat_<double>(state_dimension, 1);
    cmd_yellow = cv::Mat_<double>(state_dimension, 1);
	cmd_green_old = cv::Mat_<double>(state_dimension, 1);
	cmd_yellow_old = cv::Mat_<double>(state_dimension, 1);

    com_s1 = nh_.subscribe("/dvrk/PSM1/set_position_joint", 10, &KalmanFilter::newCommandCallback1, this);
    com_s2 = nh_.subscribe("/dvrk/PSM2/set_position_joint", 10, &KalmanFilter::newCommandCallback2, this);

    kinematics = Davinci_fwd_solver();

    //Pull in our first round of sensor data.
    davinci_interface::init_joint_feedback(nh_);
    std::vector<std::vector<double> > tmp;

    tmp.resize(2);
    tmp[0].resize(state_dimension);
    tmp[1].resize(state_dimension);
    sensor_green.resize(state_dimension);
    sensor_yellow.resize(state_dimension);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_green = tmp[0];
        sensor_yellow = tmp[1];
    }

    Eigen::Affine3d green_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_green.data()));
    Eigen::Vector3d green_trans = green_pos.translation();
    //Eigen::Vector3d green_rpy = green_pos.rotation().eulerAngles(0, 1, 2);

    cv::Mat green_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(green_pos, green_rvec);

    Eigen::Affine3d yellow_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_yellow.data()));
    Eigen::Vector3d yellow_trans = yellow_pos.translation();
    //Eigen::Vector3d yellow_rpy = yellow_pos.rotation().eulerAngles(0, 1, 2);

    cv::Mat yellow_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(yellow_pos, yellow_rvec);

    kalman_mu = cv::Mat_<double>::zeros(12, 1);

	kalman_mu.at<double>(0, 0) = green_trans[0];
	kalman_mu.at<double>(1, 0) = green_trans[1];
	kalman_mu.at<double>(2, 0) = green_trans[2];
	kalman_mu.at<double>(3, 0) = green_rvec.at<double>(0,0);
	kalman_mu.at<double>(4, 0) = green_rvec.at<double>(1,0);
	kalman_mu.at<double>(5, 0) = green_rvec.at<double>(2,0);
	kalman_mu.at<double>(6, 0) = yellow_trans[0];
	kalman_mu.at<double>(7, 0) = yellow_trans[1];
	kalman_mu.at<double>(8, 0) = yellow_trans[2];
	kalman_mu.at<double>(9, 0) = yellow_rvec.at<double>(0,0);
	kalman_mu.at<double>(10, 0) = yellow_rvec.at<double>(1,0);
	kalman_mu.at<double>(11, 0) = yellow_rvec.at<double>(2,0);

    //kalman_sigma = (cv::Mat_<double>::zeros(12, 12));
    kalman_sigma = (cv::Mat_<double>::eye(12, 12));
    for (int j = 0; j < 12; ++j) {
        kalman_sigma.at<double>(j,j) = ukfToolModel.randomNumber(0.04,0); //gaussian generator
    }

    //Temporarily populate the motion commands to give zero motion.
	cmd_green.at<double>(0, 0) = green_trans[0];
	cmd_green.at<double>(1, 0) = green_trans[1];
	cmd_green.at<double>(2, 0) = green_trans[2];
	cmd_green.at<double>(3, 0) = green_rvec.at<double>(0,0);
	cmd_green.at<double>(4, 0) = green_rvec.at<double>(1,0);
	cmd_green.at<double>(5, 0) = green_rvec.at<double>(2,0);
	cmd_yellow.at<double>(0, 0) = yellow_trans[0];
	cmd_yellow.at<double>(1, 0) = yellow_trans[1];
	cmd_yellow.at<double>(2, 0) = yellow_trans[2];
	cmd_yellow.at<double>(3, 0) = yellow_rvec.at<double>(0,0);
	cmd_yellow.at<double>(4, 0) = yellow_rvec.at<double>(1,0);
	cmd_yellow.at<double>(5, 0) = yellow_rvec.at<double>(2,0);
	cmd_green_old = cmd_green.clone();
	cmd_yellow_old = cmd_yellow.clone();
	cmd_time_green = ros::Time::now().toSec();
	cmd_time_yellow = ros::Time::now().toSec();
	cmd_time_green_old = ros::Time::now().toSec();
	cmd_time_yellow_old = ros::Time::now().toSec();

    //ROS_INFO("GREEN ARM AT (%f %f %f): %f %f %f", green_trans[0], green_trans[1], green_trans[2], green_rpy[0], green_rpy[1], green_rpy[2]);
    //ROS_INFO("YELLOW ARM AT (%f %f %f): %f %f %f", yellow_trans[0], yellow_trans[1], yellow_trans[2], yellow_rpy[0], yellow_rpy[1], yellow_rpy[2]);

    freshCameraInfo = false; //should be left and right

    projectionMat_subscriber_r = nh_.subscribe("/davinci_endo/right/camera_info", 1, &KalmanFilter::projectionRightCB, this);
    projectionMat_subscriber_l = nh_.subscribe("/davinci_endo/left/camera_info", 1, &KalmanFilter::projectionLeftCB, this);

    P_left = cv::Mat::zeros(3,4,CV_64FC1);
    P_right = cv::Mat::zeros(3,4,CV_64FC1);

    //Subscribe to the necessary transforms.
    tf::StampedTransform arm_l__cam_l_st;
    tf::StampedTransform arm_r__cam_l_st;
    tf::StampedTransform arm_l__cam_r_st;
    tf::StampedTransform arm_r__cam_r_st;

    try{
        tf::TransformListener l;
        while(!l.waitForTransform("/left_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
        l.lookupTransform("/left_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), arm_l__cam_l_st);
        while(!l.waitForTransform("/left_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
        l.lookupTransform("/left_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), arm_r__cam_l_st);
        while(!l.waitForTransform("/right_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
        l.lookupTransform("/right_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), arm_l__cam_r_st);
        while(!l.waitForTransform("/right_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
        l.lookupTransform("/right_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), arm_r__cam_r_st);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        exit(1);
    }

    //Convert to Affine3ds for storage, which is the format they will be used in for rendering.
    XformUtils xfu;

    /////arm_l is arm_1???
    arm_l__cam_l = xfu.transformTFToAffine3d(arm_l__cam_l_st);
    arm_l__cam_r = xfu.transformTFToAffine3d(arm_l__cam_r_st);
    arm_r__cam_l = xfu.transformTFToAffine3d(arm_r__cam_l_st);
    arm_r__cam_r = xfu.transformTFToAffine3d(arm_r__cam_r_st);

//    print_affine(arm_l__cam_l);

    convertEigenToMat(arm_l__cam_l, Cam_left_arm_1);
    convertEigenToMat(arm_l__cam_r, Cam_right_arm_1);
    convertEigenToMat(arm_r__cam_l, Cam_left_arm_2);
    convertEigenToMat(arm_r__cam_r, Cam_right_arm_2);

    //TODO: DEBUG: current ISSUE, z is 0 which cannot make projection???
    ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
    ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);
    ROS_INFO_STREAM("Cam_left_arm_2: " << Cam_left_arm_2);
    ROS_INFO_STREAM("Cam_right_arm_2: " << Cam_right_arm_2);

	last_update = ros::Time::now().toSec();

    fvc_yellow = false;
    fvc_green = false;

	ros::spinOnce();
	ros::spinOnce();

};

void KalmanFilter::print_affine(Eigen::Affine3d &affine) {
	cout<<"Rotation: "<<endl;
	cout<<affine.linear()<<endl;
	cout<<"origin: "<<affine.translation().transpose()<<endl;

};

void KalmanFilter::projectionRightCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight){

	P_right.at<double>(0,0) = projectionRight->P[0];
	P_right.at<double>(0,1) = projectionRight->P[1];
	P_right.at<double>(0,2) = projectionRight->P[2];
	P_right.at<double>(0,3) = projectionRight->P[3];

	P_right.at<double>(1,0) = projectionRight->P[4];
	P_right.at<double>(1,1) = projectionRight->P[5];
	P_right.at<double>(1,2) = projectionRight->P[6];
	P_right.at<double>(1,3) = projectionRight->P[7];

	P_right.at<double>(2,0) = projectionRight->P[8];
	P_right.at<double>(2,1) = projectionRight->P[9];
	P_right.at<double>(2,2) = projectionRight->P[10];
	P_right.at<double>(2,3) = projectionRight->P[11];

	//ROS_INFO_STREAM("right: " << P_right);
	freshCameraInfo = true;
};

void KalmanFilter::projectionLeftCB(const sensor_msgs::CameraInfo::ConstPtr &projectionLeft){

	P_left.at<double>(0,0) = projectionLeft->P[0];
	P_left.at<double>(0,1) = projectionLeft->P[1];
	P_left.at<double>(0,2) = projectionLeft->P[2];
	P_left.at<double>(0,3) = projectionLeft->P[3];

	P_left.at<double>(1,0) = projectionLeft->P[4];
	P_left.at<double>(1,1) = projectionLeft->P[5];
	P_left.at<double>(1,2) = projectionLeft->P[6];
	P_left.at<double>(1,3) = projectionLeft->P[7];

	P_left.at<double>(2,0) = projectionLeft->P[8];
	P_left.at<double>(2,1) = projectionLeft->P[9];
	P_left.at<double>(2,2) = projectionLeft->P[10];
	P_left.at<double>(2,3) = projectionLeft->P[11];

//	ROS_INFO_STREAM("left: " << P_left);
	freshCameraInfo = true;
};

KalmanFilter::~KalmanFilter() {

};

void KalmanFilter::newCommandCallback1(const sensor_msgs::JointState::ConstPtr& incoming){
	cmd_green_old = cmd_green.clone();
	cmd_time_green_old = cmd_time_green;

	Eigen::Affine3d cmd_green_af = kinematics.fwd_kin_solve(Vectorq7x1(incoming->position.data()));
	Eigen::Vector3d green_trans = cmd_green_af.translation();
	cv::Mat green_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(cmd_green_af, green_rvec);

	cmd_green.at<double>(0, 0) = green_trans[0];
	cmd_green.at<double>(1, 0) = green_trans[1];
	cmd_green.at<double>(2, 0) = green_trans[2];
	cmd_green.at<double>(3, 0) = green_rvec.at<double>(0,0);
	cmd_green.at<double>(4, 0) = green_rvec.at<double>(1,0);
	cmd_green.at<double>(5, 0) = green_rvec.at<double>(2,0);
	
	cmd_time_green = ros::Time::now().toSec();

    fvc_green = true;
};

void KalmanFilter::newCommandCallback2(const sensor_msgs::JointState::ConstPtr& incoming){

	cmd_yellow_old = cmd_yellow.clone();
	cmd_time_yellow_old = cmd_time_yellow;

	Eigen::Affine3d cmd_yellow_af = kinematics.fwd_kin_solve(Vectorq7x1(incoming->position.data()));

	Eigen::Vector3d yellow_trans = cmd_yellow_af.translation();
	cv::Mat yellow_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(cmd_yellow_af, yellow_rvec);

	cmd_yellow.at<double>(0, 0) = yellow_trans[0];
	cmd_yellow.at<double>(1, 0) = yellow_trans[1];
	cmd_yellow.at<double>(2, 0) = yellow_trans[2];
	cmd_yellow.at<double>(3, 0) = yellow_rvec.at<double>(0,0);
	cmd_yellow.at<double>(4, 0) = yellow_rvec.at<double>(1,0);
	cmd_yellow.at<double>(5, 0) = yellow_rvec.at<double>(2,0);
	
	cmd_time_yellow = ros::Time::now().toSec();

    fvc_yellow = true;
};

////Deprecated by KalmanFilter::update(). Archival code only.
double KalmanFilter::measureFunc( cv::Mat & toolImage_left, cv::Mat & toolImage_right, ToolModel::toolModel &toolPose, const cv::Mat &segmented_left,
	const cv::Mat &segmented_right,
	cv::Mat &Cam_left,
	cv::Mat &Cam_right) {

	//Looks mostly IP-related; need to resturcture to not be dependant on particles and instead use sigma-points.
	toolImage_left.setTo(0);
	toolImage_right.setTo(0);

	/***do the sampling and get the matching score***/
	//first get the rendered image using 3d model of the tool

    ukfToolModel.renderTool(toolImage_left, toolPose, Cam_left, P_left);
    double left = ukfToolModel.calculateMatchingScore(toolImage_left, segmented_left);  //get the matching score

    ukfToolModel.renderTool(toolImage_right, toolPose, Cam_right, P_right);
    double right = ukfToolModel.calculateMatchingScore(toolImage_right, segmented_right);

	double matchingScore = sqrt(pow(left, 2) + pow(right, 2));

	return matchingScore;

};

void KalmanFilter::update(const cv::Mat &segmented_left, const cv::Mat &segmented_right){
	ROS_INFO("In update");


    ros::spinOnce();
    //avoid coredump
    /******Find and convert our various params and inputs******/
    //Get sensor update.
    std::vector<std::vector<double> > tmp;
    int state_dimension = 6;
    tmp.resize(2);
    tmp[0].resize(state_dimension);
    tmp[1].resize(state_dimension);
    sensor_green.resize(state_dimension);
    sensor_yellow.resize(state_dimension);

    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_green = tmp[0];
        sensor_yellow = tmp[1];
    }
    double sensor_time = ros::Time::now().toSec();
    ROS_WARN("Sensor time %f", sensor_time);
    ROS_WARN("Comman time %f", cmd_time_green);

    //Convert into proper format, not want to use the euler angle prefer rodrigues for vision
    Eigen::Affine3d green_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_green.data()));
    Eigen::Vector3d green_trans = green_pos.translation();
    Eigen::Vector3d green_rpy = green_pos.rotation().eulerAngles(0, 1, 2);

    cv::Mat green_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(green_pos, green_rvec);

    Eigen::Affine3d yellow_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_yellow.data()));
    Eigen::Vector3d yellow_trans = yellow_pos.translation();
    Eigen::Vector3d yellow_rpy = yellow_pos.rotation().eulerAngles(0, 1, 2);
    cv::Mat yellow_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(yellow_pos, yellow_rvec);

    cv::Mat zt = cv::Mat_<double>(L, 1);
    zt.at<double>(0, 0) = green_trans[0];
    zt.at<double>(1, 0) = green_trans[1];
    zt.at<double>(2, 0) = green_trans[2];
    zt.at<double>(3, 0) = green_rvec.at<double>(0,0);
    zt.at<double>(4, 0) = green_rvec.at<double>(1,0);
    zt.at<double>(5, 0) = green_rvec.at<double>(2,0);
    zt.at<double>(6, 0) = yellow_trans[0];
    zt.at<double>(7, 0) = yellow_trans[1];
    zt.at<double>(8, 0) = yellow_trans[2];
    zt.at<double>(9, 0) = yellow_rvec.at<double>(0,0);
    zt.at<double>(10, 0) = yellow_rvec.at<double>(1,0);
    zt.at<double>(11, 0) = yellow_rvec.at<double>(2,0);

    ROS_INFO("SENSOR GREEN ARM AT (%f %f %f): %f %f %f", zt.at<double>(0, 0), zt.at<double>(1, 0),zt.at<double>(2, 0),zt.at<double>(3, 0),zt.at<double>(4, 0), zt.at<double>(5, 0));
    ROS_INFO("SENSOR YELLOW ARM AT (%f %f %f): %f %f %f", zt.at<double>(6, 0), zt.at<double>(7, 0),zt.at<double>(8, 0),zt.at<double>(9, 0),zt.at<double>(10, 0), zt.at<double>(11, 0));

    cv::Mat sigma_t_last = kalman_sigma.clone();
    cv::Mat mu_t_last = kalman_mu.clone();

    //****Generate the sigma points.****
    double lambda = alpha * alpha * (L + k) - L;
    double gamma = sqrt(L + lambda);

    ///get the square root for sigma point generation using SVD decomposition
    cv::Mat root_sigma_t_last = cv::Mat_<double>::zeros(L, 1);

    cv::Mat s = cv::Mat_<double>::zeros(L, 1);  //allocate space for SVD
    cv::Mat vt = cv::Mat_<double>::zeros(L, L);  //allocate space for SVD
    cv::Mat u = cv::Mat_<double>::zeros(L, L);  //allocate space for SVD

    cv::SVD::compute(sigma_t_last, s, u, vt);//The actual square root gets saved into s

    root_sigma_t_last = s.clone(); //store that back into the designated square root term
    ROS_INFO_STREAM("root_sigma_t_last" << root_sigma_t_last);
    //Populate the sigma points:
    std::vector<cv::Mat_<double> > sigma_pts_last;
    sigma_pts_last.resize(2*L + 1);

    sigma_pts_last[0] = mu_t_last.clone();//X_0
    for (int i = 1; i <= L; i++) {
        sigma_pts_last[i] = sigma_pts_last[0] + (gamma * root_sigma_t_last);
        sigma_pts_last[i + L] = sigma_pts_last[0] - (gamma * root_sigma_t_last);
    }

    //Compute their weights:
    std::vector<double> w_m;
    w_m.resize(2*L + 1);
    std::vector<double> w_c;
    w_c.resize(2*L + 1);
    w_m[0] = lambda / (L + lambda);
    w_c[0] = lambda / (L + lambda) + (1.0 - (alpha * alpha) + beta);
    for(int i = 1; i < 2 * L + 1; i++){
        w_m[i] = 1.0 / (2.0 * (L + lambda));
        w_c[i] = 1.0 / (2.0 * (L + lambda));
    }

    /*****Update sigma points based on motion model******/
    std::vector<cv::Mat_<double> > sigma_pts_bar;
    sigma_pts_bar.resize(2*L + 1);

   // ROS_WARN("S pos green (%f, %f, %f, %f, %f, %f)", zt.at<double>(0, 0), zt.at<double>(1, 0), zt.at<double>(2, 0), zt.at<double>(3, 0), zt.at<double>(4, 0), zt.at<double>(5, 0));
    //ROS_WARN("C pos green (%f, %f, %f, %f, %f, %f)", cmd_green.at<double>(0, 0), cmd_green.at<double>(1, 0), cmd_green.at<double>(2, 0), cmd_green.at<double>(3, 0), cmd_green.at<double>(4, 0), cmd_green.at<double>(5, 0));

    for(int i = 0; i < 2 * L + 1; i++){
        g(sigma_pts_bar[i], sigma_pts_last[i], zt);
        //ROS_ERROR("%f %f %f %f %f %f", sigma_pts_bar[i].at<double>(1, 1),sigma_pts_bar[i].at<double>(2, 1),sigma_pts_bar[i].at<double>(3, 1),sigma_pts_bar[i].at<double>(4, 1),sigma_pts_bar[i].at<double>(5, 1),sigma_pts_bar[i].at<double>(6, 1));
    }
    last_update = ros::Time::now().toSec();
    fvc_green = false;
    fvc_yellow = false;

    //TODO: Calculate a second set of weights based on the matching score, to influence the effect of the sigma points.

    /*****Create the predicted mus and sigmas.*****/
    cv::Mat mu_bar = cv::Mat_<double>::zeros(L, 1);
    for(int i = 0; i < 2 * L + 1; i++){
        mu_bar = mu_bar + w_m[i] * sigma_pts_bar[i];
    }
    cv::Mat sigma_bar = cv::Mat_<double>::zeros(L, L);
    for(int i = 0; i < 2 * L + 1; i++){
        sigma_bar = sigma_bar + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((sigma_pts_bar[i] - mu_bar).t());
    }
    //ROS_INFO_STREAM("sigma_bar "<< sigma_bar);
    /*****Render each sigma point and compute its matching score.*****/

    std::vector<double> mscores;
    mscores.resize(2*L + 1);

    computeSigmaMeasures(mscores, sigma_pts_bar, segmented_left, segmented_right);
    //TODO:intend to use mscores as wit=eights to bias the mu of z_hat

    /*****Correction Step: Move the sigma points through the measurement function.*****/
    std::vector<cv::Mat_<double> > Z_bar;
    Z_bar.resize(2 * L + 1);
    cv::Mat sig_delt = zt - sigma_pts_bar[0];
    for(int i = 0; i < 2 * L + 1; i++){

        h(Z_bar[i], sigma_pts_bar[i], sig_delt);
    }

    /*****Calculate derived variance statistics.*****/
    cv::Mat z_caret = cv::Mat_<double>::zeros(L, 1);
    for(int i = 0; i < 2 * L + 1; i++){
        z_caret = z_caret + w_m[i] * Z_bar[i];
    }

    ROS_INFO_STREAM("correction? " << zt - z_caret);

    cv::Mat S = cv::Mat_<double>::zeros(L, L);
    for(int i = 0; i < 2 * L + 1; i++){
        S = S + w_c[i] * (Z_bar[i] - z_caret) * ((Z_bar[i] - z_caret).t());
    }

    cv::Mat sigma_xz = cv::Mat_<double>::zeros(L, L);
    for(int i = 0; i < 2 * L + 1; i++){
        sigma_xz = sigma_xz + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((Z_bar[i] - z_caret).t());
    }

    cv::Mat K = sigma_xz * S.inv();

    /*****Update our mu and sigma.*****/
    kalman_mu = mu_bar + (zt - z_caret);
    kalman_sigma = sigma_bar - S;
//    kalman_mu = mu_bar + K * (zt - z_caret);
//    kalman_sigma = sigma_bar - K * S * K.t();
    ROS_WARN("GREEN ARM AT (%f %f %f): %f %f %f", kalman_mu.at<double>(0, 0), kalman_mu.at<double>(1, 0),kalman_mu.at<double>(2, 0),kalman_mu.at<double>(3, 0),kalman_mu.at<double>(4, 0), kalman_mu.at<double>(5, 0));
    ROS_WARN("YELLOW ARM AT (%f %f %f): %f %f %f", kalman_mu.at<double>(6, 0), kalman_mu.at<double>(7, 0),kalman_mu.at<double>(8, 0),kalman_mu.at<double>(9, 0),kalman_mu.at<double>(10, 0), kalman_mu.at<double>(11, 0));

};

void KalmanFilter::g(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in, const cv::Mat & zt){
	cv::Mat delta_green = cv::Mat_<double>::zeros(6, 1);
	cv::Mat delta_yellow = cv::Mat_<double>::zeros(6, 1);
	if(fvc_green){
		cv::Mat sensor_green = cv::Mat_<double>::zeros(6, 1);
		sensor_green.at<double>(0, 0) = zt.at<double>(0, 0);
		sensor_green.at<double>(1, 0) = zt.at<double>(1, 0);
		sensor_green.at<double>(2, 0) = zt.at<double>(2, 0);
		sensor_green.at<double>(3, 0) = zt.at<double>(3, 0);
		sensor_green.at<double>(4, 0) = zt.at<double>(4, 0);
		sensor_green.at<double>(5, 0) = zt.at<double>(5, 0);
		delta_green = (sensor_green - cmd_green);
		//ROS_INFO("GREEN DELTAS %f %f %f %f %f %f", delta_green.at<double>(0, 0), delta_green.at<double>(1, 0), delta_green.at<double>(2, 0), delta_green.at<double>(3, 0), delta_green.at<double>(4, 0), delta_green.at<double>(5, 0));
	}
	if(fvc_yellow){
		cv::Mat sensor_yellow = cv::Mat_<double>::zeros(6, 1);
		sensor_yellow.at<double>(0, 0) = zt.at<double>(6, 0);
		sensor_yellow.at<double>(1, 0) = zt.at<double>(7, 0);
		sensor_yellow.at<double>(2, 0) = zt.at<double>(8, 0);
		sensor_yellow.at<double>(3, 0) = zt.at<double>(9, 0);
		sensor_yellow.at<double>(4, 0) = zt.at<double>(10, 0);
		sensor_yellow.at<double>(5, 0) = zt.at<double>(11, 0);
		delta_yellow = (sensor_yellow - cmd_yellow);
		//ROS_INFO("YELLOW DELTAS %f %f %f %f %f %f", delta_yellow.at<double>(0, 0), delta_yellow.at<double>(1, 0), delta_yellow.at<double>(2, 0), delta_yellow.at<double>(3, 0), delta_yellow.at<double>(4, 0), delta_yellow.at<double>(5, 0));
	}
	cv::Mat delta_all = cv::Mat_<double>::zeros(12, 1);
	vconcat(delta_green, delta_yellow, delta_all);
	double current_time = ros::Time::now().toSec();

	sigma_point_out = sigma_point_in.clone();
	sigma_point_out = sigma_point_in + delta_all;
};

/***this function should compute the matching score for each sigma points: for future use****/
void KalmanFilter::computeSigmaMeasures(std::vector<double> & measureWeights, const std::vector<cv::Mat_<double> > & sigma_point_in, const cv::Mat &segmented_left, const cv::Mat &segmented_right){
	//ROS_ERROR("IN CSM FUNC: %lu, %lu", measureWeights.size(), sigma_point_in.size());
	double total = 0.0;
	for (int i = 0; i < sigma_point_in.size() ; i++) {
		measureWeights[i] = matching_score(sigma_point_in[i], segmented_left, segmented_right );
		total += measureWeights[i];
	}
    if(total > 0.0){
        //normalization of measurement weights
        for (int j = 0; j < sigma_point_in.size(); j++) {
            measureWeights[j] = measureWeights[j] / total;
        }
    }else{
        ROS_ERROR("Cannot find good measurement scores");
    }

};

double KalmanFilter::matching_score(const cv::Mat & stat, const cv::Mat &segmented_left, const cv::Mat &segmented_right) {

    //ROS_INFO_STREAM("stat IS: " << stat);
    //Convert our state into Eigen::Affine3ds; one for each arm
    Eigen::Affine3d arm1 =
            Eigen::Translation3d(stat.at<double>(0, 0), stat.at<double>(1, 0), stat.at<double>(2, 0))
            *
            Eigen::AngleAxisd(stat.at<double>(3, 0), Eigen::Vector3d::UnitX())
            *
            Eigen::AngleAxisd(stat.at<double>(4, 0), Eigen::Vector3d::UnitY())
            *
            Eigen::AngleAxisd(stat.at<double>(5, 0), Eigen::Vector3d::UnitZ());
    Eigen::Affine3d arm2 =
            Eigen::Translation3d(stat.at<double>(6, 0), stat.at<double>(7, 0), stat.at<double>(8, 0))
            *
            Eigen::AngleAxisd(stat.at<double>(9, 0), Eigen::Vector3d::UnitX())
            *
            Eigen::AngleAxisd(stat.at<double>(10, 0), Eigen::Vector3d::UnitY())
            *
            Eigen::AngleAxisd(stat.at<double>(11, 0), Eigen::Vector3d::UnitZ());

    //Convert them into tool models
    ToolModel::toolModel arm_1;
    ToolModel::toolModel arm_2;
    convertToolModel(arm1, arm_1);
    convertToolModel(arm2, arm_2);

//    //this is the POSE of the ELLIPSE part of the tool for arm 1
//They are also really annoying.
   /* ROS_INFO_STREAM(" ARM 1 tvec(0)" << arm_1.tvec_elp(0) );
    ROS_INFO_STREAM(" ARM 1 tvec(1)" << arm_1.tvec_elp(1) );
    ROS_INFO_STREAM(" ARM 1 tvec(2)" << arm_1.tvec_elp(2) );
    ROS_INFO_STREAM(" ARM 1 rvec(0)" << arm_1.rvec_elp(0) );
    ROS_INFO_STREAM(" ARM 1 rvec(1)" << arm_1.rvec_elp(1) );
    ROS_INFO_STREAM(" ARM 1 rvec(2)" << arm_1.rvec_elp(2) );

    //this is the POSE of the ELLIPSE part of the tool for arm 2
    ROS_INFO_STREAM(" ARM 2 tvec(0)" << arm_2.tvec_elp(0) );
    ROS_INFO_STREAM(" ARM 2 tvec(1)" << arm_2.tvec_elp(1) );
    ROS_INFO_STREAM(" ARM 2 tvec(2)" << arm_2.tvec_elp(2) );
    ROS_INFO_STREAM(" ARM 2 rvec(0)" << arm_2.rvec_elp(0) );
    ROS_INFO_STREAM(" ARM 2 rvec(1)" << arm_2.rvec_elp(1) );
    ROS_INFO_STREAM(" ARM 2 rvec(2)" << arm_2.rvec_elp(2) );*/

    //Render the tools and compute the matching score

    double matchingScore_arm_1 = measureFunc(toolImage_left_arm_1, toolImage_right_arm_1, arm_1, segmented_left, segmented_right, Cam_left_arm_1, Cam_right_arm_1);
    double matchingScore_arm_2 = measureFunc(toolImage_left_arm_2, toolImage_right_arm_2, arm_2, segmented_left, segmented_right, Cam_left_arm_2, Cam_right_arm_2);

//    cv::imshow("Render arm 1 Left cam" ,toolImage_left_arm_1 );
//    cv::imshow("Render arm 1 Right cam" ,toolImage_right_arm_1 );
//    cv::imshow("Render arm 2 Left cam" ,toolImage_left_arm_2 );
//    cv::imshow("Render arm 2 Right cam" ,toolImage_right_arm_2 );
//    cv::waitKey(10);

    double result = (matchingScore_arm_1 + matchingScore_arm_2) / 2;

    return result;

};

void KalmanFilter::h(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in, const cv::Mat & sigma_delt){
	//Assumes a non-functionally-distorted sensor. Largely substituted for by the matching-score weighting.
	//ROS_INFO_STREAM("DELT " << sigma_delt);
	cv::Mat temp = sigma_point_in.clone();
	sigma_point_out = temp; // + sigma_delt;
};

/******from eigen to opencv matrix****/
void KalmanFilter::convertEigenToMat(const Eigen::Affine3d & trans, cv::Mat & outputMatrix){

	outputMatrix = cv::Mat::eye(4,4,CV_64FC1);

	Eigen::Vector3d pos = trans.translation();
	Eigen::Matrix3d rot = trans.linear();

	//this is the last col, translation
	outputMatrix.at<double>(0,3) = pos(0);
	outputMatrix.at<double>(1,3) = pos(1);
	outputMatrix.at<double>(2,3) = pos(2);

	Eigen::Vector3d col_0, col_1, col_2;
	//this is the first col, rotation x
	col_0 = rot.col(0);
	outputMatrix.at<double>(0,0) = col_0(0);
	outputMatrix.at<double>(1,0) = col_0(1);
	outputMatrix.at<double>(2,0) = col_0(2);

	//this is the second col, rotation y
	col_1 = rot.col(1);
	outputMatrix.at<double>(0,1) = col_1(0);
	outputMatrix.at<double>(1,1) = col_1(1);
	outputMatrix.at<double>(2,1) = col_1(2);

	//this is the third col, rotation z
	col_2 = rot.col(2);
	outputMatrix.at<double>(0,2) = col_2(0);
	outputMatrix.at<double>(1,2) = col_2(1);
	outputMatrix.at<double>(2,2) = col_2(2);

};

//TODO: STILL NEED OTHER DIMENSIONS
void KalmanFilter::convertToolModel(const Eigen::Affine3d & trans, ToolModel::toolModel &toolModel){
    Eigen::Vector3d pos = trans.translation();
    ////Not use euler angles or Rodrigues angles
//    Eigen::Vector3d rpy = trans.rotation().eulerAngles(0, 1, 2);
//    ROS_INFO_STREAM("RPY " << rpy);
//    toolModel.tvec_elp(0) = pos[0];
//    toolModel.tvec_elp(1) = pos[1];
//    toolModel.tvec_elp(2) = pos[2];
//    toolModel.rvec_elp(0) = rpy[0];
//    toolModel.rvec_elp(1) = rpy[1];
//    toolModel.rvec_elp(2) = rpy[2];

    Eigen::Matrix3d rot_affine = trans.rotation();

    cv::Mat rot(3,3,CV_64FC1);
    rot.at<double>(0,0) = rot_affine(0,0);
    rot.at<double>(0,1) = rot_affine(0,1);
    rot.at<double>(0,2) = rot_affine(0,2);
    rot.at<double>(1,0) = rot_affine(1,0);
    rot.at<double>(1,1) = rot_affine(1,1);
    rot.at<double>(1,2) = rot_affine(1,2);
    rot.at<double>(2,0) = rot_affine(2,0);
    rot.at<double>(2,1) = rot_affine(2,1);
    rot.at<double>(2,2) = rot_affine(2,2);

    cv::Mat rot_vec(3,1, CV_64FC1);
    cv::Rodrigues(rot, rot_vec );
    //ROS_INFO_STREAM("rot_vec " << rot_vec);

    //TODO: need to add the joint angles
    //Do we even HAVE that?
    toolModel.tvec_elp(0) = pos[0];
    toolModel.tvec_elp(1) = pos[1];
    toolModel.tvec_elp(2) = pos[2];
    toolModel.rvec_elp(0) = rot_vec.at<double>(0,0);
    toolModel.rvec_elp(1) = rot_vec.at<double>(1,0);
    toolModel.rvec_elp(2) = rot_vec.at<double>(2,0);

    ukfToolModel.computeModelPose(toolModel, 0.0, 0.3, 0.0);
};

void KalmanFilter::computeRodriguesVec(const Eigen::Affine3d & trans, cv::Mat rot_vec){
//	Eigen::Vector3d rpy = trans.rotation().eulerAngles(0, 1, 2);
//	ROS_INFO_STREAM("RPY " << rpy);

	Eigen::Matrix3d rot_affine = trans.rotation();

	cv::Mat rot(3,3,CV_64FC1);
	rot.at<double>(0,0) = rot_affine(0,0);
	rot.at<double>(0,1) = rot_affine(0,1);
	rot.at<double>(0,2) = rot_affine(0,2);
	rot.at<double>(1,0) = rot_affine(1,0);
	rot.at<double>(1,1) = rot_affine(1,1);
	rot.at<double>(1,2) = rot_affine(1,2);
	rot.at<double>(2,0) = rot_affine(2,0);
	rot.at<double>(2,1) = rot_affine(2,1);
	rot.at<double>(2,2) = rot_affine(2,2);

	rot_vec = cv::Mat::zeros(3,1, CV_64FC1);
	cv::Rodrigues(rot, rot_vec );
	//ROS_INFO_STREAM("rot_vec " << rot_vec);
};