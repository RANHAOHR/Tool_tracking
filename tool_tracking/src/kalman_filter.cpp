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

using namespace std;

/***********ROBOT ARM NAMING PROTOCOL*********/
/**********************************************
The robot arms are to be referred to by color or number. Arm 1 is green and Arm 2 is yellow.
The arms are not under any circumstances to be referred to by 'left' or 'right'.
The cameras, conversely, are to be referred to *ONLY* by 'left' and 'right'- never 'one' or 'two'.
**********************************************/

KalmanFilter::KalmanFilter(ros::NodeHandle *nodehandle) :
		nh_(*nodehandle), L(12) {

	ROS_INFO("Initializing UKF...");

	// initialization, just basic black image ??? how to get the size of the image
	toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);

	toolImage_left_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);

	toolImage_cam_left = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_cam_right = cv::Mat::zeros(480, 640, CV_8UC3);

	tool_rawImg_left = cv::Mat::zeros(480, 640, CV_8UC3);
	tool_rawImg_right =cv::Mat::zeros(480, 640, CV_8UC3);

	//Set up forward kinematics.
	/***motion model params***/
	unsigned int state_dimension = 6;
	cmd_1 = cv::Mat_<double>(state_dimension, 1);
	cmd_2 = cv::Mat_<double>(state_dimension, 1);
	cmd_1_old = cv::Mat_<double>(state_dimension, 1);
	cmd_2_old = cv::Mat_<double>(state_dimension, 1);

	com_s1 = nh_.subscribe("/dvrk/PSM1/set_position_joint", 10, &KalmanFilter::newCommandCallback1, this);
	com_s2 = nh_.subscribe("/dvrk/PSM2/set_position_joint", 10, &KalmanFilter::newCommandCallback2, this);

	kinematics = Davinci_fwd_solver();

	//Pull in our first round of sensor data.
	davinci_interface::init_joint_feedback(nh_);
	std::vector<std::vector<double> > tmp;

	tmp.resize(2);
	tmp[0].resize(state_dimension);
	tmp[1].resize(state_dimension);
	sensor_1.resize(state_dimension);
	sensor_2.resize(state_dimension);
	if(davinci_interface::get_fresh_robot_pos(tmp)){
		sensor_1 = tmp[0];
		sensor_2 = tmp[1];
	}

	Eigen::Affine3d a1_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_1.data()));
	Eigen::Vector3d a1_trans = a1_pos.translation();
	//Eigen::Vector3d green_rpy = green_pos.rotation().eulerAngles(0, 1, 2);

	cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(a1_pos, a1_rvec);

	Eigen::Affine3d a2_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_2.data()));
	Eigen::Vector3d a2_trans = a2_pos.translation();
	//Eigen::Vector3d yellow_rpy = yellow_pos.rotation().eulerAngles(0, 1, 2);

	cv::Mat a2_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(a2_pos, a2_rvec);

	kalman_mu = cv::Mat_<double>::zeros(12, 1);

	//TODO:HERE has the orientation CHANGED weired!!!!!!!!!!!!!!!!!!!!!!!!!!
	kalman_mu.at<double>(0 , 0) = a1_trans[0];
	kalman_mu.at<double>(1 , 0) = a1_trans[1];
	kalman_mu.at<double>(2 , 0) = a1_trans[2];
	kalman_mu.at<double>(3 , 0) = a2_rvec.at<double>(0,0);
	kalman_mu.at<double>(4 , 0) = a2_rvec.at<double>(1,0);
	kalman_mu.at<double>(5 , 0) = a2_rvec.at<double>(2,0);
	kalman_mu.at<double>(6 , 0) = a2_trans[0];
	kalman_mu.at<double>(7 , 0) = a2_trans[1];
	kalman_mu.at<double>(8 , 0) = a2_trans[2];
	kalman_mu.at<double>(9 , 0) = a1_rvec.at<double>(0,0);
	kalman_mu.at<double>(10, 0) = a1_rvec.at<double>(1,0);
	kalman_mu.at<double>(11, 0) = a1_rvec.at<double>(2,0);

	kalman_sigma = (cv::Mat_<double>::eye(12, 12));
	for (int j = 0; j < 12; ++j) {
		kalman_sigma.at<double>(j,j) = ukfToolModel.randomNumber(0.04,0); //gaussian generator
	}

	//Temporarily populate the motion commands to give zero motion.
	cmd_1.at<double>(0, 0) = a1_trans[0];
	cmd_1.at<double>(1, 0) = a1_trans[1];
	cmd_1.at<double>(2, 0) = a1_trans[2];
	cmd_1.at<double>(3, 0) = a1_rvec.at<double>(0,0);
	cmd_1.at<double>(4, 0) = a1_rvec.at<double>(1,0);
	cmd_1.at<double>(5, 0) = a1_rvec.at<double>(2,0);
	cmd_2.at<double>(0, 0) = a2_trans[0];
	cmd_2.at<double>(1, 0) = a2_trans[1];
	cmd_2.at<double>(2, 0) = a2_trans[2];
	cmd_2.at<double>(3, 0) = a2_rvec.at<double>(0,0);
	cmd_2.at<double>(4, 0) = a2_rvec.at<double>(1,0);
	cmd_2.at<double>(5, 0) = a2_rvec.at<double>(2,0);
	cmd_1_old = cmd_1.clone();
	cmd_2_old = cmd_2.clone();
	cmd_time_1 = ros::Time::now().toSec();
	cmd_time_2 = ros::Time::now().toSec();
	cmd_time_1_old = ros::Time::now().toSec();
	cmd_time_2_old = ros::Time::now().toSec();

	//ROS_INFO("GREEN ARM AT (%f %f %f): %f %f %f", green_trans[0], green_trans[1], green_trans[2], green_rpy[0], green_rpy[1], green_rpy[2]);
	//ROS_INFO("YELLOW ARM AT (%f %f %f): %f %f %f", yellow_trans[0], yellow_trans[1], yellow_trans[2], yellow_rpy[0], yellow_rpy[1], yellow_rpy[2]);

	//freshCameraInfo = false; //should be left and right
	
	//The projection matrix from the simulation does not accurately reflect the Da Vinci robot. We are hardcoding the matrix from the da vinci itself.
	//projectionMat_subscriber_r = nh_.subscribe("/davinci_endo/right/camera_info", 1, &KalmanFilter::projectionRightCB, this);
	//projectionMat_subscriber_l = nh_.subscribe("/davinci_endo/left/camera_info", 1, &KalmanFilter::projectionLeftCB, this);

	P_left = cv::Mat::zeros(3,4,CV_64FC1);
	P_right = cv::Mat::zeros(3,4,CV_64FC1);
	
	cv::Mat P_l(3, 4, CV_64FC1);
	P_l.at<double>(0, 0) = 893.7852590197848;
	P_l.at<double>(1, 0) = 0;
	P_l.at<double>(2, 0) = 0;

	P_l.at<double>(0, 1) = 0;
	P_l.at<double>(1, 1) = 893.7852590197848;
	P_l.at<double>(2, 1) = 0;

	P_l.at<double>(0, 2) = 288.4443244934082; // horiz
	P_l.at<double>(1, 2) = 259.7727756500244; //verticle
	P_l.at<double>(2, 2) = 1;

	P_l.at<double>(0, 3) = 0;
	P_l.at<double>(1, 3) = 0;
	P_l.at<double>(2, 3) = 0;

	cv::Mat P_r(3, 4, CV_64FC1);
	P_r.at<double>(0, 0) = 893.7852590197848;
	P_r.at<double>(1, 0) = 0;
	P_r.at<double>(2, 0) = 0;

	P_r.at<double>(0, 1) = 0;
	P_r.at<double>(1, 1) = 893.7852590197848;
	P_r.at<double>(2, 1) = 0;

	P_r.at<double>(0, 2) = 288.4443244934082; // horiz
	P_r.at<double>(1, 2) = 259.7727756500244; //verticle
	P_r.at<double>(2, 2) = 1;

	P_r.at<double>(0, 3) = 4.732953897952732;
	P_r.at<double>(1, 3) = 0;
	P_r.at<double>(2, 3) = 0;
	
	P_left = P_l.clone();
	P_right = P_r.clone();

	//Subscribe to the necessary transforms.
	tf::StampedTransform arm_1__cam_l_st;
	tf::StampedTransform arm_2__cam_l_st;
	tf::StampedTransform arm_1__cam_r_st;
	tf::StampedTransform arm_2__cam_r_st;

	try{
		tf::TransformListener l;
		while(!l.waitForTransform("/left_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
		l.lookupTransform("/left_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), arm_1__cam_l_st);
		while(!l.waitForTransform("/left_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
		l.lookupTransform("/left_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), arm_2__cam_l_st);
		while(!l.waitForTransform("/right_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
		l.lookupTransform("/right_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), arm_1__cam_r_st);
		while(!l.waitForTransform("/right_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
		l.lookupTransform("/right_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), arm_2__cam_r_st);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		exit(1);
	}

	//Convert to Affine3ds for storage, which is the format they will be used in for rendering.
	XformUtils xfu;
	arm_1__cam_l = xfu.transformTFToAffine3d(arm_1__cam_l_st);//.inverse();
	arm_1__cam_r = xfu.transformTFToAffine3d(arm_1__cam_r_st);//.inverse();
	arm_2__cam_l = xfu.transformTFToAffine3d(arm_2__cam_l_st);//.inverse();
	arm_2__cam_r = xfu.transformTFToAffine3d(arm_2__cam_r_st);//.inverse();

//	print_affine(arm_l__cam_l);

	convertEigenToMat(arm_1__cam_l, Cam_left_arm_1);
	convertEigenToMat(arm_1__cam_r, Cam_right_arm_1);
	convertEigenToMat(arm_2__cam_l, Cam_left_arm_2);
	convertEigenToMat(arm_2__cam_r, Cam_right_arm_2);

	ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);
	ROS_INFO_STREAM("Cam_left_arm_2: " << Cam_left_arm_2);
	ROS_INFO_STREAM("Cam_right_arm_2: " << Cam_right_arm_2);

	fvc_1 = false;
	fvc_2 = false;

	ros::spinOnce();
	
	last_update = ros::Time::now().toSec();
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

	ROS_INFO_STREAM("right: " << P_right);
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

	ROS_INFO_STREAM("left: " << P_left);
	freshCameraInfo = true;
};

KalmanFilter::~KalmanFilter() {
};

void KalmanFilter::newCommandCallback1(const sensor_msgs::JointState::ConstPtr& incoming){
	cmd_1_old = cmd_1.clone();
	cmd_time_1_old = cmd_time_1;

	Eigen::Affine3d cmd_1_af = kinematics.fwd_kin_solve(Vectorq7x1(incoming->position.data()));
	Eigen::Vector3d a1_trans = cmd_1_af.translation();
	cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(cmd_1_af, a1_rvec);

	cmd_1.at<double>(0, 0) = a1_trans[0];
	cmd_1.at<double>(1, 0) = a1_trans[1];
	cmd_1.at<double>(2, 0) = a1_trans[2];
	cmd_1.at<double>(3, 0) = a1_rvec.at<double>(0,0);
	cmd_1.at<double>(4, 0) = a1_rvec.at<double>(1,0);
	cmd_1.at<double>(5, 0) = a1_rvec.at<double>(2,0);
	
	cmd_time_1 = ros::Time::now().toSec();

	fvc_1 = true;
};

void KalmanFilter::newCommandCallback2(const sensor_msgs::JointState::ConstPtr& incoming){

	cmd_2_old = cmd_2.clone();
	cmd_time_2_old = cmd_time_2;

	Eigen::Affine3d cmd_2_af = kinematics.fwd_kin_solve(Vectorq7x1(incoming->position.data()));

	Eigen::Vector3d a2_trans = cmd_2_af.translation();
	cv::Mat a2_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(cmd_2_af, a2_rvec);

	cmd_2.at<double>(0, 0) = a2_trans[0];
	cmd_2.at<double>(1, 0) = a2_trans[1];
	cmd_2.at<double>(2, 0) = a2_trans[2];
	cmd_2.at<double>(3, 0) = a2_rvec.at<double>(0,0);
	cmd_2.at<double>(4, 0) = a2_rvec.at<double>(1,0);
	cmd_2.at<double>(5, 0) = a2_rvec.at<double>(2,0);
	
	cmd_time_2 = ros::Time::now().toSec();

	fvc_2 = true;
};

////Deprecated by KalmanFilter::update()?
double KalmanFilter::measureFunc(
	cv::Mat & toolImage_left,
	cv::Mat & toolImage_right,
	ToolModel::toolModel &toolPose,
	const cv::Mat &segmented_left,
	const cv::Mat &segmented_right,
	cv::Mat &Cam_left,
	cv::Mat &Cam_right
) {

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

double KalmanFilter::measureFuncSameCam(cv::Mat & toolImage_cam, ToolModel::toolModel &toolPose_left, ToolModel::toolModel &toolPose_right,
										const cv::Mat &segmented_cam, const cv::Mat & Projection_mat, cv::Mat &raw_img, cv::Mat &Cam_matrix_tool_left, cv::Mat &Cam_matrix_tool_right) {

	toolImage_cam.setTo(0);
	/***do the sampling and get the matching score***/
	//first get the rendered image using 3d model of the tool

	ukfToolModel.renderTool(toolImage_cam, toolPose_left, Cam_matrix_tool_left, Projection_mat);
	ukfToolModel.renderTool(toolImage_cam, toolPose_right, Cam_matrix_tool_right, Projection_mat);

	ukfToolModel.renderTool(raw_img, toolPose_left, Cam_matrix_tool_left, Projection_mat);
	ukfToolModel.renderTool(raw_img, toolPose_right, Cam_matrix_tool_right, Projection_mat);

	double matchingScore = ukfToolModel.calculateMatchingScore(toolImage_cam, segmented_cam);

	return matchingScore;
};

void KalmanFilter::update(const cv::Mat &segmented_left, const cv::Mat &segmented_right){
	//avoid coredump
	ros::spinOnce();
	
	/******Find and convert our various params and inputs******/
	//Get sensor update.
	std::vector<std::vector<double> > tmp;
	int state_dimension = 6;
	tmp.resize(2);
	tmp[0].resize(state_dimension);
	tmp[1].resize(state_dimension);
	sensor_1.resize(state_dimension);
	sensor_2.resize(state_dimension);

	if(davinci_interface::get_fresh_robot_pos(tmp)){
		sensor_1 = tmp[0];
		sensor_2 = tmp[1];
	}

	//Convert into proper format, not want to use the euler angle prefer rodrigues for vision
	Eigen::Affine3d a1_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_1.data()));
	Eigen::Vector3d a1_trans = a1_pos.translation();
	//Eigen::Vector3d a1_rpy = a1_pos.rotation().eulerAngles(0, 1, 2);

	cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(a1_pos, a1_rvec);

	Eigen::Affine3d a2_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_2.data()));
	Eigen::Vector3d a2_trans = a2_pos.translation();
	//Eigen::Vector3d a2_rpy = a2_pos.rotation().eulerAngles(0, 1, 2);
	cv::Mat a2_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(a2_pos, a2_rvec);

	cv::Mat zt = cv::Mat_<double>(L, 1);
	zt.at<double>(0 , 0) = a1_trans[0];
	zt.at<double>(1 , 0) = a1_trans[1];
	zt.at<double>(2 , 0) = a1_trans[2];
	zt.at<double>(3 , 0) = a2_rvec.at<double>(0,0);
	zt.at<double>(4 , 0) = a2_rvec.at<double>(1,0);
	zt.at<double>(5 , 0) = a2_rvec.at<double>(2,0);
	zt.at<double>(6 , 0) = a2_trans[0];
	zt.at<double>(7 , 0) = a2_trans[1];
	zt.at<double>(8 , 0) = a2_trans[2];
	zt.at<double>(9 , 0) = a1_rvec.at<double>(0,0);
	zt.at<double>(10, 0) = a1_rvec.at<double>(1,0);
	zt.at<double>(11, 0) = a1_rvec.at<double>(2,0);

	ROS_INFO("SENSOR 1 ARM AT (%f %f %f): %f %f %f", zt.at<double>(0, 0), zt.at<double>(1, 0),zt.at<double>(2, 0),zt.at<double>(3, 0),zt.at<double>(4, 0), zt.at<double>(5, 0));
	ROS_INFO("SENSOR 2 ARM AT (%f %f %f): %f %f %f", zt.at<double>(6, 0), zt.at<double>(7, 0),zt.at<double>(8, 0),zt.at<double>(9, 0),zt.at<double>(10, 0), zt.at<double>(11, 0));

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
		g(sigma_pts_bar[i], sigma_pts_last[i], sigma_pts_last[0] - zt);
		//ROS_ERROR("%f %f %f %f %f %f", sigma_pts_bar[i].at<double>(1, 1),sigma_pts_bar[i].at<double>(2, 1),sigma_pts_bar[i].at<double>(3, 1),sigma_pts_bar[i].at<double>(4, 1),sigma_pts_bar[i].at<double>(5, 1),sigma_pts_bar[i].at<double>(6, 1));
	}
	last_update = ros::Time::now().toSec();
	fvc_1 = false;
	fvc_2 = false;

	/*****Create the predicted mus and sigmas.*****/
	cv::Mat mu_bar = cv::Mat_<double>::zeros(L, 1);
	for(int i = 0; i < 2 * L + 1; i++){
		mu_bar = mu_bar + w_m[i] * sigma_pts_bar[i];
	}
	cv::Mat sigma_bar = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_bar = sigma_bar + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((sigma_pts_bar[i] - mu_bar).t());
	}
	/*****Render each sigma point and compute its matching score.*****/

	std::vector<double> mscores;
	mscores.resize(2*L + 1);

	computeSigmaMeasures(mscores, sigma_pts_bar, segmented_left, segmented_right, tmp[0], tmp[1]);

	//debuging
//	for(int i = 0; i < mscores.size(); i++){
//		ROS_INFO("MSCORES %f", mscores[i]);
//		ROS_WARN_STREAM("sigma_pts_bar " << sigma_pts_bar[i]);
//	}

	/*****Correction Step: Move the sigma points through the measurement function.*****/
	std::vector<cv::Mat_<double> > Z_bar;
	Z_bar.resize(2 * L + 1);
	for(int i = 0; i < 2 * L + 1; i++){
		h(Z_bar[i], sigma_pts_bar[i]);
	}

	/*****Calculate derived variance statistics.*****/
	cv::Mat z_caret = cv::Mat_<double>::zeros(L, 1);
	for(int i = 0; i < 2 * L + 1; i++){
		z_caret = z_caret + w_m[i] * mscores[i] * Z_bar[i];
	}

	//ROS_INFO_STREAM("correction? " << zt - z_caret);

	cv::Mat S = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		S = S + mscores[i] * w_c[i] * (Z_bar[i] - z_caret) * ((Z_bar[i] - z_caret).t());
	}
	cv::Mat Q = cv::Mat::eye(L,L,CV_64FC1);
	Q = Q * 0.38;
	S = S + Q;

	cv::Mat sigma_xz = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_xz = sigma_xz + mscores[i] * w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((Z_bar[i] - z_caret).t());
	}

	cv::Mat K = sigma_xz * S.inv();
	/*****Update our mu and sigma.*****/
	kalman_mu = mu_bar + K * (zt - z_caret);
	kalman_sigma = sigma_bar - K * S * K.t();
	ROS_WARN("1 ARM AT (%f %f %f): %f %f %f", kalman_mu.at<double>(0, 0), kalman_mu.at<double>(1, 0),kalman_mu.at<double>(2, 0),kalman_mu.at<double>(3, 0),kalman_mu.at<double>(4, 0), kalman_mu.at<double>(5, 0));
	ROS_WARN("2 ARM AT (%f %f %f): %f %f %f", kalman_mu.at<double>(6, 0), kalman_mu.at<double>(7, 0),kalman_mu.at<double>(8, 0),kalman_mu.at<double>(9, 0),kalman_mu.at<double>(10, 0), kalman_mu.at<double>(11, 0));


};

void KalmanFilter::g(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in, const cv::Mat & zt){
	/*cv::Mat delta_1 = cv::Mat_<double>::zeros(6, 1);
	cv::Mat delta_2 = cv::Mat_<double>::zeros(6, 1);
	if(fvc_1){
		cv::Mat sensor_1 = cv::Mat_<double>::zeros(6, 1);
		sensor_1.at<double>(0, 0) = sigma_point_in.at<double>(0, 0);
		sensor_1.at<double>(1, 0) = sigma_point_in.at<double>(1, 0);
		sensor_1.at<double>(2, 0) = sigma_point_in.at<double>(2, 0);
		sensor_1.at<double>(3, 0) = sigma_point_in.at<double>(3, 0);
		sensor_1.at<double>(4, 0) = sigma_point_in.at<double>(4, 0);
		sensor_1.at<double>(5, 0) = sigma_point_in.at<double>(5, 0);
		delta_1 = (sensor_1 - cmd_1);
		//ROS_INFO("GREEN DELTAS %f %f %f %f %f %f", delta_green.at<double>(0, 0), delta_green.at<double>(1, 0), delta_green.at<double>(2, 0), delta_green.at<double>(3, 0), delta_green.at<double>(4, 0), delta_green.at<double>(5, 0));
	}
	if(fvc_2){
		cv::Mat sensor_2 = cv::Mat_<double>::zeros(6, 1);
		sensor_2.at<double>(0, 0) = sigma_point_in.at<double>(6 , 0);
		sensor_2.at<double>(1, 0) = sigma_point_in.at<double>(7 , 0);
		sensor_2.at<double>(2, 0) = sigma_point_in.at<double>(8 , 0);
		sensor_2.at<double>(3, 0) = sigma_point_in.at<double>(9 , 0);
		sensor_2.at<double>(4, 0) = sigma_point_in.at<double>(10, 0);
		sensor_2.at<double>(5, 0) = sigma_point_in.at<double>(11, 0);
		delta_2 = (sensor_2 - cmd_2);
		//ROS_INFO("YELLOW DELTAS %f %f %f %f %f %f", delta_yellow.at<double>(0, 0), delta_yellow.at<double>(1, 0), delta_yellow.at<double>(2, 0), delta_yellow.at<double>(3, 0), delta_yellow.at<double>(4, 0), delta_yellow.at<double>(5, 0));
	}
	cv::Mat delta_all = cv::Mat_<double>::zeros(12, 1);
	vconcat(delta_1, delta_2, delta_all);

	//sigma_point_out = sigma_point_in.clone();
	sigma_point_out = sigma_point_in - (delta_all * (ros::Time::now().toSec() - last_update));
	//last_call = ros::Time::now().toSec();
	//sigma_point_out = zt.clone();*/
	sigma_point_out = sigma_point_in - zt;
};

/***this function should compute the matching score for all of the sigma points****/
void KalmanFilter::computeSigmaMeasures(
	std::vector<double> & measureWeights,
	const std::vector<cv::Mat_<double> > & sigma_point_in,
	const cv::Mat &segmented_left,
	const cv::Mat &segmented_right,
	const std::vector<double> & joints_1,
	const std::vector<double> & joints_2
){
	//ROS_ERROR("IN CSM FUNC: %lu, %lu", measureWeights.size(), sigma_point_in.size());
	//wrong coord????
	double total = 0.0;
	for (int i = 0; i < sigma_point_in.size() ; i++) {
		measureWeights[i] = matching_score(sigma_point_in[i], segmented_left, segmented_right, joints_1, joints_2);
		total += measureWeights[i];
	}
	if(total != 0.0){
		//normalization of measurement weights
		for (int j = 0; j < sigma_point_in.size(); j++) {
			measureWeights[j] = (measureWeights[j] / total) * (2*L + 1); //NORMALIZE here as the dimension of the size of sigma points, because is too small
		}
	}else{
		ROS_ERROR("Cannot find good measurement scores");
		for (int j = 0; j < sigma_point_in.size(); j++) {
			measureWeights[j] = 1.0;
		}
	}

};

double KalmanFilter::matching_score(
	const cv::Mat &stat,
	const cv::Mat &segmented_left,
	const cv::Mat &segmented_right,
	const std::vector<double> & joints_1,
	const std::vector<double> & joints_2
){

	//ROS_INFO_STREAM("stat IS: " << stat);
	//Convert our state into Eigen::Affine3ds; one for each arm, I change the order here
	cv::Mat arm1 = cv::Mat_<double>::zeros(6, 1);
	cv::Mat arm2 = cv::Mat_<double>::zeros(6, 1);
	arm1.at<double>(0,0) = stat.at<double>(0 , 0);
	arm1.at<double>(1,0) = stat.at<double>(1 , 0);
	arm1.at<double>(2,0) = stat.at<double>(2 , 0);
	arm1.at<double>(3,0) = stat.at<double>(3 , 0);
	arm1.at<double>(4,0) = stat.at<double>(4 , 0);
	arm1.at<double>(5,0) = stat.at<double>(5 , 0);

	arm2.at<double>(0,0) = stat.at<double>(6 , 0);
	arm2.at<double>(1,0) = stat.at<double>(7 , 0);
	arm2.at<double>(2,0) = stat.at<double>(8 , 0);
	arm2.at<double>(3,0) = stat.at<double>(9 , 0);
	arm2.at<double>(4,0) = stat.at<double>(10, 0);
	arm2.at<double>(5,0) = stat.at<double>(11, 0);

	//Convert them into tool models
	ToolModel::toolModel arm_1;
	ToolModel::toolModel arm_2;

	convertToolModel(arm1, arm_1, joints_1[4], joints_1[5], joints_1[6]);
	convertToolModel(arm2, arm_2, joints_2[4], joints_2[5], joints_2[6]);
	
	//ROS_ERROR("SHOWING SIGPOINT %f %f %f %f %f %f",stat.at<double>(0 , 0), stat.at<double>(1 , 0), stat.at<double>(2 , 0), stat.at<double>(3 , 0), stat.at<double>(4 , 0), stat.at<double>(5 , 0));

	//Render the tools and compute the matching score
	//TODO: Need both arms in the same image.
//
//	double matchingScore_arm_1 = measureFunc(
//		toolImage_left_arm_1,
//		toolImage_right_arm_1,
//		arm_1,
//		segmented_left,
//		segmented_right,
//		Cam_left_arm_1,
//		Cam_right_arm_1
//	);
//	double matchingScore_arm_2 = measureFunc(
//		toolImage_left_arm_2,
//		toolImage_right_arm_2,
//		arm_2,
//		segmented_left,
//		segmented_right,
//		Cam_left_arm_2,
//		Cam_right_arm_2
//	);
//	cv::imshow("Real Left Cam", segmented_left);
//	cv::imshow("Real Right Cam", segmented_right);
//
//	cv::imshow("Render arm 1 Left cam" ,toolImage_left_arm_1 );
//	cv::imshow("Render arm 1 Right cam" ,toolImage_right_arm_1 );
//	cv::imshow("Render arm 2 Left cam" ,toolImage_left_arm_2 );
//	cv::imshow("Render arm 2 Right cam" ,toolImage_right_arm_2 );
//	cv::waitKey(1);
//
//	double result = (matchingScore_arm_1 + matchingScore_arm_2) / 2;
	////testing
	double matchingScore_left = measureFuncSameCam(toolImage_cam_left, arm_1, arm_2, segmented_left, P_left, tool_rawImg_left, Cam_left_arm_1, Cam_left_arm_2);
	double matchingScore_right = measureFuncSameCam(toolImage_cam_right, arm_1, arm_2, segmented_right, P_right,tool_rawImg_right, Cam_right_arm_1, Cam_right_arm_2);

	cv::imshow("Real Left Cam", tool_rawImg_left);
	cv::imshow("Real Right Cam", tool_rawImg_right);

	cv::imshow("Render under LEFT cam" ,toolImage_cam_left );
	cv::imshow("Render under RIGHT cam" ,toolImage_cam_right );
	cv::waitKey(1);

	double result = sqrt(pow(matchingScore_left, 2) + pow(matchingScore_right, 2));

	return result;

};

void KalmanFilter::h(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in){
	//Assumes a non-distorted sensor. Largely substituted for by the matching-score weighting.
	//ROS_INFO_STREAM("DELT " << sigma_delt);
	//cv::Mat temp = sigma_point_in.clone();
	sigma_point_out = sigma_point_in.clone(); // + sigma_delt;
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

void KalmanFilter::convertToolModel(const cv::Mat & trans, ToolModel::toolModel &toolModel, double ja1, double ja2, double ja3){
	//Eigen::Vector3d pos = trans.translation();
	////Not use euler angles or Rodrigues angles
//	Eigen::Vector3d rpy = trans.rotation().eulerAngles(0, 1, 2);
//	ROS_INFO_STREAM("RPY " << rpy);
//	toolModel.tvec_elp(0) = pos[0];
//	toolModel.tvec_elp(1) = pos[1];
//	toolModel.tvec_elp(2) = pos[2];
//	toolModel.rvec_elp(0) = rpy[0];
//	toolModel.rvec_elp(1) = rpy[1];
//	toolModel.rvec_elp(2) = rpy[2];

	/*Eigen::Matrix3d rot_affine = trans.rotation();

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
	//ROS_INFO_STREAM("rot_vec " << rot_vec);*/

	toolModel.tvec_elp(0) = trans.at<double>(0,0);
	toolModel.tvec_elp(1) = trans.at<double>(1,0);
	toolModel.tvec_elp(2) = trans.at<double>(2,0);
	toolModel.rvec_elp(0) = trans.at<double>(3,0);
	toolModel.rvec_elp(1) = trans.at<double>(4,0);
	toolModel.rvec_elp(2) = trans.at<double>(5,0);

	ukfToolModel.computeDavinciModel(toolModel, ja1, ja2, ja3);
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
