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
using namespace cv_projective;

/***********ROBOT ARM NAMING PROTOCOL*********/
/**********************************************
The robot arms are to be referred to by color or number. Arm 1 is green and Arm 2 is yellow.
The arms are not under any circumstances to be referred to by 'left' or 'right'.
The cameras, conversely, are to be referred to *ONLY* by 'left' and 'right'- never 'one' or 'two'.
**********************************************/

KalmanFilter::KalmanFilter(ros::NodeHandle *nodehandle) :
		nh_(*nodehandle), L(9){

	ROS_INFO("Initializing UKF...");

	// initialization, just basic black image ??? how to get the size of the image
	toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);

	toolImage_left_arm_2 = cv::Mat::zeros(800, 1200, CV_8UC3);
	toolImage_right_arm_2 = cv::Mat::zeros(800, 1200, CV_8UC3);

	toolImage_cam_left = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_cam_right = cv::Mat::zeros(480, 640, CV_8UC3);

	tool_rawImg_left = cv::Mat::zeros(480, 640, CV_8UC3);
	tool_rawImg_right =cv::Mat::zeros(480, 640, CV_8UC3);

	seg_left  = cv::Mat::zeros(480, 640, CV_32FC1);
	seg_right  = cv::Mat::zeros(480, 640, CV_32FC1);

	freshSegImage = false;

	/***motion model params***/

	kinematics = Davinci_fwd_solver();

	//Pull in our first round of sensor data.
	davinci_interface::init_joint_feedback(nh_);
	std::vector<std::vector<double> > tmp;

	tmp.resize(2);
	if(davinci_interface::get_fresh_robot_pos(tmp)){
		sensor_1 = tmp[0];
		sensor_2 = tmp[1];
	}

//	Eigen::Affine3d a1_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_1.data()));
//	Eigen::Vector3d a1_trans = a1_pos.translation();
//	cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
//	computeRodriguesVec(a1_pos, a1_rvec);
//

	Eigen::Affine3d a2_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_2.data()));
	Eigen::Vector3d a2_trans = a2_pos.translation();
	cv::Mat a2_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(a2_pos, a2_rvec);

	Eigen::Affine3d arm_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
	Eigen::Affine3d arm_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
	Eigen::Affine3d arm_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );

	Eigen::Affine3d arm_pos = kinematics.affine_frame0_wrt_base_ * arm_pos_1 * arm_pos_2 * arm_pos_3;// * a1_4 *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
	Eigen::Vector3d arm_trans = arm_pos.translation();

 	cv::Mat arm_rvec = cv::Mat::zeros(3,1,CV_64FC1);

	computeRodriguesVec(arm_pos, arm_rvec);

	kalman_mu_arm1 = cv::Mat_<double>::zeros(L, 1);

	kalman_mu_arm1.at<double>(0 , 0) = arm_trans[0];
	kalman_mu_arm1.at<double>(1 , 0) = arm_trans[1];
	kalman_mu_arm1.at<double>(2 , 0) = arm_trans[2];
	kalman_mu_arm1.at<double>(3 , 0) = arm_rvec.at<double>(0,0);
	kalman_mu_arm1.at<double>(4 , 0) = arm_rvec.at<double>(1,0);
	kalman_mu_arm1.at<double>(5 , 0) = arm_rvec.at<double>(2,0);
	kalman_mu_arm1.at<double>(6 , 0) = tmp[0][4];
	kalman_mu_arm1.at<double>(7 , 0) = tmp[0][5];
	kalman_mu_arm1.at<double>(8 , 0) = tmp[0][6];

    zt_arm1 = cv::Mat_<double>::zeros(L, 1);
    zt_arm1 = kalman_mu_arm1.clone();    //initialization for maeasurement

	kalman_mu_arm2 = cv::Mat_<double>::zeros(L, 1);

	kalman_mu_arm2.at<double>(9 , 0) = a2_trans[0];
	kalman_mu_arm2.at<double>(10, 0) = a2_trans[1];
	kalman_mu_arm2.at<double>(11, 0) = a2_trans[2];
	kalman_mu_arm2.at<double>(12, 0) = a2_rvec.at<double>(0,0);
	kalman_mu_arm2.at<double>(13, 0) = a2_rvec.at<double>(1,0);
	kalman_mu_arm2.at<double>(14, 0) = a2_rvec.at<double>(2,0);
	kalman_mu_arm2.at<double>(15, 0) = tmp[1][4];
	kalman_mu_arm2.at<double>(16, 0) = tmp[1][5];
	kalman_mu_arm2.at<double>(17, 0) = tmp[1][6];

	double dev_pos = ukfToolModel.randomNum(0.6, 0.2);  ///deviation for position
	double dev_ori = ukfToolModel.randomNum(0.8, 0.5);  ///deviation for orientation
	double dev_ang = ukfToolModel.randomNum(0.2, 0); ///deviation for joint angles

	kalman_sigma_arm1 = (cv::Mat_<double>::eye(L, L));
	for (int j = 0; j < 3; ++j) {
		kalman_sigma_arm1.at<double>(j,j) = dev_pos; //gaussian generator
	}
	for (int j = 3; j < 6; ++j) {
		kalman_sigma_arm1.at<double>(j,j) = dev_ori; //gaussian generator
	}
	for (int j = 6; j < 9; ++j) {
		kalman_sigma_arm1.at<double>(j,j) = dev_ang; //gaussian generator
	}

	kalman_sigma_arm2 = (cv::Mat_<double>::eye(L, L));
	for (int j = 0; j < L; ++j) {
		kalman_sigma_arm2.at<double>(j,j) = ukfToolModel.randomNumber(0.38,0); //gaussian generator
	}

	freshCameraInfo = false; //should be left and right
	
	//The projection matrix from the simulation does not accurately reflect the Da Vinci robot. We are hardcoding the matrix from the da vinci itself.
//	projectionMat_subscriber_r = nh_.subscribe("/davinci_endo/right/camera_info", 1, &KalmanFilter::projectionRightCB, this);
//	projectionMat_subscriber_l = nh_.subscribe("/davinci_endo/left/camera_info", 1, &KalmanFilter::projectionLeftCB, this);

	P_left = cv::Mat::zeros(3,4,CV_64FC1);
	P_right = cv::Mat::zeros(3,4,CV_64FC1);

	cv::Mat P_l(3, 4, CV_64FC1);
	P_l.at<double>(0, 0) = 880.0575531441748;
	P_l.at<double>(1, 0) = 0;
	P_l.at<double>(2, 0) = 0;

	P_l.at<double>(0, 1) = 0;
	P_l.at<double>(1, 1) = 880.0575531441748;
	P_l.at<double>(2, 1) = 0;

	P_l.at<double>(0, 2) = 300.5; // horiz
	P_l.at<double>(1, 2) = 150.5; //verticle
	P_l.at<double>(2, 2) = 1;

	P_l.at<double>(0, 3) = 0;
	P_l.at<double>(1, 3) = 0;
	P_l.at<double>(2, 3) = 0;


	cv::Mat P_r(3, 4, CV_64FC1);

	P_r.at<double>(0, 0) = 880.0575531441748;
	P_r.at<double>(1, 0) = 0;
	P_r.at<double>(2, 0) = 0;

	P_r.at<double>(0, 1) = 0;
	P_r.at<double>(1, 1) = 880.0575531441748;
	P_r.at<double>(2, 1) = 0;

	P_r.at<double>(0, 2) = 300.5; // horiz
	P_r.at<double>(1, 2) = 150.5; //verticle
	P_r.at<double>(2, 2) = 1;

	P_r.at<double>(0, 3) = 5.1043338082362135;
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

double KalmanFilter::measureFunc(
	cv::Mat & toolImage_left,
	cv::Mat & toolImage_right,
	ToolModel::toolModel &toolPose,
	cv::Mat &Cam_left,
	cv::Mat &Cam_right,
	cv::Mat & rawImage_left,
	cv::Mat & rawImage_right){

	toolImage_left.setTo(0);
	toolImage_right.setTo(0);

	/***do the sampling and get the matching score***/
	//first get the rendered image using 3d model of the tool
	ukfToolModel.renderTool(toolImage_left, toolPose, Cam_left, P_left);
	double left = ukfToolModel.calculateMatchingScore(toolImage_left, seg_left);  //get the matching score

	ukfToolModel.renderTool(toolImage_right, toolPose, Cam_right, P_right);
	double right = ukfToolModel.calculateMatchingScore(toolImage_right, seg_right);

	ukfToolModel.renderTool(rawImage_left, toolPose, Cam_left, P_left);
	ukfToolModel.renderTool(rawImage_right, toolPose, Cam_right, P_right);

	double matchingScore = sqrt(pow(left, 2) + pow(right, 2));

	return matchingScore;
};

/***temp, is for not showing the rendered image of zt***/
double KalmanFilter::tempmeasureFunc(const cv::Mat &stat,
		cv::Mat & toolImage_left,
		cv::Mat & toolImage_right,
		cv::Mat &Cam_left,
		cv::Mat &Cam_right) {

		//ROS_INFO_STREAM("stat IS: " << stat);
		//Convert our state into Eigen::Affine3ds; one for each arm, I change the order here
		cv::Mat arm1 = cv::Mat_<double>::zeros(6, 1);

		arm1.at<double>(0,0) = stat.at<double>(0 , 0);
		arm1.at<double>(1,0) = stat.at<double>(1 , 0);
		arm1.at<double>(2,0) = stat.at<double>(2 , 0);
		arm1.at<double>(3,0) = stat.at<double>(3 , 0);
		arm1.at<double>(4,0) = stat.at<double>(4 , 0);
		arm1.at<double>(5,0) = stat.at<double>(5 , 0);

		//Convert them into tool models
		ToolModel::toolModel arm_1;

		/*TODO: different coordinate system and definition of orientations*/
		double joint_oval_1 = stat.at<double>(6 , 0);
		double joint_grip_dist_1 = stat.at<double>(7 , 0);
		double joint_grip_angle_1 = stat.at<double>(8 , 0);

		convertToolModel(arm1, arm_1, joint_oval_1, joint_grip_dist_1, joint_grip_angle_1);

		//Render the tools and compute the matching score
		//TODO: Need both arms in the same image.

		toolImage_left.setTo(0);
		toolImage_right.setTo(0);

		/***do the sampling and get the matching score***/
		//first get the rendered image using 3d model of the tool
		ukfToolModel.renderTool(toolImage_left, arm_1, Cam_left, P_left);
		double left = ukfToolModel.calculateChamferScore(toolImage_left, seg_left);  //get the matching score

		ukfToolModel.renderTool(toolImage_right, arm_1, Cam_right, P_right);
		double right = ukfToolModel.calculateChamferScore(toolImage_right, seg_right);

		double matchingScore_arm_1 = sqrt(pow(left, 2) + pow(right, 2));

		return matchingScore_arm_1;

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

/*
 * get a course estimation for dynamic tracking TODO:
 */
void KalmanFilter::getCourseEstimation(){

    std::vector<std::vector<double> > tmp;

    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[0];
        sensor_2 = tmp[1];
    }

    //	Eigen::Affine3d arm_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_data.data()));
//	Eigen::Vector3d arm_trans = arm_pos.translation();
//	cv::Mat arm_rvec = cv::Mat::zeros(3,1,CV_64FC1);
//	computeRodriguesVec(arm_pos, arm_rvec);

    Eigen::Affine3d arm_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    Eigen::Affine3d arm_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    Eigen::Affine3d arm_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );

    Eigen::Affine3d arm_pos = kinematics.affine_frame0_wrt_base_ * arm_pos_1 * arm_pos_2 * arm_pos_3;// * a1_4 *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
    Eigen::Vector3d arm_trans = arm_pos.translation();

    cv::Mat arm_rvec = cv::Mat::zeros(3,1,CV_64FC1);

    computeRodriguesVec(arm_pos, arm_rvec);

    kalman_mu_arm1 = cv::Mat_<double>::zeros(L, 1);

    kalman_mu_arm1.at<double>(0 , 0) = arm_trans[0];
    kalman_mu_arm1.at<double>(1 , 0) = arm_trans[1];
    kalman_mu_arm1.at<double>(2 , 0) = arm_trans[2];
    kalman_mu_arm1.at<double>(3 , 0) = arm_rvec.at<double>(0,0);
    kalman_mu_arm1.at<double>(4 , 0) = arm_rvec.at<double>(1,0);
    kalman_mu_arm1.at<double>(5 , 0) = arm_rvec.at<double>(2,0);
    kalman_mu_arm1.at<double>(6 , 0) = tmp[0][4];
    kalman_mu_arm1.at<double>(7 , 0) = tmp[0][5];
    kalman_mu_arm1.at<double>(8 , 0) = tmp[0][6];

    zt_arm1  = cv::Mat_<double>::zeros(L, 1);
    zt_arm1 = kalman_mu_arm1.clone();    //initialization for maeasurement

    double dev_pos = ukfToolModel.randomNum(0.6, 0.2);  ///deviation for position
    double dev_ori = ukfToolModel.randomNum(0.8, 0.6);  ///deviation for orientation
    double dev_ang = ukfToolModel.randomNum(0.2, 0); ///deviation for joint angles

    kalman_sigma_arm1 = (cv::Mat_<double>::eye(L, L));
    for (int j = 0; j < 3; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_pos; //gaussian generator
    }
    for (int j = 3; j < 6; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_ori; //gaussian generator
    }
    for (int j = 6; j < 9; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_ang; //gaussian generator
    }

};

void KalmanFilter::UKF_double_arm(){ //well, currently just one......

	seg_left = segmentation(tool_rawImg_left);
	seg_right = segmentation(tool_rawImg_right);

	ROS_INFO("--------------ARM 1 : --------------");
	ROS_INFO_STREAM("BEFORE kalman_mu_arm1: " << kalman_mu_arm1);
	update(kalman_mu_arm1, kalman_sigma_arm1, zt_arm1, toolImage_left_arm_1,
		   toolImage_right_arm_1, Cam_left_arm_1, Cam_right_arm_1);

	//ROS_INFO("--------------ARM 2 : --------------");
	//update(sensor_2, kalman_mu_arm2, kalman_sigma_arm2, toolImage_left_arm_2,
	//toolImage_right_arm_2, Cam_left_arm_2, Cam_right_arm_2);

//	cv::imshow("Render arm 1 Left cam" ,toolImage_left_arm_1 );
//	cv::imshow("Render arm 1 Right cam" ,toolImage_right_arm_1 );

//	cv::imshow("Render arm 2 Left cam" ,toolImage_left_arm_2 );
//	cv::imshow("Render arm 2 Right cam" ,toolImage_right_arm_2 );

	cv::imshow("Real Left Cam", tool_rawImg_left);
	cv::imshow("Real Right Cam", tool_rawImg_right);

	cv::waitKey(10);
};

void KalmanFilter::update(cv::Mat & kalman_mu, cv::Mat & kalman_sigma,cv::Mat &zt,
						  cv::Mat &left_image,cv::Mat &right_image,
						  cv::Mat &cam_left, cv::Mat &cam_right){

	/******Find and convert our various params and inputs******/
	cv::Mat sigma_t_last = kalman_sigma.clone();

	//****Generate the sigma points.****
	double lambda = alpha * alpha * (L + k) - L;
	double gamma = sqrt(L + lambda);

	///get the square root for sigma point generation using SVD decomposition
	cv::Mat root_sigma_t_last = cv::Mat_<double>::zeros(L, L);

	getSquareRootCov(sigma_t_last, root_sigma_t_last);
	ROS_INFO_STREAM(" root_sigma_t_last" << root_sigma_t_last);
	//Populate the sigma points:
	std::vector<cv::Mat_<double> > sigma_pts_last;
	sigma_pts_last.resize(2*L + 1);

	sigma_pts_last[0] = kalman_mu.clone();//X_0

	for (int i = 1; i <= L; i++) {
		cv::Mat square_root_sigma = root_sigma_t_last.col(i - 1);
		sigma_pts_last[i] = sigma_pts_last[0] + (gamma * square_root_sigma );
		sigma_pts_last[i + L] = sigma_pts_last[0] - (gamma * square_root_sigma);
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

	for(int i = 0; i < 2 * L + 1; i++){
		g(sigma_pts_bar[i], sigma_pts_last[i], sigma_pts_last[0] - zt); //no gaussian here,
		//ROS_INFO_STREAM("sigma_pts_last[i]: " << sigma_pts_last[i]);
	}

	/*****Create the predicted mus and sigmas.*****/
	cv::Mat mu_bar = cv::Mat_<double>::zeros(L, 1);
	for(int i = 0; i < 2 * L + 1; i++){
		mu_bar = mu_bar + w_m[i] * sigma_pts_bar[i]; //seems like every time is the corse guess

	}
	cv::Mat sigma_bar = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_bar = sigma_bar + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((sigma_pts_bar[i] - mu_bar).t());
	}

	/*****Render each sigma point and compute its matching score.*****/

	std::vector<double> mscores;
	mscores.resize(2*L + 1);

	computeSigmaMeasures(mscores, zt, sigma_pts_bar, left_image, right_image, cam_left, cam_right);

//	for(int i = 0; i < mscores.size(); i++){
//		ROS_INFO("MSCORES %f", mscores[i]);
////		ROS_WARN_STREAM("sigma_pts_bar " << sigma_pts_bar[i]);
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

	cv::Mat sigma_xz = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_xz = sigma_xz + mscores[i] * w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((Z_bar[i] - z_caret).t());
	}

	cv::Mat K = sigma_xz * S.inv();
	/*****Update our mu and sigma.*****/
	//ROS_INFO_STREAM("mu_bar" << mu_bar);
	kalman_mu = mu_bar + K * (zt - z_caret);
	kalman_sigma = sigma_bar - K * S * K.t();

	ROS_WARN("KALMAN ARM AT (%f %f %f): %f %f %f, joints: %f %f %f ",kalman_mu.at<double>(0, 0), kalman_mu.at<double>(1, 0),kalman_mu.at<double>(2, 0),kalman_mu.at<double>(3, 0),kalman_mu.at<double>(4, 0), kalman_mu.at<double>(5, 0), kalman_mu.at<double>(6, 0), kalman_mu.at<double>(7, 0),kalman_mu.at<double>(8, 0));

	cv::Mat arm1 = cv::Mat_<double>::zeros(6, 1);

	arm1.at<double>(0,0) = zt.at<double>(0 , 0);
	arm1.at<double>(1,0) = zt.at<double>(1 , 0);
	arm1.at<double>(2,0) = zt.at<double>(2 , 0);
	arm1.at<double>(3,0) = zt.at<double>(3 , 0);
	arm1.at<double>(4,0) = zt.at<double>(4 , 0);
	arm1.at<double>(5,0) = zt.at<double>(5 , 0);

	//Convert them into tool models
	ToolModel::toolModel arm_1;

	/*TODO: different coordinate system and definition of orientations*/
	double joint_oval_1 = zt.at<double>(6 , 0);
	double joint_grip_dist_1 = zt.at<double>(7 , 0);
	double joint_grip_angle_1 = zt.at<double>(8 , 0);

	convertToolModel(arm1, arm_1, joint_oval_1, joint_grip_dist_1, joint_grip_angle_1);
	cv::Mat seg_test = left_image.clone();
	ukfToolModel.renderTool(seg_test, arm_1, cam_left, P_left);
	cv::imshow("test mu image: " , seg_test );
	//cv::waitKey();

};

void KalmanFilter::g(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in, const cv::Mat & delta_zt){

	 sigma_point_out = sigma_point_in - delta_zt;

};

/***this function should compute the matching score for all of the sigma points****/
void KalmanFilter::computeSigmaMeasures(std::vector<double> & measureWeights, cv::Mat & zt, const std::vector<cv::Mat_<double> > & sigma_point_in,
										cv::Mat &left_image,cv::Mat &right_image,
										cv::Mat &cam_left, cv::Mat &cam_right){
	//ROS_ERROR("IN CSM FUNC: %lu, %lu", measureWeights.size(), sigma_point_in.size());
	double total = 0.0;
	double max_score = -1.0;
	for (int i = 0; i < sigma_point_in.size() ; i++) {
		measureWeights[i] = matching_score(sigma_point_in[i], left_image, right_image, cam_left, cam_right);
		total += measureWeights[i];
		if (measureWeights[i] >= max_score) {
			max_score = measureWeights[i];
			zt = sigma_point_in[i].clone();
			// ROS_INFO_STREAM("max_score: " << max_score);
		}
	}
	if(total != 0.0){
		//normalization of measurement weights
		for (int j = 0; j < sigma_point_in.size(); j++) {
			measureWeights[j] = (measureWeights[j] / total);// * (2*L + 1); //NORMALIZE here as the dimension of the size of sigma points, because is too small
		}
	}else{
		ROS_ERROR("Cannot find good measurement scores");
		for (int j = 0; j < sigma_point_in.size(); j++) {
			measureWeights[j] = 1.0;
		}
	}

};


/****using vision fucntions here careful with which image to use****/
double KalmanFilter::matching_score(const cv::Mat &stat, cv::Mat &left_image,cv::Mat &right_image,
									cv::Mat &cam_left, cv::Mat &cam_right){

	//ROS_INFO_STREAM("stat IS: " << stat);
	//Convert our state into Eigen::Affine3ds; one for each arm, I change the order here
	cv::Mat arm1 = cv::Mat_<double>::zeros(6, 1);

	arm1.at<double>(0,0) = stat.at<double>(0 , 0);
	arm1.at<double>(1,0) = stat.at<double>(1 , 0);
	arm1.at<double>(2,0) = stat.at<double>(2 , 0);
	arm1.at<double>(3,0) = stat.at<double>(3 , 0);
	arm1.at<double>(4,0) = stat.at<double>(4 , 0);
	arm1.at<double>(5,0) = stat.at<double>(5 , 0);

	//Convert them into tool models
	ToolModel::toolModel arm_1;

	/*TODO: different coordinate system and definition of orientations*/
	double joint_oval_1 = stat.at<double>(6 , 0);
	double joint_grip_dist_1 = stat.at<double>(7 , 0);
	double joint_grip_angle_1 = stat.at<double>(8 , 0);

	convertToolModel(arm1, arm_1, joint_oval_1, joint_grip_dist_1, joint_grip_angle_1);

	//Render the tools and compute the matching score
	//TODO: Need both arms in the same image.

	double matchingScore_arm_1 = measureFunc(left_image, right_image, arm_1,
		cam_left, cam_right, tool_rawImg_left, tool_rawImg_right);

	double result = matchingScore_arm_1;
	////testing
//	double matchingScore_left = measureFuncSameCam(toolImage_cam_left, arm_1, arm_2, segmented_left, P_left, tool_rawImg_left, Cam_left_arm_1, Cam_left_arm_2);
//	double matchingScore_right = measureFuncSameCam(toolImage_cam_right, arm_1, arm_2, segmented_right, P_right,tool_rawImg_right, Cam_right_arm_1, Cam_right_arm_2);

//	double result = sqrt(pow(matchingScore_left, 2) + pow(matchingScore_right, 2));

	return result;

};

void KalmanFilter::h(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in){

	sigma_point_out = sigma_point_in.clone(); //current strategy
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

//	toolModel.tvec_grip1(0) = trans.at<double>(0,0);
//	toolModel.tvec_grip1(1) = trans.at<double>(1,0);
//	toolModel.tvec_grip1(2) = trans.at<double>(2,0);
//	toolModel.rvec_grip1(0) = trans.at<double>(3,0);
//	toolModel.rvec_grip1(1) = trans.at<double>(4,0);
//	toolModel.rvec_grip1(2) = trans.at<double>(5,0);
//
//	ukfToolModel.computeDavinciModel(toolModel, ja1, ja2, ja3);

	toolModel.tvec_cyl(0) = trans.at<double>(0,0);
	toolModel.tvec_cyl(1) = trans.at<double>(1,0);
	toolModel.tvec_cyl(2) = trans.at<double>(2,0);
	toolModel.rvec_cyl(0) = trans.at<double>(3,0);
	toolModel.rvec_cyl(1) = trans.at<double>(4,0);
	toolModel.rvec_cyl(2) = trans.at<double>(5,0);

	ukfToolModel.computeEllipsePose(toolModel, ja1, ja2, ja3);

};

void KalmanFilter::computeRodriguesVec(const Eigen::Affine3d & trans, cv::Mat rot_vec){

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

void KalmanFilter::getSquareRootCov(cv::Mat &sigma_cov, cv::Mat &square_root){

	cv::Mat s = cv::Mat_<double>::zeros(L, 1);  //allocate space for SVD
	cv::Mat vt = cv::Mat_<double>::zeros(L, L);  //allocate space for SVD
	cv::Mat u = cv::Mat_<double>::zeros(L, L);  //allocate space for SVD

	cv::SVD::compute(sigma_cov, s, u, vt);//The actual square root gets saved into s

	for (int i = 0; i < L; ++i) {
		square_root.at<double>(i,i) = s.at<double>(i,0);
	}
};

cv::Mat KalmanFilter::segmentation(cv::Mat &InputImg) {

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

	freshSegImage = true;

	return res;

};

void KalmanFilter::Cholesky( const cv::Mat& A, cv::Mat& S )
{
	CV_Assert(A.type() == CV_64FC1);

	int dim = A.rows;
	S.create(dim, dim, CV_64FC1);

	int i, j, k;

	for( i = 0; i < dim; i++ )
	{
		for( j = 0; j < i; j++ )
			S.at<double>(i,j) = 0.00;

		double sum = 0.00;
		for( k = 0; k < i; k++ )
		{
			double val = S.at<double>(k,i);
			sum += val*val;
		}

		S.at<double>(i,i) = std::sqrt(std::max(A.at<double>(i,i) - sum, 0.00));
		double ival = 1.00/S.at<double>(i, i);

		for( j = i + 1; j < dim; j++ )
		{
			sum = 0;
			for( k = 0; k < i; k++ )
				sum += S.at<double>(k, i) * S.at<double>(k, j);

			S.at<double>(i, j) = (A.at<double>(i, j) - sum)*ival;
		}
	}

};