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
		nh_(*nodehandle), L(12){

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

	seg_left  = cv::Mat::zeros(480, 640, CV_32FC1);
	seg_right  = cv::Mat::zeros(480, 640, CV_32FC1);

	freshSegImage = false;

	/***motion model params***/
	//Initialization of sensor datas.
	kinematics = Davinci_fwd_solver();
	davinci_interface::init_joint_feedback(nh_);

	freshCameraInfo = false; //should be left and right

	//The projection matrix from the simulation does not accurately reflect the Da Vinci robot. We are hardcoding the matrix from the da vinci itself.
	projectionMat_subscriber_r = nh_.subscribe("/davinci_endo/right/camera_info", 1, &KalmanFilter::projectionRightCB, this);
	projectionMat_subscriber_l = nh_.subscribe("/davinci_endo/left/camera_info", 1, &KalmanFilter::projectionLeftCB, this);

	P_left = cv::Mat::zeros(3,4,CV_64FC1);
	P_right = cv::Mat::zeros(3,4,CV_64FC1);

	////Subscribe to the necessary transforms, this is for gazebo
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

	convertEigenToMat(arm_1__cam_l, Cam_left_arm_1);
	convertEigenToMat(arm_1__cam_r, Cam_right_arm_1);
	convertEigenToMat(arm_2__cam_l, Cam_left_arm_2);
	convertEigenToMat(arm_2__cam_r, Cam_right_arm_2);

	/***** the calibrated transformation for real da vinci robot comes in : ****/
//	Cam_left_arm_1 = (cv::Mat_<double>(4,4) << -0.9999999999863094, -3.726808388799082e-06, 3.673205103273929e-06, -0.2209000899903854,
//			-3.726781403852194e-06, 0.9999999999660707, 7.346410206619205e-06, -0.025046118487469873,
//			-3.673232481812485e-06, 7.346396517286155e-06, -0.999999999966269, 0.05029912523761887,
//			0, 0, 0, 1);
//
//	Cam_right_arm_1 = (cv::Mat_<double>(4,4) << -0.9999999999863094, -3.726808388799082e-06, 3.673205103273929e-06, -0.2109000899905203,
//			-3.726781403852194e-06, 0.9999999999660707, 7.346410206619205e-06, -0.025046081755553767,
//			-3.673232481812485e-06, 7.346396517286155e-06, -0.999999999966269, 0.05029916196993972,
//			0, 0, 0, 1);

	ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);
//	ROS_INFO_STREAM("Cam_left_arm_2: " << Cam_left_arm_2);
//	ROS_INFO_STREAM("Cam_right_arm_2: " << Cam_right_arm_2);

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

	//ROS_INFO_STREAM("left: " << P_left);
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
	double left = ukfToolModel.calculateChamferScore(toolImage_left, seg_left);  //get the matching score

	ukfToolModel.renderTool(toolImage_right, toolPose, Cam_right, P_right);
	double right = ukfToolModel.calculateChamferScore(toolImage_right, seg_right);

	ukfToolModel.renderTool(rawImage_left, toolPose, Cam_left, P_left);
	ukfToolModel.renderTool(rawImage_right, toolPose, Cam_right, P_right);

	double matchingScore = sqrt(pow(left, 2) + pow(right, 2));

	return matchingScore;
};

/*
 * get the measurement zt using coarse guess
 */
void KalmanFilter::getMeasurementModel(const cv::Mat &coarse_guess_vector, const cv::Mat &segmentation_img, const cv::Mat &projection_mat,
		cv::Mat &Cam_matrix, cv::Mat &rawImage, cv::Mat &zt, cv::Mat &normal_measurement)
{
	/*** blur the segmentation image: distance transformation ***/
	cv::Mat segImageGrey = segmentation_img.clone(); //crop segmented image, notice the size of the segmented image

	segImageGrey.convertTo(segImageGrey, CV_8UC1);
	/***segmented image process**/
	for (int i = 0; i < segImageGrey.rows; i++) {
		for (int j = 0; j < segImageGrey.cols; j++) {
			segImageGrey.at<uchar>(i,j) = 255 - segImageGrey.at<uchar>(i,j);
		}
	}

	cv::Mat segImgBlur;
	cv::Mat distance_img;
	cv::distanceTransform(segImageGrey, distance_img, CV_DIST_L2, 3);
	cv::normalize(distance_img, segImgBlur, 0.00, 1.00, cv::NORM_MINMAX);

	cv::imshow("segImgBlur", segImgBlur);

	/*** get the rendered image points and normals ***/
	cv::Mat temp_point = cv::Mat(1,2,CV_64FC1);
	cv::Mat temp_normal = cv::Mat(1,2,CV_64FC1);

	cv::Mat rendered_image = segmentation_img.clone();

	ToolModel::toolModel coarse_tool;
	convertToolModel(coarse_guess_vector, coarse_tool);
	ukfToolModel.renderToolUKF(rendered_image, coarse_tool, Cam_matrix, projection_mat, temp_point, temp_normal);

	ROS_INFO_STREAM("temp_normal row: " << temp_normal.rows );

	showNormals(temp_point, temp_normal, rendered_image );
	cv::imshow("rendered_image:", rendered_image);

	measurement_dim = temp_point.rows;

    cv::Mat measurement_points = cv::Mat_<double>::zeros(measurement_dim, 2);
	int radius = 40;

	//TODO: NEED TEST!
	cv::Mat test_measurement = segmentation_img.clone();
	ukfToolModel.renderToolUKF(test_measurement, coarse_tool, Cam_matrix, projection_mat, temp_point, temp_normal);

	cv::Mat temp_show = segImgBlur.clone();
	for (int i = 0; i < measurement_dim; ++i) {  //each vertex

		float max_intensity = 100.0;// -1.0;
		double x = temp_point.at<double>(i, 0);
		double y = temp_point.at<double>(i, 1);

		double x_normal = temp_normal.at<double>(i, 0);
		double y_normal = temp_normal.at<double>(i, 1);

		double theta = atan2(y_normal, x_normal);

		for (int j = -radius; j < radius + 10; ++j) {

			double delta_x =(x + j * cos(theta));
			double delta_y = (y + j * sin(theta));

//			/** testing **/
//			ROS_INFO_STREAM("double target_x " << delta_x);
//			ROS_INFO_STREAM("double target_y " << delta_y);
//
//			cv::Point2d prjpt_1;
//			prjpt_1.x = delta_x;
//			prjpt_1.y = delta_y;
//
//			cv::Point2d prjpt_2;
//
//			double new_delta_x = (x + (j+1) * cos(theta));
//			double new_delta_y = (y + (j+1) * sin(theta));
//
//			prjpt_2.x = new_delta_x;
//			prjpt_2.y = new_delta_y;
//			cv::line(temp_show, prjpt_1, prjpt_2, cv::Scalar(255, 255, 255), 1, 8, 0);
//			cv::imshow("temp image show the search range:", temp_show);

			float intensity = segImgBlur.at<float>(cv::Point2d(delta_x, delta_y));

			if (intensity < max_intensity) {
				max_intensity = intensity;
				measurement_points.at<double>(i, 0) = delta_x;
				measurement_points.at<double>(i, 1) = delta_y;
			}

		}
		//ROS_INFO_STREAM("temp_point: " << temp_point.at<double>(i, 0 ) << ", " << temp_point.at<double>(i, 1) );
		cv::Point2d prjpt_1;
		prjpt_1.x = temp_point.at<double>(i, 0);
		prjpt_1.y = temp_point.at<double>(i, 1);
		cv::Point2d prjpt_2;
		prjpt_2.x = measurement_points.at<double>(i, 0);
		prjpt_2.y = measurement_points.at<double>(i, 1);
		cv::line(test_measurement, prjpt_1, prjpt_2, cv::Scalar(255, 255, 255), 1, 8, 0);

	}
	cv::imshow("test_measurement", test_measurement);

	zt = cv::Mat_<double>::zeros(measurement_dim, 1);  //don't forget this
    for (int i = 0; i <measurement_dim; ++i) {
        cv::Mat normal = temp_normal.row(i);
        cv::Mat pixel = measurement_points.row(i);
        double dot_product = pixel.dot(normal);  //n^T * x
		zt.at<double>(i,0) = dot_product;
    }
    normal_measurement = temp_normal.clone();
};

/*
 * get the measurement model form both cameras
 */
void KalmanFilter::getStereoMeasurement(const cv::Mat & coarse_guess_vector, cv::Mat &zt, cv::Mat &normal_measurement){

	///get measurement model from LEFT camera
	cv::Mat zt_left;
	cv::Mat normal_left;
	getMeasurementModel(coarse_guess_vector, seg_left, P_left,
						Cam_left_arm_1, tool_rawImg_left, zt_left, normal_left);
	ROS_INFO_STREAM("zt_left" << zt_left);
	ROS_INFO_STREAM("normal_left" << normal_left);
	ROS_INFO("-------- RIGHT ------------------");
	///get measurement model from RIGHT camera
	cv::Mat zt_right;
	cv::Mat normal_right;
	getMeasurementModel(coarse_guess_vector, seg_right, P_right,
						Cam_right_arm_1, tool_rawImg_right, zt_right, normal_right);
	ROS_INFO_STREAM("zt_right" << zt_right);
	ROS_INFO_STREAM("normal_right" << normal_right);

	int left_dim = zt_left.rows;
	int right_dim = zt_right.rows;

	zt = cv::Mat_<double>::zeros(left_dim + right_dim, 1);
	normal_measurement = cv::Mat_<double>::zeros(left_dim + right_dim, 2);

	zt_left.copyTo( zt.rowRange(0, left_dim));
	zt_right.copyTo( zt.rowRange(left_dim, left_dim + right_dim));

	normal_left.copyTo( normal_measurement.rowRange(0, left_dim));
	normal_right.copyTo( normal_measurement.rowRange(left_dim, left_dim + right_dim));

};

/*
 * get the predicted measurements
 */
void KalmanFilter::h(cv::Mat & sigma_point_out, const cv::Mat_<double> & sigma_point_in,
					 cv::Mat &left_image,cv::Mat &right_image,
					 cv::Mat &cam_left, cv::Mat &cam_right, cv::Mat &normal_measurement){  ////normal_measurement indicate the direction of the mu or mean contour points

	//Convert sigma point (Mat) into tool models
	ToolModel::toolModel sigma_arm;
	convertToolModel(sigma_point_in, sigma_arm);

	left_image.setTo(0);
	right_image.setTo(0);

	cv::Mat temp_point_l = cv::Mat(1,2,CV_64FC1);
	cv::Mat temp_normal_l = cv::Mat(1,2,CV_64FC1);

	ukfToolModel.renderToolUKF(left_image, sigma_arm, cam_left, P_left, temp_point_l, temp_normal_l); //temp_normal not using right now

	//using vertex points and normals to get predicted measurement:
	cv::Mat left_pz = cv::Mat::zeros(temp_point_l.rows, 1, CV_64FC1);  ///left predicted measurements

	for (int i = 0; i <temp_point_l.rows ; ++i) {
		cv::Mat normal = normal_measurement.row(i);   //// or temp_normal_l
		cv::Mat pixel = temp_point_l.row(i);
		double dot_product = pixel.dot(normal);  //n^T * x
		left_pz.at<double>(i,0) = dot_product;
	}
	//ROS_INFO_STREAM("LEFT PREDICTED SIZE: " << temp_point_l.rows);
	ROS_INFO_STREAM("left_pz " << left_pz);

	sigma_point_out = left_pz.clone(); ////don't forget this

	/*** right camera predicted measurement ***/
	cv::Mat temp_point_r = cv::Mat(1,2,CV_64FC1);
	cv::Mat temp_normal_r = cv::Mat(1,2,CV_64FC1);
	ukfToolModel.renderToolUKF(right_image, sigma_arm, cam_right, P_right, temp_point_r, temp_normal_r); //temp_normal not using right now
//	showNormals(temp_point_r, temp_normal_r, right_image );
	cv::Mat right_pz = cv::Mat::zeros(temp_point_r.rows, 1, CV_64FC1);  ///left predicted measurements

	int left_dim = temp_point_l.rows;
	int right_dim = temp_normal_r.rows;

	int total_dim = left_dim + right_dim;
	ROS_INFO_STREAM("LEFT AND RIGHT SIZE : " << total_dim);
	ROS_INFO_STREAM("normal_measurement " << normal_measurement.rows);
	if(total_dim != normal_measurement.rows){
		ROS_ERROR("ONE SIGMA POINT HAS DIFFERENT NUMBER OD NORMALS !");
		exit(1);
	}else{
		//int left_dim = 0;
		for (int i = 0; i <temp_point_r.rows; ++i) {
			cv::Mat normal = normal_measurement.row(i + left_dim);   //// if using temp_normal_r, there can be singularities for covariance matrix
			cv::Mat pixel = temp_point_r.row(i);
			double dot_product = pixel.dot(normal);  //n^T * x
			right_pz.at<double>(i,0) = dot_product;
		}
		//ROS_INFO_STREAM("RIGHT PREDICTED SIZE: " << temp_point_r.rows);
		ROS_INFO_STREAM("right_pz " << right_pz);

		//sigma_point_out = right_pz.clone(); ////don't forget this

		sigma_point_out = cv::Mat_<double>::zeros(left_dim + right_dim, 1);

		left_pz.copyTo( sigma_point_out.rowRange(0, left_dim));

		right_pz.copyTo( sigma_point_out.rowRange(left_dim, left_dim + right_dim));
	}


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
//    Eigen::Affine3d arm_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
//    Eigen::Affine3d arm_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
//    Eigen::Affine3d arm_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
//    Eigen::Affine3d arm_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );
//
//    Eigen::Affine3d arm_pos = kinematics.affine_frame0_wrt_base_ * arm_pos_1 * arm_pos_2 * arm_pos_3 * arm_pos_4;// * kinematics.affine_gripper_wrt_frame6_ ;
//    Eigen::Vector3d arm_trans = arm_pos.translation();
//
//    cv::Mat arm_rvec = cv::Mat::zeros(3,1,CV_64FC1);
//
//    computeRodriguesVec(arm_pos, arm_rvec);
//
//    kalman_mu_arm1 = cv::Mat_<double>::zeros(L, 1);
//    kalman_mu_arm1.at<double>(0 , 0) = arm_trans[0];
//    kalman_mu_arm1.at<double>(1 , 0) = arm_trans[1];
//    kalman_mu_arm1.at<double>(2 , 0) = arm_trans[2];
//    kalman_mu_arm1.at<double>(3 , 0) = arm_rvec.at<double>(0,0);
//    kalman_mu_arm1.at<double>(4 , 0) = arm_rvec.at<double>(1,0);
//    kalman_mu_arm1.at<double>(5 , 0) = arm_rvec.at<double>(2,0);

	////intentionally bad ones......
	kalman_mu_arm1 = cv::Mat_<double>::zeros(L, 1);
	kalman_mu_arm1.at<double>(0 , 0) = -0.209029;
	kalman_mu_arm1.at<double>(1 , 0) = 0.011988;
	kalman_mu_arm1.at<double>(2 , 0) = -0.075185;
	kalman_mu_arm1.at<double>(3 , 0) = 0.80909;
	kalman_mu_arm1.at<double>(4 , 0) = -1.328505;
	kalman_mu_arm1.at<double>(5 , 0) = 1.464622;

//    kalman_mu_arm1.at<double>(6 , 0) = tmp[0][4];
//    kalman_mu_arm1.at<double>(7 , 0) = tmp[0][5];
//    kalman_mu_arm1.at<double>(8 , 0) = tmp[0][6];

    double dev_pos = ukfToolModel.randomNum(0.00001, 0.0);  ///deviation for position
    double dev_ori = ukfToolModel.randomNum(0.00001, 0.0);  ///deviation for orientation

    kalman_sigma_arm1 = (cv::Mat_<double>::eye(L, L));

    for (int j = 0; j < 3; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_pos; //gaussian generator
    }
    for (int j = 3; j < 6; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_ori; //gaussian generator
    }

	dev_pos = ukfToolModel.randomNum(0.0001, 0.0);  ///deviation for position
	dev_ori = ukfToolModel.randomNum(0.0001, 0.0);  ///deviation for orientation

	for (int j = 6; j < 9; ++j) {
		kalman_sigma_arm1.at<double>(j,j) = dev_pos; //gaussian generator
	}
	for (int j = 10; j < 12; ++j) {
		kalman_sigma_arm1.at<double>(j,j) = dev_ori; //gaussian generator
	}

//	double dev_ang = ukfToolModel.randomNum(0.0001, 0); ///deviation for joint angles
//	kalman_sigma_arm1.at<double>(6,6) = dev_ang; //gaussian generator
//
//	dev_ang = ukfToolModel.randomNum(0.0001, 0); ///deviation for joint angles
//	kalman_sigma_arm1.at<double>(7,7) = dev_ang; //gaussian generator
//	kalman_sigma_arm1.at<double>(8,8) = dev_ang; //gaussian generator

	//arm_2 waits here
};

void KalmanFilter::showNormals(cv::Mat &temp_point, cv::Mat &temp_normal, cv::Mat &inputImage ){
	int dim = temp_point.rows;
	for (int i = 0; i < dim; ++i) {

		cv::Point2d prjpt_1;
		prjpt_1.x = temp_point.at<double>(i,0);
		prjpt_1.y = temp_point.at<double>(i,1);

		double y_k = temp_normal.at<double>(i,1);
		double x_k = temp_normal.at<double>(i,0);
		double theta = atan2(y_k, x_k);
		double r = 40;

		cv::Point2d prjpt_2;
		prjpt_2.x = prjpt_1.x  + r * cos(theta);
		prjpt_2.y = prjpt_1.y  + r * sin(theta);
		cv::line(inputImage, prjpt_1, prjpt_2, cv::Scalar(255, 255, 255), 1, 8, 0);
		cv::imshow("rendered_image:", inputImage);
		cv::waitKey();
	}

};

void KalmanFilter::UKF_double_arm(){ //well, currently just one......

	seg_left = segmentation(tool_rawImg_left);
	seg_right = segmentation(tool_rawImg_right);

	ROS_INFO("--------------ARM 1 : --------------");
	getCourseEstimation();   ///get new kalman_mu_arm1, kalman_sigma_arm1
	ROS_INFO_STREAM("BEFORE kalman_mu_arm1: " << kalman_mu_arm1);

	update(kalman_mu_arm1, kalman_sigma_arm1, toolImage_left_arm_1,
		   toolImage_right_arm_1, Cam_left_arm_1, Cam_right_arm_1);

	//cv::imshow("Real Left Cam", tool_rawImg_left);
	//cv::imshow("Real Right Cam", tool_rawImg_right);

	cv::waitKey();
};

void KalmanFilter::update(cv::Mat & kalman_mu, cv::Mat & kalman_sigma,
						  cv::Mat &left_image,cv::Mat &right_image,
						  cv::Mat &cam_left, cv::Mat &cam_right){

	/******Find and convert our various params and inputs******/
	cv::Mat sigma_t_last = kalman_sigma.clone();
	//****Generate the sigma points.****
	double lambda = alpha * alpha * (L + k) - L;
	//ROS_INFO_STREAM("lambda " <<  lambda);
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

	/**** get measurement model ****/
	cv::Mat normal_measurement;
	cv::Mat zt;
	//getMeasurementModel(kalman_mu, seg_left, P_left,Cam_left_arm_1, tool_rawImg_left, zt, normal_measurement);   ///using left camera measurements
	//getMeasurementModel(kalman_mu, seg_right, P_right,Cam_right_arm_1, tool_rawImg_right, zt, normal_measurement);    ////using right camera measurements
	getStereoMeasurement(kalman_mu, zt, normal_measurement);

	ROS_INFO_STREAM(" zt: " << zt);

	//double normalizer = 1.0/measurement_dim;
	//ROS_INFO_STREAM("normal_measurement " << normal_measurement);
	cv::waitKey();
	/*****Update sigma points based on motion model******/
	std::vector<cv::Mat_<double> > sigma_pts_bar;
	sigma_pts_bar.resize(2*L + 1);

	cv::Mat render_test = seg_left.clone();
	cv::Mat temp_point_test = cv::Mat(1,2,CV_64FC1);
	cv::Mat temp_normal_test = cv::Mat(1,2,CV_64FC1);
	sigma_pts_bar[0] = sigma_pts_last[0];
	for(int i = 1; i < 2 * L + 1; i++){
		g(sigma_pts_bar[i], sigma_pts_last[i], sigma_pts_last[0]); // TODO: motion model
		///testing?
		ToolModel::toolModel test_arm;
		convertToolModel(sigma_pts_bar[i], test_arm);
		//render_test = seg_left.clone();
//		ROS_INFO_STREAM("sigma_pts_bar[i]: " << sigma_pts_bar[i]);
		ukfToolModel.renderToolUKF(render_test, test_arm, Cam_left_arm_1, P_left,temp_point_test, temp_normal_test );
		//cv::imshow(" test tun: " , render_test);
		//cv::waitKey();
	}
	cv::imshow(" test tun left : " , render_test);

	/***** Create the predicted mus and sigmas. *****/
	cv::Mat mu_bar = cv::Mat_<double>::zeros(L, 1);
	for(int i = 0; i < 2 * L + 1; i++){
//		ROS_INFO_STREAM("sigma_pts_bar[i]" << sigma_pts_bar[i]);
//		ROS_INFO_STREAM("w_m[i]" << w_m[i]);
		mu_bar = mu_bar + w_m[i] * sigma_pts_bar[i]; //seems like every time is the coarse guess
	}
	ROS_INFO_STREAM("mu_bar" << mu_bar);
	cv::waitKey();

	cv::Mat sigma_bar = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_bar = sigma_bar + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((sigma_pts_bar[i] - mu_bar).t());
	}

	ROS_INFO_STREAM("sigma_bar" << sigma_bar);
	/***** Correction Step: Move the sigma points through the measurement function *****/
	std::vector<cv::Mat_<double> > Z_bar;
	Z_bar.resize(2 * L + 1);

	for(int i = 0; i < 2 * L + 1; i++){
		h(Z_bar[i], sigma_pts_bar[i], left_image, right_image, cam_left, cam_right, normal_measurement);
//		ROS_INFO_STREAM(" Z_bar  " <<Z_bar[i] << "i " << i);
//		cv::waitKey();
	}

	/***** Calculate derived variance statistics *****/
	int measurement_dimension = Z_bar[0].rows;
	cv::Mat z_caret = cv::Mat_<double>::zeros(measurement_dimension, 1);
	for(int i = 0; i < 2 * L + 1; i++){
		z_caret = z_caret + w_m[i] * Z_bar[i];
	}
	ROS_INFO_STREAM("z_caret " << z_caret);
	cv::Mat S = cv::Mat_<double>::zeros(measurement_dimension, measurement_dimension);
	for(int i = 0; i < 2 * L + 1; i++){
		//ROS_INFO_STREAM(" Z_bar[i] - z_caret " <<Z_bar[i] - z_caret);
//		ROS_INFO_STREAM(" w_c[i]  " << w_c[i] );
		S = S + w_c[i] * (Z_bar[i] - z_caret) * ((Z_bar[i] - z_caret).t());
		//cv::waitKey();
	}

	ROS_INFO_STREAM(" S inv  " << S.inv());
	cv::waitKey();
	cv::Mat sigma_xz = cv::Mat_<double>::zeros(L, measurement_dimension);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_xz = sigma_xz + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((Z_bar[i] - z_caret).t());
	}

	cv::Mat K = sigma_xz * S.inv();
	ROS_INFO_STREAM(" K" << K);
    //cv::waitKey();
	/***** Update our mu and sigma *****/
	ROS_INFO_STREAM("mu_bar" << mu_bar);
	ROS_INFO_STREAM("zt - z_caret" << zt - z_caret);
	kalman_mu = mu_bar + K * (zt - z_caret);
	kalman_sigma = sigma_bar - K * S * K.t();

//	ROS_WARN("KALMAN ARM AT (%f %f %f): %f %f %f, joints: %f %f %f ",kalman_mu.at<double>(0, 0), kalman_mu.at<double>(1, 0),kalman_mu.at<double>(2, 0),kalman_mu.at<double>(3, 0),kalman_mu.at<double>(4, 0), kalman_mu.at<double>(5, 0), kalman_mu.at<double>(6, 0), kalman_mu.at<double>(7, 0),kalman_mu.at<double>(8, 0));
	ROS_WARN("KALMAN ARM AT (%f %f %f): %f %f %f, ",kalman_mu.at<double>(0, 0), kalman_mu.at<double>(1, 0),kalman_mu.at<double>(2, 0),kalman_mu.at<double>(3, 0),kalman_mu.at<double>(4, 0), kalman_mu.at<double>(5, 0));

	//Convert them into tool models
	ToolModel::toolModel show_arm;

	convertToolModel(kalman_mu, show_arm);
	cv::Mat test_l = tool_rawImg_left.clone();


    //cv::Mat seg_test_r = seg_right.clone();
	ukfToolModel.renderTool(test_l, show_arm, cam_left, P_left);
    //ukfToolModel.renderTool(seg_test_r, show_arm, cam_right, P_right);
	cv::imshow("kalman_mu left: " , test_l );

	cv::Mat test_r = tool_rawImg_right.clone();
	ukfToolModel.renderTool(test_r, show_arm, cam_right, P_right);
	cv::imshow("kalman_mu right: " , test_r );
    //cv::imshow("seg kalman_mu right: " , seg_test_r );
};

//TODO:
void KalmanFilter::g(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in, const cv::Mat & delta_zt){

	//sigma_point_out = sigma_point_in - delta_zt;
	sigma_point_out = sigma_point_in.clone();

	double dev_pos = ukfToolModel.randomNum(0.0001, 0.00);  ///deviation for position
	double dev_ori = ukfToolModel.randomNum(0.0001, 0.00);  ///deviation for orientation
	double dev_ang = ukfToolModel.randomNum(0.00001, 0); ///deviation for //cv::waitKey(); joint angles

	for (int j = 0; j < 3; ++j) {
		//double dev_pos = ukfToolModel.randomNum(0.06, 0.00);
		sigma_point_out.at<double>(j,0) = sigma_point_in.at<double>(j,0) + dev_pos;//gaussian generator
	}
	for (int j = 3; j < 6; ++j) {
		//double dev_ori = ukfToolModel.randomNum(0.08, 0.00);
		sigma_point_out.at<double>(j,0) = sigma_point_in.at<double>(j,0) + dev_ori;//gaussian generator
	}
//	for (int j = 6; j < 9; ++j) {
//		//double dev_ang = ukfToolModel.randomNum(0.001, 0);
//		sigma_point_out.at<double>(j,0) = delta_zt.at<double>(j,0) + dev_ang;//gaussian generator
//	}

};

/***this function should compute the matching score for all of the sigma points****/
void KalmanFilter::computeSigmaMeasures(std::vector<double> & measureWeights, const std::vector<cv::Mat_<double> > & sigma_point_in,
										cv::Mat &left_image,cv::Mat &right_image,
										cv::Mat &cam_left, cv::Mat &cam_right){
	double total = 0.0;

	for (int i = 0; i < sigma_point_in.size() ; i++) {
		measureWeights[i] = matching_score(sigma_point_in[i], left_image, right_image, cam_left, cam_right);
		total += measureWeights[i];

	}
	if(total != 0.0){
		//normalization of measurement weights
		for (int j = 0; j < sigma_point_in.size(); j++) {
			measureWeights[j] = (measureWeights[j] / total);//(2*L + 1); //NORMALIZE here as the dimension of the size of sigma points, because is too small
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
	//Convert them into tool models
	ToolModel::toolModel arm_1;

	convertToolModel(stat, arm_1);

	double matchingScore_arm_1 = measureFunc(left_image, right_image, arm_1,
		cam_left, cam_right, tool_rawImg_left, tool_rawImg_right);

	double result = matchingScore_arm_1;

	return result;

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

void KalmanFilter::convertToolModel(const cv::Mat & trans, ToolModel::toolModel &toolModel){

//	toolModel.tvec_grip1(0) = trans.at<double>(0,0);
//	toolModel.tvec_grip1(1) = trans.at<double>(1,0);
//	toolModel.tvec_grip1(2) = trans.at<double>(2,0);
//	toolModel.rvec_grip1(0) = trans.at<double>(3,0);
//	toolModel.rvec_grip1(1) = trans.at<double>(4,0);
//	toolModel.rvec_grip1(2) = trans.at<double>(5,0);
//
//	double ja1 = trans.at<double>(6,0);
//	double ja2 = trans.at<double>(7,0);
//	double ja3 = trans.at<double>(8,0);
//	ukfToolModel.computeDavinciModel(toolModel, ja1, ja2, ja3);

	toolModel.tvec_cyl(0) = trans.at<double>(0,0);
	toolModel.tvec_cyl(1) = trans.at<double>(1,0);
	toolModel.tvec_cyl(2) = trans.at<double>(2,0);
	toolModel.rvec_cyl(0) = trans.at<double>(3,0);
	toolModel.rvec_cyl(1) = trans.at<double>(4,0);
	toolModel.rvec_cyl(2) = trans.at<double>(5,0);

	double ja1 = trans.at<double>(6,0);
	double ja2 = trans.at<double>(7,0);
	double ja3 = trans.at<double>(8,0);

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

	/** get square root of the covariance **/
//	cv::Mat s = cv::Mat_<double>::zeros(L, 1);  //allocate space for SVD
//	cv::Mat vt = cv::Mat_<double>::zeros(L, L);  //allocate space for SVD
//	cv::Mat u = cv::Mat_<double>::zeros(L, L);  //allocate space for SVD
//
//	cv::SVD::compute(sigma_cov, s, u, vt);//The actual square root gets saved into s
//
//	for (int i = 0; i < L; ++i) {
//		square_root.at<double>(i,i) = s.at<double>(i,0);
//	}

	/** cholesky decomposition **/
	cv::Mat chol_mat(L,L,CV_64FC1);
	Cholesky( sigma_cov, chol_mat );

	square_root = chol_mat.clone();

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


void KalmanFilter::testRenderGazebo(){

    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[0];
        sensor_2 = tmp[1];
    }
    Eigen::Affine3d arm_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_1.data()));
    Eigen::Vector3d arm_trans = arm_pos.translation();
    cv::Mat arm_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(arm_pos, arm_rvec);

    cv::Mat test_render = tool_rawImg_left.clone();

    cv::Mat trans = cv::Mat::eye(4,4,CV_64FC1);

    cv::Mat tvec = cv::Mat_<double>::zeros(3, 1);
    tvec.at<double>(0 , 0) = arm_trans[0];
    tvec.at<double>(1 , 0) = arm_trans[1];
    tvec.at<double>(2 , 0) = arm_trans[2];

    Eigen::Matrix3d rot_affine = arm_pos.rotation();

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

    rot.copyTo(trans.colRange(0,3).rowRange(0,3));
    tvec.copyTo(trans.colRange(3,4).rowRange(0,3));

    ROS_INFO_STREAM("trans  " << trans);
    print_affine(arm_pos);

    cv::Mat origin = cv::Mat::zeros(4, 1, CV_64FC1);
    origin.at<double>(3,0) = 1;

    origin = transformPoints(origin, arm_rvec, tvec);
    origin = Cam_left_arm_1 * origin;
    //origin = Cam_left_arm_1 * trans * origin;

    ROS_INFO_STREAM("origin 2  " << origin);
    ROS_INFO_STREAM("P_left 2  " << P_left);
    cv::Point2d test_center;
    test_center = ukfToolModel.reproject(origin, P_left);

    cv::circle(test_render, test_center, 6, cv::Scalar(255,255,255),CV_FILLED, 8,0);
    cv::imshow("test_render", test_render);
    cv::waitKey();
};