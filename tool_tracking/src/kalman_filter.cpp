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

	toolImage_left_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);

	tool_rawImg_left = cv::Mat::zeros(480, 640, CV_8UC3);
	tool_rawImg_right =cv::Mat::zeros(480, 640, CV_8UC3);

	seg_left  = cv::Mat::zeros(480, 640, CV_32FC1);
	seg_right  = cv::Mat::zeros(480, 640, CV_32FC1);

	resulting_image = cv::Mat::zeros(480, 640, CV_8UC3);
	freshSegImage = false;
	freshCameraInfo = false; //should be left and right

	/***motion model params***/
	//Initialization of sensor data.
	kinematics = Davinci_fwd_solver();
	davinci_interface::init_joint_feedback(nh_);

	//The projection matrix from the simulation does not accurately reflect the Da Vinci robot. We are hardcoding the matrix from the da vinci itself.
	projectionMat_subscriber_r = nh_.subscribe("/davinci_endo/unsynched/right/camera_info", 1, &KalmanFilter::projectionRightCB, this);
	projectionMat_subscriber_l = nh_.subscribe("/davinci_endo/unsynched/left/camera_info", 1, &KalmanFilter::projectionLeftCB, this);

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
	arm_1__cam_l = xfu.transformTFToAffine3d(arm_1__cam_l_st);
	arm_1__cam_r = xfu.transformTFToAffine3d(arm_1__cam_r_st);
	arm_2__cam_l = xfu.transformTFToAffine3d(arm_2__cam_l_st);
	arm_2__cam_r = xfu.transformTFToAffine3d(arm_2__cam_r_st);

	convertEigenToMat(arm_1__cam_l, Cam_left_arm_1);
	convertEigenToMat(arm_1__cam_r, Cam_right_arm_1);
	convertEigenToMat(arm_2__cam_l, Cam_left_arm_2);
	convertEigenToMat(arm_2__cam_r, Cam_right_arm_2);

	ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);
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

//	ROS_INFO_STREAM("right: " << P_right);
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

//	ROS_INFO_STREAM("temp_normal row: " << temp_normal.rows );

	showNormals(temp_point, temp_normal, rendered_image );

	int measurement_dim = temp_point.rows;
    cv::Mat measurement_points = cv::Mat_<double>::zeros(measurement_dim, 2);
	int radius = 40;

	cv::Mat test_measurement = segmentation_img.clone();
	ukfToolModel.renderToolUKF(test_measurement, coarse_tool, Cam_matrix, projection_mat, temp_point, temp_normal);

//	cv::Mat temp_show = segImgBlur.clone();  ///test_show matrix is to show the search range and rendered
//	ukfToolModel.renderToolUKF(temp_show, coarse_tool, Cam_matrix, projection_mat, temp_point, temp_normal);
	for (int i = 0; i < measurement_dim; ++i) {  //each vertex

		float min_intensity = 100.0;// -1.0;
		double x = temp_point.at<double>(i, 0);
		double y = temp_point.at<double>(i, 1);

		double x_normal = temp_normal.at<double>(i, 0);
		double y_normal = temp_normal.at<double>(i, 1);

		double theta = atan2(y_normal, x_normal);

		for (int j = -radius; j < radius + 10; ++j) {

			double delta_x =(x + j * cos(theta));
			double delta_y = (y + j * sin(theta));

			 /** drawing the searching range **/
//			 ROS_INFO_STREAM("double target_x " << delta_x);
//			 ROS_INFO_STREAM("double target_y " << delta_y);

			 cv::Point2d prjpt_1;
			 prjpt_1.x = delta_x;
			 prjpt_1.y = delta_y;

			 cv::Point2d prjpt_2;

			 double new_delta_x = (x + (j+1) * cos(theta));
			 double new_delta_y = (y + (j+1) * sin(theta));

			 prjpt_2.x = new_delta_x;
			 prjpt_2.y = new_delta_y;
//			 cv::line(temp_show, prjpt_1, prjpt_2, cv::Scalar(255, 255, 255), 1, 8, 0);
//			 cv::imshow("temp image show the search range:", temp_show);
//			 cv::waitKey();

			float intensity = segImgBlur.at<float>(cv::Point2d(delta_x, delta_y));

			if (intensity < min_intensity) {
				min_intensity = intensity;
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
//    cv::imshow("test_measurement", test_measurement);
	zt = cv::Mat_<double>::zeros(measurement_dim, 1);
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
//	ROS_INFO_STREAM("zt_left" << zt_left);
//	ROS_INFO_STREAM("normal_left" << normal_left);
//	ROS_INFO("-------- RIGHT ------------------");
	///get measurement model from RIGHT camera
	cv::Mat zt_right;
	cv::Mat normal_right;
	getMeasurementModel(coarse_guess_vector, seg_right, P_right,
						Cam_right_arm_1, tool_rawImg_right, zt_right, normal_right);
//	ROS_INFO_STREAM("zt_right" << zt_right);
//	ROS_INFO_STREAM("normal_right" << normal_right);

	int left_dim = zt_left.rows;
	int right_dim = zt_right.rows;

	measurement_dimension = left_dim + right_dim;

	zt = cv::Mat_<double>::zeros(measurement_dimension, 1);
	normal_measurement = cv::Mat_<double>::zeros(measurement_dimension, 2);

	zt_left.copyTo( zt.rowRange(0, left_dim));
	zt_right.copyTo( zt.rowRange(left_dim, measurement_dimension));

	normal_left.copyTo( normal_measurement.rowRange(0, left_dim));
	normal_right.copyTo( normal_measurement.rowRange(left_dim, measurement_dimension));

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
    //resulting_image is to show the predicted measurement points
	//ukfToolModel.renderToolUKF(resulting_image, sigma_arm, cam_left, P_left, temp_point_l, temp_normal_l);
	for (int i = 0; i <temp_point_l.rows ; ++i) {
		cv::Mat normal = normal_measurement.row(i);   //// or temp_normal_l
		cv::Mat pixel = temp_point_l.row(i);
//		////showing the rendered predicted measurements
//		 cv::Point2d pixel_pts;
//		 pixel_pts.x = pixel.at<double>(0,0);
//		 pixel_pts.y = pixel.at<double>(0,1);
//		 cv::circle(resulting_image, pixel_pts, 2, cv::Scalar(0,0,255),CV_FILLED, 8,0);

		double dot_product = pixel.dot(normal);  //n^T * x
		left_pz.at<double>(i,0) = dot_product;
	}
//	 cv::imshow("resulting_image for sigma : " , resulting_image);
//	 cv::waitKey();

	 sigma_point_out = left_pz.clone(); //// for left camera tracking
	/*** right camera predicted measurement ***/
	cv::Mat temp_point_r = cv::Mat(1,2,CV_64FC1);
	cv::Mat temp_normal_r = cv::Mat(1,2,CV_64FC1);
	ukfToolModel.renderToolUKF(right_image, sigma_arm, cam_right, P_right, temp_point_r, temp_normal_r); //temp_normal not using right now
	cv::Mat right_pz = cv::Mat::zeros(temp_point_r.rows, 1, CV_64FC1);  ///left predicted measurements

	int left_dim = temp_point_l.rows;
	int right_dim = temp_normal_r.rows;

	int total_dim = left_dim + right_dim;
//	ROS_INFO_STREAM("LEFT AND RIGHT SIZE : " << total_dim);
//	ROS_INFO_STREAM("normal_measurement " << normal_measurement.rows);
	if(total_dim != normal_measurement.rows){
		ROS_ERROR("ONE SIGMA POINT HAS DIFFERENT NUMBER OD NORMALS !");
		exit(1);
	}else{
//		//int left_dim = 0;
//        resulting_image = cv::Mat::zeros(480,640,CV_8UC3);
//        ukfToolModel.renderToolUKF(resulting_image, sigma_arm, cam_right, P_right, temp_point_r, temp_normal_r); //testing in right camera
		for (int i = 0; i <temp_point_r.rows; ++i) {
			cv::Mat normal = normal_measurement.row(i + left_dim);   //// if using temp_normal_r, there can be singularities for covariance matrix
			cv::Mat pixel = temp_point_r.row(i);
            double dot_product = pixel.dot(normal);  //n^T * x
			right_pz.at<double>(i,0) = dot_product;
		}
		//ROS_INFO_STREAM("RIGHT PREDICTED SIZE: " << temp_point_r.rows);
		//ROS_INFO_STREAM("right_pz " << right_pz);

		sigma_point_out = cv::Mat_<double>::zeros(left_dim + right_dim, 1);

		left_pz.copyTo( sigma_point_out.rowRange(0, left_dim));
		right_pz.copyTo( sigma_point_out.rowRange(left_dim, left_dim + right_dim));
	}
};

/*
 * get a course estimation for dynamic tracking TODO:
 */
void KalmanFilter::getCoarseEstimation(){

    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[1];
        sensor_2 = tmp[0];
    }

	for (int i = 0; i < 7; ++i) {
		ROS_INFO_STREAM("SENSORS: " << sensor_1[i]);
	}
    Eigen::Affine3d arm_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    Eigen::Affine3d arm_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    Eigen::Affine3d arm_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
    Eigen::Affine3d arm_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );

    Eigen::Affine3d arm_pos = kinematics.affine_frame0_wrt_base_ * arm_pos_1 * arm_pos_2 * arm_pos_3 * arm_pos_4;// * kinematics.affine_gripper_wrt_frame6_ ;
    Eigen::Vector3d arm_trans = arm_pos.translation();

    cv::Mat arm_rvec = cv::Mat::zeros(3,1,CV_64FC1);

    computeRodriguesVec(arm_pos, arm_rvec);

	real_mu = cv::Mat_<double>::zeros(9, 1);
	real_mu.at<double>(0 , 0) = arm_trans[0];
	real_mu.at<double>(1 , 0) = arm_trans[1];
	real_mu.at<double>(2 , 0) = arm_trans[2];
	real_mu.at<double>(3 , 0) = arm_rvec.at<double>(0,0);
	real_mu.at<double>(4 , 0) = arm_rvec.at<double>(1,0);
	real_mu.at<double>(5 , 0) = arm_rvec.at<double>(2,0);

	real_mu.at<double>(6 , 0) = sensor_1[4];
	real_mu.at<double>(7 , 0) = sensor_1[5];
	real_mu.at<double>(8 , 0) = sensor_1[6];

	////intentionally bad ones......
	kalman_mu_arm1 = cv::Mat_<double>::zeros(L, 1);
	kalman_mu_arm1.at<double>(0 , 0) = arm_trans[0];
	kalman_mu_arm1.at<double>(1 , 0) = arm_trans[1] /*- 0.004*/;
	kalman_mu_arm1.at<double>(2 , 0) = arm_trans[2] /*+ 0.002*/;
	kalman_mu_arm1.at<double>(3 , 0) = arm_rvec.at<double>(0,0);
	kalman_mu_arm1.at<double>(4 , 0) = arm_rvec.at<double>(1,0) /*+ 0.03*/;
	kalman_mu_arm1.at<double>(5 , 0) = arm_rvec.at<double>(2,0) /*+ 0.02*/;

	kalman_mu_arm1.at<double>(6 , 0) = sensor_1[4];
    kalman_mu_arm1.at<double>(7 , 0) = sensor_1[5];
    kalman_mu_arm1.at<double>(8 , 0) = sensor_1[6];

    double dev_pos = ukfToolModel.randomNum(0.00001, 0.0);  ///deviation for position
    double dev_ori = ukfToolModel.randomNum(0.00001, 0.0);  ///deviation for orientation

    kalman_sigma_arm1 = (cv::Mat_<double>::eye(L, L));

    for (int j = 0; j < 3; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_pos; //gaussian generator
    }
    for (int j = 3; j < 6; ++j) {
        kalman_sigma_arm1.at<double>(j,j) = dev_ori; //gaussian generator
    }

	dev_pos = ukfToolModel.randomNum(0.00001, 0.0);  ///deviation for position
	dev_ori = ukfToolModel.randomNum(0.0001, 0.0);  ///deviation for orientation

	for (int j = 6; j < 9; ++j) {
		kalman_sigma_arm1.at<double>(j,j) = dev_pos; //gaussian generator
	}
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
//		cv::waitKey();
	}
};

void KalmanFilter::UKF_double_arm(){ //well, currently just one......

	seg_left = segmentation(tool_rawImg_left);
	seg_right = segmentation(tool_rawImg_right);

	ROS_INFO("--------------ARM 1 : --------------");
	getCoarseEstimation();   ///get new kalman_mu_arm1, kalman_sigma_arm1
	ROS_INFO_STREAM("BEFORE kalman_mu_arm1: " << kalman_mu_arm1);

    showRenderedImage(kalman_mu_arm1); //has cv::waitKey() inside

	ROS_INFO_STREAM("after render!");
	update(kalman_mu_arm1, kalman_sigma_arm1, toolImage_left_arm_1,
		   toolImage_right_arm_1, Cam_left_arm_1, Cam_right_arm_1);
	ROS_WARN_STREAM("FORAWRD KINEMATICS: " << real_mu);
	showGazeboToolError(real_mu, kalman_mu_arm1);
	showRenderedImage(kalman_mu_arm1); //has cv::waitKey() inside
};

void KalmanFilter::update(cv::Mat & kalman_mu, cv::Mat & kalman_sigma,
						  cv::Mat &left_image,cv::Mat &right_image,
						  cv::Mat &cam_left, cv::Mat &cam_right){

	/******Find and convert our various params and inputs******/
	cv::Mat sigma_t_last = kalman_sigma.clone();
	//****Generate the sigma points.****
	double lambda = alpha * alpha * (L + k) - L;
	double gamma = sqrt(L + lambda);

	///get the square root for sigma point generation using SVD decomposition
	cv::Mat root_sigma_t_last = cv::Mat_<double>::zeros(L, L);
	Cholesky(sigma_t_last, root_sigma_t_last);
//	ROS_INFO_STREAM(" root_sigma_t_last" << root_sigma_t_last);
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

	//getMeasurementModel(kalman_mu, seg_left, P_left,Cam_left_arm_1, tool_rawImg_left, zt, normal_measurement);  ///using only left camera measurements
	getStereoMeasurement(kalman_mu, zt, normal_measurement); ///using both camera measurements
//
//	ROS_INFO_STREAM(" zt: " << zt);
//	cv::waitKey();
	/*****Update sigma points based on motion model******/
	std::vector<cv::Mat_<double> > sigma_pts_bar;
	sigma_pts_bar.resize(2*L + 1);

	cv::Mat render_test_l = seg_left.clone();
	cv::Mat render_test_r = seg_right.clone();
	cv::Mat temp_point_test = cv::Mat(1,2,CV_64FC1);
	cv::Mat temp_normal_test = cv::Mat(1,2,CV_64FC1);
	sigma_pts_bar[0] = sigma_pts_last[0];
	for(int i = 1; i < 2 * L + 1; i++){
		g(sigma_pts_bar[i], sigma_pts_last[i]); // TODO: motion model
		///testing
		ToolModel::toolModel test_arm;
		convertToolModel(sigma_pts_bar[i], test_arm);
		//render_test = seg_left.clone();
//		ROS_INFO_STREAM("sigma_pts_bar[i]: " << sigma_pts_bar[i]);
		ukfToolModel.renderToolUKF(render_test_l, test_arm, Cam_left_arm_1, P_left,temp_point_test, temp_normal_test );
		ukfToolModel.renderToolUKF(render_test_r, test_arm, Cam_right_arm_1, P_right,temp_point_test, temp_normal_test );
	}
	cv::imshow(" sigma left : " , render_test_l);
	cv::imshow(" sigma right : " , render_test_l);
	/***** Create the predicted mus and sigmas. *****/
	cv::Mat mu_bar = cv::Mat_<double>::zeros(L, 1);
	for(int i = 0; i < 2 * L + 1; i++){
		mu_bar = mu_bar + w_m[i] * sigma_pts_bar[i]; //seems like every time is the coarse guess
	}
//	ROS_INFO_STREAM("mu_bar" << mu_bar);

	cv::Mat sigma_bar = cv::Mat_<double>::zeros(L, L);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_bar = sigma_bar + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((sigma_pts_bar[i] - mu_bar).t());
	}

//	ROS_INFO_STREAM("sigma_bar" << sigma_bar);
//	cv::waitKey();
	/***** Correction Step: Move the sigma points through the measurement function *****/
	std::vector<cv::Mat_<double> > Z_bar;
	Z_bar.resize(2 * L + 1);

	for(int i = 0; i < 2 * L + 1; i++){
		h(Z_bar[i], sigma_pts_bar[i], left_image, right_image, cam_left, cam_right, normal_measurement);
	}

	/***** Calculate predicted observation vector *****/
	cv::Mat z_caret = cv::Mat_<double>::zeros(measurement_dimension, 1);
	for(int i = 0; i < 2 * L + 1; i++){
		z_caret = z_caret + w_m[i] * Z_bar[i];
	}
//	ROS_INFO_STREAM("z_caret " << z_caret);
	cv::Mat S = cv::Mat_<double>::zeros(measurement_dimension, measurement_dimension);  ///covariance for predicted observation
	for(int i = 0; i < 2 * L + 1; i++){
		S = S + w_c[i] * (Z_bar[i] - z_caret) * ((Z_bar[i] - z_caret).t());
	}

	cv::Mat sigma_xz = cv::Mat_<double>::zeros(L, measurement_dimension);
	for(int i = 0; i < 2 * L + 1; i++){
		sigma_xz = sigma_xz + w_c[i] * (sigma_pts_bar[i] - mu_bar) * ((Z_bar[i] - z_caret).t());
	}

	cv::Mat K = sigma_xz * S.inv();

	/***** Update our mu and sigma *****/
//	ROS_INFO_STREAM("mu_bar" << mu_bar);
//	ROS_INFO_STREAM("zt - z_caret" << zt - z_caret);
	kalman_mu = mu_bar + K * (zt - z_caret);
	kalman_sigma = sigma_bar - K * S * K.t();
//	ROS_WARN_STREAM("KALMAN ARM AT : " << kalman_mu);
};

void KalmanFilter::g(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in){

	sigma_point_out = sigma_point_in.clone(); //initialization

	double dev_pos = ukfToolModel.randomNum(0.00001, 0.00);  ///deviation for position
	double dev_ori = ukfToolModel.randomNum(0.00001, 0.00);  ///deviation for orientation
	double dev_ang = ukfToolModel.randomNum(0.000001, 0); ///deviation for joint angles

	for (int j = 0; j < 3; ++j) {
		sigma_point_out.at<double>(j,0) = sigma_point_in.at<double>(j,0) + dev_pos;//gaussian generator
	}
	for (int j = 3; j < 6; ++j) {
		sigma_point_out.at<double>(j,0) = sigma_point_in.at<double>(j,0) + dev_ori;//gaussian generator
	}
	for (int j = 6; j < 9; ++j) {
		sigma_point_out.at<double>(j,0) = sigma_point_in.at<double>(j,0) + dev_ang;//gaussian generator
	}

};

void KalmanFilter::convertEigenToMat(const Eigen::Affine3d & arm_pose, cv::Mat & outputMatrix){

	outputMatrix = cv::Mat::eye(4,4,CV_64FC1);

	Eigen::Vector3d pos = arm_pose.translation();
	Eigen::Matrix3d rot = arm_pose.linear();

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

void KalmanFilter::convertToolModel(const cv::Mat & arm_pose, ToolModel::toolModel &toolModel){

	toolModel.tvec_cyl(0) = arm_pose.at<double>(0,0);
	toolModel.tvec_cyl(1) = arm_pose.at<double>(1,0);
	toolModel.tvec_cyl(2) = arm_pose.at<double>(2,0);
	toolModel.rvec_cyl(0) = arm_pose.at<double>(3,0);
	toolModel.rvec_cyl(1) = arm_pose.at<double>(4,0);
	toolModel.rvec_cyl(2) = arm_pose.at<double>(5,0);

	double ja1 = arm_pose.at<double>(6,0);
	double ja2 = arm_pose.at<double>(7,0);
	double ja3 = arm_pose.at<double>(8,0);

	ukfToolModel.computeEllipsePose(toolModel, ja1, ja2, ja3);

};

void KalmanFilter::computeRodriguesVec(const Eigen::Affine3d & arm_pose, cv::Mat &rot_vec){

	Eigen::Matrix3d rot_affine = arm_pose.rotation();

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

		S.at<double>(i,i) = sqrt(std::max(A.at<double>(i,i) - sum, 0.00));
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

void KalmanFilter::showGazeboToolError(cv::Mat &real_pose, cv::Mat &KalmanMu){

	//Get matrix representations of the toolModels
	int dim = 24;

	ToolModel::toolModel real_tool;
	ToolModel::toolModel kalman_tool;
	convertToolModel(real_pose, real_tool);
	convertToolModel(KalmanMu, kalman_tool);

	cv::Mat renderedMat = cv::Mat::zeros(dim,1,CV_64FC1);
	cv::Mat real_tool_vector = cv::Mat::zeros(dim,1,CV_64FC1);
	convertToolModeltoMatrix(kalman_tool, renderedMat);
	convertToolModeltoMatrix(real_tool, real_tool_vector);

	//Find difference in pos/orientation for real_pose and particle_pose
	cv::Mat position = real_tool_vector.rowRange(0, 12) - renderedMat.rowRange(0,12);
	cv::Mat orientation = real_tool_vector.rowRange(12, dim) - renderedMat.rowRange(12,dim);
	double error_pos = position.dot(position);
	double error_ori = orientation.dot(orientation);
	error_pos = sqrt(error_pos / 12);  //divide by dimension
	error_ori = sqrt(error_ori / 12);
	ROS_WARN_STREAM("Position  error: " << error_pos);
	ROS_WARN_STREAM("orientation  error: " << error_ori);

};

void KalmanFilter::showRenderedImage(cv::Mat &inputToolPose){
	//Convert them into tool models
	ToolModel::toolModel show_arm;

	convertToolModel(inputToolPose, show_arm);
	cv::Mat test_l = tool_rawImg_left.clone();

	ukfToolModel.renderTool(test_l, show_arm, Cam_left_arm_1, P_left);

	cv::Mat test_r = tool_rawImg_right.clone();
	ukfToolModel.renderTool(test_r, show_arm, Cam_right_arm_1, P_right);
	cv::imshow("kalman_mu left: " , test_l );
	cv::imshow("kalman_mu right: " , test_r );

	cv::waitKey();

};

void KalmanFilter::convertToolModeltoMatrix(const ToolModel::toolModel &inputToolModel, cv::Mat &toolMatrix){
	toolMatrix.at<double>(0,0) = inputToolModel.tvec_cyl(0);
	toolMatrix.at<double>(1,0) = inputToolModel.tvec_cyl(1);
	toolMatrix.at<double>(2,0) = inputToolModel.tvec_cyl(2);

	toolMatrix.at<double>(3,0) = inputToolModel.tvec_elp(0);
	toolMatrix.at<double>(4,0) = inputToolModel.tvec_elp(1);
	toolMatrix.at<double>(5,0) = inputToolModel.tvec_elp(2);

	toolMatrix.at<double>(6,0) = inputToolModel.tvec_grip1(0);
	toolMatrix.at<double>(7,0) = inputToolModel.tvec_grip1(1);
	toolMatrix.at<double>(8,0) = inputToolModel.tvec_grip1(2);

	toolMatrix.at<double>(9,0) = inputToolModel.tvec_grip2(0);
	toolMatrix.at<double>(10,0) = inputToolModel.tvec_grip2(1);
	toolMatrix.at<double>(11,0) = inputToolModel.tvec_grip2(2);

	toolMatrix.at<double>(12,0) = inputToolModel.rvec_cyl(0);
	toolMatrix.at<double>(13,0) = inputToolModel.rvec_cyl(1);
	toolMatrix.at<double>(14,0) = inputToolModel.rvec_cyl(2);

	toolMatrix.at<double>(15,0) = inputToolModel.rvec_elp(0);
	toolMatrix.at<double>(16,0) = inputToolModel.rvec_elp(1);
	toolMatrix.at<double>(17,0) = inputToolModel.rvec_elp(2);

	toolMatrix.at<double>(18,0) = inputToolModel.rvec_grip1(0);
	toolMatrix.at<double>(19,0) = inputToolModel.rvec_grip1(1);
	toolMatrix.at<double>(20,0) = inputToolModel.rvec_grip1(2);

	toolMatrix.at<double>(21,0) = inputToolModel.rvec_grip2(0);
	toolMatrix.at<double>(22,0) = inputToolModel.rvec_grip2(1);
	toolMatrix.at<double>(23,0) = inputToolModel.rvec_grip2(2);
};