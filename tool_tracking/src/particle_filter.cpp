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

#include <tool_tracking/particle_filter.h>  //everything inside here
using namespace std;

ParticleFilter::ParticleFilter(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle), numParticles(180), down_sample_rate(0.0015), error(1)
{

	/**** need to subscribe this for simulation ***/
    // tf::StampedTransform arm_1__cam_l_st;
    // tf::StampedTransform arm_2__cam_l_st;
    // tf::StampedTransform arm_1__cam_r_st;
    // tf::StampedTransform arm_2__cam_r_st;

    // try{
    //     tf::TransformListener l;
    //     while(!l.waitForTransform("/left_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
    //     l.lookupTransform("/left_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), arm_1__cam_l_st);
    //     while(!l.waitForTransform("/left_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
    //     l.lookupTransform("/left_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), arm_2__cam_l_st);
    //     while(!l.waitForTransform("/right_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
    //     l.lookupTransform("/right_camera_optical_frame", "one_psm_base_link", ros::Time(0.0), arm_1__cam_r_st);
    //     while(!l.waitForTransform("/right_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), ros::Duration(10.0)) && ros::ok()){}
    //     l.lookupTransform("/right_camera_optical_frame", "two_psm_base_link", ros::Time(0.0), arm_2__cam_r_st);
    // }
    // catch (tf::TransformException ex){
    //     ROS_ERROR("%s",ex.what());
    //     exit(1);
    // }

    // //Convert to Affine3ds for storage, which is the format they will be used in for rendering.
    // XformUtils xfu;
    // arm_1__cam_l = xfu.transformTFToAffine3d(arm_1__cam_l_st);//.inverse();
    // arm_1__cam_r = xfu.transformTFToAffine3d(arm_1__cam_r_st);//.inverse();
    // arm_2__cam_l = xfu.transformTFToAffine3d(arm_2__cam_l_st);//.inverse();
    // arm_2__cam_r = xfu.transformTFToAffine3d(arm_2__cam_r_st);//.inverse();

    // convertEigenToMat(arm_1__cam_l, Cam_left_arm_1);
    // convertEigenToMat(arm_1__cam_r, Cam_right_arm_1);
    // convertEigenToMat(arm_2__cam_l, Cam_left_arm_2);
    // convertEigenToMat(arm_2__cam_r, Cam_right_arm_2);

//    ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
//    ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);
//    ROS_INFO_STREAM("Cam_left_arm_2: " << Cam_left_arm_2);
//    ROS_INFO_STREAM("Cam_right_arm_2: " << Cam_right_arm_2);

    /********** using calibration results: camera-base transformation *******/
    Cam_left_arm_1 = (cv::Mat_<double>(4,4) << -0.7882, 0.6067, 0.1025, -0.12249,  //-0.7882, 0.6067, 0.1025, -0.1449,
            0.5854, 0.7909, -0.1784, -0.0480,   //	0.5854, 0.7909, -0.1784, -0.0607,
            -0.1894, -0.0806, -0.9786, 0.02, //	-0.1894, -0.0806, -0.9786, 0.0200,   0.0157
            0,0, 0, 1.0000);

    cv::Mat rot(3,3,CV_64FC1);
    cv::Mat rot_vec = (cv::Mat_<double>(3,1) << 1.09976677, 2.5802519, -0.200696); //1.0996677, 2.6502519, -0.20696
    cv::Rodrigues(rot_vec, rot);
    rot.copyTo(Cam_left_arm_1.colRange(0,3).rowRange(0,3));

    Cam_right_arm_1 = (cv::Mat_<double>(4,4) << -0.7893, 0.6067, 0.0949, -0.13599, // -0.7893, 0.6067, 0.0949, -0.1428,
            0.5852, 0.7899, -0.1835, -0.0500,   ///	0.5852, 0.7899, -0.1835, -0.0612,
            -0.1861, -0.0892, -0.9784, 0.0180,     //	-0.1861, -0.0892, -0.9784, 0.0223,
            0,0, 0, 1.0000);
    rot_vec = (cv::Mat_<double>(3,1) << 1.059996677, 2.5802519, -0.18696);
    cv::Rodrigues(rot_vec, rot);
    rot.copyTo(Cam_right_arm_1.colRange(0,3).rowRange(0,3));

	ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);

	initializeParticles();

	projectionMat_subscriber_r = node_handle.subscribe("/davinci_endo/right/camera_info", 1, &ParticleFilter::projectionRightCB, this);
	projectionMat_subscriber_l = node_handle.subscribe("/davinci_endo/left/camera_info", 1, &ParticleFilter::projectionLeftCB, this);

	toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);

	toolImage_left_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3);

	toolImage_left_temp = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_temp = cv::Mat::zeros(480, 640, CV_8UC3);

	raw_image_left = cv::Mat::zeros(480, 640, CV_8UC3);
	raw_image_right = cv::Mat::zeros(480, 640, CV_8UC3);

	P_left = cv::Mat::zeros(3,4,CV_64FC1);
	P_right = cv::Mat::zeros(3,4,CV_64FC1);

	freshCameraInfo = false;
};

ParticleFilter::~ParticleFilter() {

};

void ParticleFilter::initializeParticles() {
	ROS_INFO("---- Initialize particle is called---");
	matchingScores_arm_1.resize(numParticles); //initialize matching score array
	matchingScores_arm_2.resize(numParticles); //initialize matching score array

	particles_arm_1.resize(numParticles); //initialize particle array
	particleWeights_arm_1.resize(numParticles); //initialize particle weight array

	particles_arm_2.resize(numParticles); //initialize particle array
	particleWeights_arm_2.resize(numParticles); //initialize particle weight array

    /******Find and convert our various params and inputs******/
    //Get sensor update.
    kinematics = Davinci_fwd_solver();
    getCoarseGuess();

};

void ParticleFilter::getCoarseGuess(){
    //Pull in our first round of sensor data.
    davinci_interface::init_joint_feedback(node_handle);

    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[0];
        sensor_2 = tmp[1];
    }

    Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
    Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );

    Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
    Eigen::Vector3d a1_trans = a1_pos.translation();

    cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a1_pos, a1_rvec);

	predicted_real_pose.tvec_cyl(0) = a1_trans[0];
	predicted_real_pose.tvec_cyl(1) = a1_trans[1];
	predicted_real_pose.tvec_cyl(2) = a1_trans[2];
	predicted_real_pose.rvec_cyl(0) = a1_rvec.at<double>(0,0);
	predicted_real_pose.rvec_cyl(1) = a1_rvec.at<double>(1,0);
	predicted_real_pose.rvec_cyl(2) = a1_rvec.at<double>(2,0);

	newToolModel.computeEllipsePose(predicted_real_pose, sensor_1[4], sensor_1[5], sensor_1[6]);
    ROS_WARN_STREAM("sensor_1[6] " << sensor_1[6]);
    /*** first arm particles initialization ***/
	std::vector<double>  initialParticle;
	initialParticle.resize(19);
	for (int i = 0; i < 7; ++i) {
		initialParticle[i] = sensor_1[i];
	}

	cv::Mat rotationmatrix(3,3,CV_64FC1);
	cv::Mat p(3,1,CV_64FC1);
	rotationmatrix = Cam_left_arm_1.colRange(0,3).rowRange(0,3);
	p = Cam_left_arm_1.colRange(3,4).rowRange(0,3);
	cv::Mat cat_vec(3,1, CV_64FC1);
	cv::Rodrigues(rotationmatrix, cat_vec);

	initialParticle[7] = p.at<double>(0,0);
	initialParticle[8]= p.at<double>(1,0);
	initialParticle[9] = p.at<double>(2,0);

	initialParticle[10] = cat_vec.at<double>(0,0);
	initialParticle[11] = cat_vec.at<double>(1,0);
	initialParticle[12] = cat_vec.at<double>(2,0);

	rotationmatrix = Cam_right_arm_1.colRange(0,3).rowRange(0,3);
	p = Cam_right_arm_1.colRange(3,4).rowRange(0,3);
	cv::Rodrigues(rotationmatrix, cat_vec);

	initialParticle[13] = p.at<double>(0,0);
	initialParticle[14] = p.at<double>(1,0);
	initialParticle[15] = p.at<double>(2,0);

	initialParticle[16] = cat_vec.at<double>(0,0);
	initialParticle[17] = cat_vec.at<double>(1,0);
	initialParticle[18] = cat_vec.at<double>(2,0);

	computeNoisedParticles(initialParticle, particles_arm_1);
};

void ParticleFilter::computeNoisedParticles(std::vector <double> & inputParticle, std::vector< std::vector <double> > & noisedParticles){

	for (int i = 0; i < noisedParticles.size(); ++i) {
		noisedParticles[i].resize(19);

		for (int j = 0; j < 7; ++j) {
            double dev_sensor = newToolModel.randomNumber(0.0005, 0);
			inputParticle[j] = inputParticle[j] + dev_sensor;
		}
        /**** make noise for and right camera the same ****/
        std::vector<double> dev_cam;
        dev_cam.resize(6);

        for (int j = 7; j < 13; ++j) {
            dev_cam[j-7] = newToolModel.randomNumber(0.0001, 0);
			inputParticle[j] = inputParticle[j] + dev_cam[j-7];
		}

        for (int j = 13; j < 19; ++j) {
            inputParticle[j] = inputParticle[j] + dev_cam[j-13];
        }

		noisedParticles[i] = inputParticle;
	}
};

void ParticleFilter::projectionRightCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight){

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

void ParticleFilter::projectionLeftCB(const sensor_msgs::CameraInfo::ConstPtr &projectionLeft){

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

std::vector<cv::Mat> ParticleFilter::trackingTool(const cv::Mat &segmented_left, const cv::Mat &segmented_right) {
	t_step = ros::Time::now().toSec();  //get current time step
    ros::spinOnce();

	ROS_INFO("---- in tracking function ---");
	std::vector<cv::Mat> trackingImages;
	trackingImages.resize(2);

	/***Update according to the max score***/
	double maxScore_1 = 0.0;
	int maxScoreIdx_1 = -1; //maximum scored particle index
	double totalScore_1 = 0.0; //total score

	toolImage_left_temp.setTo(0);
	toolImage_right_temp.setTo(0);

	std::vector<ToolModel::toolModel> particle_models;
	particle_models.resize(numParticles);

	std::vector<cv::Mat> cam_matrices_left_arm_1;
	std::vector<cv::Mat> cam_matrices_right_arm_1;
	cam_matrices_left_arm_1.resize(numParticles);
	cam_matrices_right_arm_1.resize(numParticles);
	ToolModel::toolModel temp_model;

	for (int k = 0; k < numParticles; ++k) {

		Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], particles_arm_1[k][0] + DH_q_offset0 );
		Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], particles_arm_1[k][1] + DH_q_offset1 );
		Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], particles_arm_1[k][2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
		Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], particles_arm_1[k][3] + DH_q_offset3 );

		Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
		Eigen::Vector3d a1_trans = a1_pos.translation();

		cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
		computeRodriguesVec(a1_pos, a1_rvec);

		temp_model.tvec_cyl(0) = a1_trans[0];  //left and right (image frame)
		temp_model.tvec_cyl(1) = a1_trans[1];  //up and down
		temp_model.tvec_cyl(2) = a1_trans[2];
		temp_model.rvec_cyl(0) = a1_rvec.at<double>(0,0);
		temp_model.rvec_cyl(1) = a1_rvec.at<double>(1,0);
		temp_model.rvec_cyl(2) = a1_rvec.at<double>(2,0);

		newToolModel.computeEllipsePose(temp_model, particles_arm_1[k][4], particles_arm_1[k][5], particles_arm_1[k][6]);

		particle_models[k] = temp_model;

		/**** left camera ***/
		cv::Mat rotationmatrix(3,3,CV_64FC1);
		cv::Mat p(3,1,CV_64FC1);
		cv::Mat cat_vec(3,1, CV_64FC1);
		p.at<double>(0,0) = particles_arm_1[k][7];
		p.at<double>(1,0) = particles_arm_1[k][8];
		p.at<double>(2,0) = particles_arm_1[k][9];

		cat_vec.at<double>(0,0) = particles_arm_1[k][10];
		cat_vec.at<double>(1,0) = particles_arm_1[k][11];
		cat_vec.at<double>(2,0) = particles_arm_1[k][12];

		cv::Rodrigues(cat_vec, rotationmatrix);
		cam_matrices_left_arm_1[k] = cv::Mat::eye(4,4,CV_64FC1);
		rotationmatrix.copyTo(cam_matrices_left_arm_1[k].colRange(0,3).rowRange(0,3));
		p.copyTo(cam_matrices_left_arm_1[k].colRange(3,4).rowRange(0,3));

        /****right camera*/
		p.at<double>(0,0) = particles_arm_1[k][13];
		p.at<double>(1,0) = particles_arm_1[k][14];
		p.at<double>(2,0) = particles_arm_1[k][15];

		cat_vec.at<double>(0,0) = particles_arm_1[k][16];
		cat_vec.at<double>(1,0) = particles_arm_1[k][17];
		cat_vec.at<double>(2,0) = particles_arm_1[k][18];

		cv::Rodrigues(cat_vec, rotationmatrix);
		cam_matrices_right_arm_1[k] = cv::Mat::eye(4,4,CV_64FC1);
		rotationmatrix.copyTo(cam_matrices_right_arm_1[k].colRange(0,3).rowRange(0,3));
		p.copyTo(cam_matrices_right_arm_1[k].colRange(3,4).rowRange(0,3));

	}

	/*** do the sampling and get the matching score ***/
	for (int i = 0; i < numParticles; ++i) {
		matchingScores_arm_1[i] = measureFuncSameCam(toolImage_left_arm_1,toolImage_right_arm_1, particle_models[i], segmented_left, segmented_right,cam_matrices_left_arm_1[i],cam_matrices_right_arm_1[i]);

		newToolModel.renderTool(toolImage_left_temp, particle_models[i], cam_matrices_left_arm_1[i], P_left);
		newToolModel.renderTool(toolImage_right_temp, particle_models[i], cam_matrices_right_arm_1[i], P_right);

		if (matchingScores_arm_1[i] >= maxScore_1) {
			maxScore_1 = matchingScores_arm_1[i];
			maxScoreIdx_1 = i;
		}
		totalScore_1 += matchingScores_arm_1[i];
	}

	cv::imshow("temp image arm_1 left: " , toolImage_left_temp);
	cv::imshow("temp image arm_1 right:  " , toolImage_right_temp);
	ROS_INFO_STREAM("Maxscore arm 1: " << maxScore_1);  //debug

	/*** calculate weights using matching score and do the resampling ***/
	for (int j = 0; j < numParticles; ++j) { // normalize the weights
		particleWeights_arm_1[j] = (matchingScores_arm_1[j] / totalScore_1);
		//ROS_INFO_STREAM("weights" << particleWeights[j]);
	}

	ToolModel::toolModel best_tool_pose = particle_models[maxScoreIdx_1];
//	ROS_INFO("Real tool at (%f %f %f): %f %f %f, ", predicted_real_pose.tvec_cyl(0), predicted_real_pose.tvec_cyl(1),predicted_real_pose.tvec_cyl(2),predicted_real_pose.rvec_cyl(0),predicted_real_pose.rvec_cyl(1), predicted_real_pose.rvec_cyl(2));
//	ROS_WARN("Particle ARM AT (%f %f %f): %f %f %f, ",best_tool_pose.tvec_cyl(0), best_tool_pose.tvec_cyl(1),best_tool_pose.tvec_cyl(2),best_tool_pose.rvec_cyl(0),best_tool_pose.rvec_cyl(1), best_tool_pose.rvec_cyl(2));
//	////compute RMS errors before update
//	showGazeboToolError(predicted_real_pose, best_tool_pose);

	std::vector<double> best_particle = particles_arm_1[maxScoreIdx_1];

	///showing results for each iteration here
	newToolModel.renderTool(raw_image_left, particle_models[maxScoreIdx_1], cam_matrices_left_arm_1[maxScoreIdx_1], P_left);
	newToolModel.renderTool(raw_image_right, particle_models[maxScoreIdx_1], cam_matrices_right_arm_1[maxScoreIdx_1], P_right);
	// showing the best particle on left and right image
	trackingImages[0] = raw_image_left.clone();
	trackingImages[1] = raw_image_right.clone();
	cv::imshow("trackingImages left", trackingImages[0]);
	cv::imshow("trackingImages right", trackingImages[1]);

	//each time will clear the particles and resample them, resample using low variance resampling method
	std::vector<std::vector<double> > oldParticles = particles_arm_1;
	resamplingParticles(oldParticles, particleWeights_arm_1, particles_arm_1);

    cv::waitKey(20);
	updateParticles(best_particle, maxScore_1, particles_arm_1, predicted_real_pose);

	return trackingImages;
};

void ParticleFilter::showGazeboToolError(ToolModel::toolModel &real_pose, ToolModel::toolModel &bestParticle){

	double dim = 24;
	cv::Mat renderedMat = cv::Mat::zeros(dim,1,CV_64FC1);
	cv::Mat real_tool_vector = cv::Mat::zeros(dim,1,CV_64FC1);

	convertToolModeltoMatrix(bestParticle, renderedMat);
	convertToolModeltoMatrix(real_pose, real_tool_vector);

	cv::Mat diff = real_tool_vector - renderedMat;
	double error = diff.dot(diff);
	error = sqrt(error);

	ROS_WARN_STREAM("Position and orientation  error: " << error);
};

/***** update particles to find and reach to the best pose ***/
void ParticleFilter::updateParticles(std::vector <double> &best_particle_last, double &maxScore, std::vector<std::vector <double> > &updatedParticles, ToolModel::toolModel & predicted_real_pose) {
	std::vector<std::vector<double> > tmp;
	tmp.resize(2);
	if(davinci_interface::get_fresh_robot_pos(tmp)){
		sensor_1 = tmp[0];
		sensor_2 = tmp[1];
	}

	Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
	Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
	Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
	Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );

	Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
	Eigen::Vector3d a1_trans = a1_pos.translation();

	cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
	computeRodriguesVec(a1_pos, a1_rvec);

	predicted_real_pose.tvec_cyl(0) = a1_trans[0];
	predicted_real_pose.tvec_cyl(1) = a1_trans[1];
	predicted_real_pose.tvec_cyl(2) = a1_trans[2];
	predicted_real_pose.rvec_cyl(0) = a1_rvec.at<double>(0,0);
	predicted_real_pose.rvec_cyl(1) = a1_rvec.at<double>(1,0);
	predicted_real_pose.rvec_cyl(2) = a1_rvec.at<double>(2,0);

	double theta_oval = sensor_1[4];
	double theta_grip = sensor_1[5];
	double theta_open = sensor_1[6];

	newToolModel.computeEllipsePose(predicted_real_pose, theta_oval, theta_grip, theta_open);

	cv::Mat next_joint_estimate = cv::Mat::zeros(7,1,CV_64FC1);
	for (int i = 0; i < 7; ++i) {
		next_joint_estimate.at<double>(i,0) = sensor_1[i];
	}
	t_1_step = ros::Time::now().toSec();

	cv::Mat current_joint = cv::Mat::zeros(7,1,CV_64FC1);
	for (int i = 0; i < 7; ++i) {
		current_joint.at<double>(i,0) = best_particle_last[i];
	}

	cv::Mat delta_thetas = next_joint_estimate - current_joint;
	double delta_t = t_1_step - t_step;
	cv::Mat nom_vel = delta_thetas *  (1 / delta_t);
    double velocity_bar = cv::sum( nom_vel )[0];

    if(fabs(velocity_bar) > 0.01){

        ROS_INFO_STREAM("velocity_bar " << velocity_bar);
        /******** using the obtained velocity to propagate the particles ********/
        for (int j = 0; j < updatedParticles.size(); ++j) {
            for (int i = 0; i < 7; ++i) {
                current_joint.at<double>(i,0) = updatedParticles[j][i];
            }
            cv::Mat new_joint_state = current_joint + nom_vel * delta_t;
            for (int k = 0; k < 7; ++k) {
                updatedParticles[j][k] = new_joint_state.at<double>(k,0);
            }

            cv::Mat rotationmatrix(3,3,CV_64FC1);
            cv::Mat p(3,1,CV_64FC1);
            rotationmatrix = Cam_left_arm_1.colRange(0,3).rowRange(0,3);
            p = Cam_left_arm_1.colRange(3,4).rowRange(0,3);
            cv::Mat cat_vec(3,1, CV_64FC1);
            cv::Rodrigues(rotationmatrix, cat_vec);

            updatedParticles[j][7] = p.at<double>(0,0);
            updatedParticles[j][8]= p.at<double>(1,0);
            updatedParticles[j][9] = p.at<double>(2,0);

            updatedParticles[j][10] = cat_vec.at<double>(0,0);
            updatedParticles[j][11] = cat_vec.at<double>(1,0);
            updatedParticles[j][12] = cat_vec.at<double>(2,0);

            rotationmatrix = Cam_right_arm_1.colRange(0,3).rowRange(0,3);
            p = Cam_right_arm_1.colRange(3,4).rowRange(0,3);
            cv::Rodrigues(rotationmatrix, cat_vec);

            updatedParticles[j][13] = p.at<double>(0,0);
            updatedParticles[j][14] = p.at<double>(1,0);
            updatedParticles[j][15] = p.at<double>(2,0);

            updatedParticles[j][16] = cat_vec.at<double>(0,0);
            updatedParticles[j][17] = cat_vec.at<double>(1,0);
            updatedParticles[j][18] = cat_vec.at<double>(2,0);
        }

    }

    ROS_INFO_STREAM("down_sample_rate " << down_sample_rate);
    if(maxScore > 1.25){
        down_sample_rate = 0.0002;
    }
    /**** add noise for propagated particles ****/
    for (int m = 1; m < updatedParticles.size(); ++m) {

        for (int l = 0; l < 7; ++l) {
            double dev_sensor = newToolModel.randomNumber(down_sample_rate, 0);
            updatedParticles[m][l] = updatedParticles[m][l] + dev_sensor;
        }

        std::vector<double> dev_cam;
        dev_cam.resize(6);

        for (int j = 7; j < 13; ++j) {
            dev_cam[j-7] = newToolModel.randomNumber(down_sample_rate, 0);
            updatedParticles[m][j] = updatedParticles[m][j] + dev_cam[j-7];
        }

        for (int j = 13; j < 19; ++j) {
            updatedParticles[m][j] = updatedParticles[m][j] + dev_cam[j-13];
        }

    }

    down_sample_rate -= 0.0001;
    if(down_sample_rate < 0.0002){
        down_sample_rate = 0.0002;
    };


};

double ParticleFilter::measureFuncSameCam(cv::Mat & toolImage_left, cv::Mat & toolImage_right, ToolModel::toolModel &toolPose,
		const cv::Mat &segmented_left, const cv::Mat &segmented_right, cv::Mat &Cam_left, cv::Mat &Cam_right) {

	toolImage_left.setTo(0);
	toolImage_right.setTo(0);

	/***do the sampling and get the matching score***/
	//first get the rendered image using 3d model of the tool
	newToolModel.renderTool(toolImage_left, toolPose, Cam_left, P_left);
	double left = newToolModel.calculateChamferScore(toolImage_left, segmented_left);  //get the matching score

	newToolModel.renderTool(toolImage_right, toolPose, Cam_right, P_right);
	double right = newToolModel.calculateChamferScore(toolImage_right, segmented_right);

	double matchingScore = sqrt(pow(left, 2) + pow(right, 2));

	return matchingScore;
};

/**** resampling method ****/
void ParticleFilter::resamplingParticles(const std::vector< std::vector<double> > &sampleModel,
										 const std::vector<double> &particleWeight,
										 std::vector<std::vector<double> > &update_particles) {

	int M = sampleModel.size(); //total number of particles
	double max = 1.0 / M;

	double r = newToolModel.randomNum(0.0, max);
	double w = particleWeight[0]; //first particle weight
	int idx = 0;

	update_particles.clear(); ///start fresh

	for (int i = 0; i < M; ++i) {

		double U = r + ((double) (i - 1) * max);

		while (U > w) {
			idx += 1;
			w = w + particleWeight[idx];
		}

		update_particles.push_back(sampleModel[idx]);
	}

};

void ParticleFilter::computeRodriguesVec(const Eigen::Affine3d & trans, cv::Mat rot_vec){
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

/******from eigen to opencv matrix****/
void ParticleFilter::convertEigenToMat(const Eigen::Affine3d & trans, cv::Mat & outputMatrix){

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

void ParticleFilter::convertToolModeltoMatrix(const ToolModel::toolModel &inputToolModel, cv::Mat &toolMatrix){
	toolMatrix.at<double>(0,0) = inputToolModel.tvec_cyl(0);
	toolMatrix.at<double>(1,0) = inputToolModel.tvec_cyl(1);
	toolMatrix.at<double>(2,0) = inputToolModel.tvec_cyl(2);

	toolMatrix.at<double>(3,0) = inputToolModel.rvec_cyl(0);
	toolMatrix.at<double>(4,0) = inputToolModel.rvec_cyl(1);
	toolMatrix.at<double>(5,0) = inputToolModel.rvec_cyl(2);

	toolMatrix.at<double>(6,0) = inputToolModel.tvec_elp(0);
	toolMatrix.at<double>(7,0) = inputToolModel.tvec_elp(1);
	toolMatrix.at<double>(8,0) = inputToolModel.tvec_elp(2);

	toolMatrix.at<double>(9,0) = inputToolModel.rvec_elp(0);
	toolMatrix.at<double>(10,0) = inputToolModel.rvec_elp(1);
	toolMatrix.at<double>(11,0) = inputToolModel.rvec_elp(2);

	toolMatrix.at<double>(12,0) = inputToolModel.tvec_grip1(0);
	toolMatrix.at<double>(13,0) = inputToolModel.tvec_grip1(1);
	toolMatrix.at<double>(14,0) = inputToolModel.tvec_grip1(2);

	toolMatrix.at<double>(15,0) = inputToolModel.rvec_grip1(0);
	toolMatrix.at<double>(16,0) = inputToolModel.rvec_grip1(1);
	toolMatrix.at<double>(17,0) = inputToolModel.rvec_grip1(2);

	toolMatrix.at<double>(18,0) = inputToolModel.tvec_grip2(0);
	toolMatrix.at<double>(19,0) = inputToolModel.tvec_grip2(1);
	toolMatrix.at<double>(20,0) = inputToolModel.tvec_grip2(2);

	toolMatrix.at<double>(21,0) = inputToolModel.rvec_grip2(0);
	toolMatrix.at<double>(22,0) = inputToolModel.rvec_grip2(1);
	toolMatrix.at<double>(23,0) = inputToolModel.rvec_grip2(2);
};