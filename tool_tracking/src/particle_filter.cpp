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
        node_handle(*nodehandle), numParticles(100){

	initializeParticles();

	/**** need to subscribe this for simulation ***/
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

//    ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
//    ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);
//    ROS_INFO_STREAM("Cam_left_arm_2: " << Cam_left_arm_2);
//    ROS_INFO_STREAM("Cam_right_arm_2: " << Cam_right_arm_2);
//
//	Cam_left_arm_1 = (cv::Mat_<double>(4,4) << -0.8361,0.5340, 0.1264, -0.1142,
//	0.5132, 0.8424, -0.1641, -0.0262,
//	-0.1942, -0.0723, -0.9783, 0.1273,
//	0,0, 0,1.0000);
//
//	Cam_right_arm_1 = (cv::Mat_<double>(4,4) << -0.8361,0.5340, 0.1264, -0.1192,
//			0.5132, 0.8424, -0.1641, -0.0262,
//			-0.1942, -0.0723, -0.9783, 0.1273,
//			0,0, 0,1.0000);

	ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);

	projectionMat_subscriber_r = node_handle.subscribe("/davinci_endo/right/camera_info", 1, &ParticleFilter::projectionRightCB, this);
	projectionMat_subscriber_l = node_handle.subscribe("/davinci_endo/left/camera_info", 1, &ParticleFilter::projectionLeftCB, this);

	// initialization, just basic black image ??? how to get the size of the image
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

//    Eigen::Affine3d a1_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_1.data()));
//
//    Eigen::Vector3d a1_trans = a1_pos.translation();
//    cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
//    computeRodriguesVec(a1_pos, a1_rvec);

    Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
    Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );

    Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
    Eigen::Vector3d a1_trans = a1_pos.translation();

    cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a1_pos, a1_rvec);

    /*** first arm particles initialization ***/
    initial.tvec_cyl(0) = a1_trans[0];  //left and right (image frame)
    initial.tvec_cyl(1) = a1_trans[1];  //up and down
    initial.tvec_cyl(2) = a1_trans[2];
    initial.rvec_cyl(0) = a1_rvec.at<double>(0,0);
    initial.rvec_cyl(1) = a1_rvec.at<double>(1,0);
    initial.rvec_cyl(2) = a1_rvec.at<double>(2,0);

    double theta_cylinder = tmp[0][4]; //initial guess
    double theta_oval = tmp[0][5]; //initial guess
    double theta_open = tmp[0][6]; //initial guess

    for (int i = 0; i < numParticles; i++) {
        particles_arm_1[i] = newToolModel.setRandomConfig(initial, theta_cylinder, theta_oval, theta_open );
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

    ros::spinOnce();

	ROS_INFO("---- in tracking function ---");
    // ROS_INFO("---- Inside tracking function ---");
	std::vector<cv::Mat> trackingImages;
	trackingImages.resize(2);

	/***Update according to the max score***/

	double maxScore_1 = 0.0;
	double maxScore_2 = 0.0;

	int maxScoreIdx_1 = -1; //maximum scored particle index
	int maxScoreIdx_2 = -1; //maximum scored particle index

	double totalScore_1 = 0.0; //total score
	double totalScore_2 = 0.0; //total score

	toolImage_left_temp.setTo(0);
	toolImage_right_temp.setTo(0);

	/***do the sampling and get the matching score***/
	for (int i = 0; i < numParticles; ++i) {

		matchingScores_arm_1[i] = measureFuncSameCam(toolImage_left_arm_1,toolImage_right_arm_1,particles_arm_1[i],segmented_left, segmented_right,Cam_left_arm_1,Cam_right_arm_1);

		newToolModel.renderTool(toolImage_left_temp, particles_arm_1[i], Cam_left_arm_1, P_left);
		newToolModel.renderTool(toolImage_right_temp, particles_arm_1[i], Cam_right_arm_1, P_right);


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

	ToolModel::toolModel best_particle = particles_arm_1[maxScoreIdx_1];
	ROS_WARN("Particle ARM AT (%f %f %f): %f %f %f, ",best_particle.tvec_cyl(0), best_particle.tvec_cyl(1),best_particle.tvec_cyl(2),best_particle.rvec_cyl(0),best_particle.rvec_cyl(1), best_particle.rvec_cyl(2));

	///showing results for each iteration here
	//render in segmented image
	newToolModel.renderTool(raw_image_left, particles_arm_1[maxScoreIdx_1], Cam_left_arm_1, P_left);
	newToolModel.renderTool(raw_image_right, particles_arm_1[maxScoreIdx_1], Cam_right_arm_1, P_right);
	// showing the best particle on left and right image
	trackingImages[0] = raw_image_left.clone();
	trackingImages[1] = raw_image_right.clone();
	cv::imshow("trackingImages left", trackingImages[0]);
	cv::imshow("trackingImages right", trackingImages[1]);

	//each time will clear the particles and resample them, resample using low variance resampling method
	std::vector<ToolModel::toolModel> oldParticles = particles_arm_1;
	resamplingParticles(oldParticles, particleWeights_arm_1, particles_arm_1);

	//std::vector<ToolModel::toolModel> updatedParticles = particles;
	updateParticles(particles_arm_1);

	return trackingImages;
};


/***** update particles to find and reach to the best pose ***/
void ParticleFilter::updateParticles(std::vector<ToolModel::toolModel> &updatedParticles) {
    ///every loop should generate different particle from one base particle k
    for (int i = 0; i < numParticles; ++i) {
		updatedParticles[i] = newToolModel.gaussianSampling(updatedParticles[i]);  //generate new particles with new deviation
    }

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

cv::Mat ParticleFilter::adjoint(cv::Mat &G) {
	cv::Mat adjG = cv::Mat::zeros(6, 6, CV_64F);

	cv::Mat Rot = G.colRange(0, 3).rowRange(0, 3);
	cv::Mat p = G.colRange(3, 4).rowRange(0, 3);

	cv::Mat p_skew = newToolModel.computeSkew(p);
	//upper right corner
	cv::Mat temp = p_skew * Rot;
	Rot.copyTo(adjG.colRange(0, 3).rowRange(0, 3));
	Rot.copyTo(adjG.colRange(3, 6).rowRange(3, 6));
	temp.copyTo(adjG.colRange(3, 6).rowRange(0, 3));
	return adjG;
};

/**** resampling method ****/
void ParticleFilter::resamplingParticles(const std::vector<ToolModel::toolModel> &sampleModel,
										 const std::vector<double> &particleWeight,
										 std::vector<ToolModel::toolModel> &update_particles) {

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

/*** one possible motion model for tool in image frame, update particles based on given spatial velocity ***/
void ParticleFilter::updateSamples(const cv::Mat &bodyVel, double &updateRate, std::vector<ToolModel::toolModel> particles) {

	cv::Mat Rot = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat p = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat particleFrame = cv::Mat::eye(4, 4, CV_64F);

	cv::Mat spatialVel = cv::Mat::zeros(6, 1, CV_64F);
	cv::Mat updatedParticleFrame = cv::Mat::eye(4, 4, CV_64F);

	cv::Mat I = cv::Mat::eye(3, 3, CV_64F);

	for (int k = 0; k < particles.size(); ++k) {

		cv::Rodrigues(particles[k].rvec_cyl, Rot); //get rotation mat from cylinder
		p.at<double>(0, 0) = particles[k].tvec_cyl(0); //get translation vec from cylinder
		p.at<double>(1, 0) = particles[k].tvec_cyl(1);
		p.at<double>(2, 0) = particles[k].tvec_cyl(2);

		Rot.copyTo(particleFrame.colRange(0, 3).rowRange(0, 3));
		p.copyTo(particleFrame.colRange(3, 4).rowRange(0, 3));

		//calculate spatial velocity
		spatialVel = adjoint(particleFrame) * bodyVel;
		// spatialVel = addNoise(spatialVel);

		cv::Mat v = spatialVel.colRange(0, 1).rowRange(0, 3); //translation velocity
		cv::Mat w = spatialVel.colRange(0, 1).rowRange(3, 6); //rotational velocity

		cv::Mat move = cv::Mat::eye(4, 4, CV_64F);

		if (w.at<double>(2, 0) == 0.0) {  //TODO: pure translation ??
			cv::Mat vdt = v * updateRate;
			vdt.copyTo(move.colRange(3, 4).rowRange(0, 3));
		} else {
			cv::Mat vtil = v * updateRate;
			cv::Mat wtil = w * updateRate;

			double M = cv::norm(wtil);
			cv::Mat v_bar = vtil / M;  //h = w*v//||w||^2
			cv::Mat w_bar = wtil / M;

			cv::Mat w_hat = newToolModel.computeSkew(w_bar);
			cv::Mat rotation = I + w_hat * sin(M) + (w_hat * w_hat) * (1 - cos(M));
			cv::Mat trans = (I - rotation) * (w_bar.cross(v_bar) + w_bar * w_bar.t() * v_bar * M);

			rotation.copyTo(move.colRange(0, 3).rowRange(0, 3));
			trans.copyTo(move.colRange(3, 4).rowRange(0, 3));
		}

		/***update the cylinder pose***/
		updatedParticleFrame = move * particleFrame;

		//convert rotation matrix to Rodrigues
		cv::Mat tempR = cv::Mat::zeros(3, 3, CV_64F);
		cv::Mat updateR = cv::Mat::zeros(3, 1, CV_64F);
		cv::Mat updateT = cv::Mat::zeros(3, 1, CV_64F);

		updateT = updatedParticleFrame.colRange(3, 4).rowRange(0, 3);
		tempR = updatedParticleFrame.colRange(0, 3).rowRange(0, 3);
		cv::Rodrigues(tempR, updateR);

		particles[k].tvec_cyl(0) = updateT.at<double>(0, 0);
		particles[k].tvec_cyl(1) = updateT.at<double>(1, 0);
		particles[k].tvec_cyl(2) = updateT.at<double>(2, 0);
		particles[k].rvec_cyl(0) = updateR.at<double>(0, 0);
		particles[k].rvec_cyl(1) = updateR.at<double>(1, 0);
		particles[k].rvec_cyl(2) = updateR.at<double>(2, 0);

		/***according to the cylinder pose, update ellipse and grippers pose***/
		//newToolModel.computeDavinciPose(particles[k], 0.0, 0.0, 0.0); // no need to change relative angles??? TODO:

	}
};