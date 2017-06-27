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
        node_handle(*nodehandle), numParticles(200), downsample_rate_pos(0.003), downsample_rate_rot(0.02), error_pos(1), error_ori(1){

	initializeParticles();

	//Intermediary variable representing Gcb
	//Will be converted to Affind3d arm_x_cam_y and then into cv::mat Cam_x_arm_y
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

	//Intermediary variables representing Gcb
	//Will be converted to cv::mat Cam_x_arm_y
	XformUtils xfu;
    arm_1__cam_l = xfu.transformTFToAffine3d(arm_1__cam_l_st);//.inverse();
    arm_1__cam_r = xfu.transformTFToAffine3d(arm_1__cam_r_st);//.inverse();
    arm_2__cam_l = xfu.transformTFToAffine3d(arm_2__cam_l_st);//.inverse();
    arm_2__cam_r = xfu.transformTFToAffine3d(arm_2__cam_r_st);//.inverse();

	//cv::mat representation of Gcb
	convertEigenToMat(arm_1__cam_l, Cam_left_arm_1);
    convertEigenToMat(arm_1__cam_r, Cam_right_arm_1);
    convertEigenToMat(arm_2__cam_l, Cam_left_arm_2);
    convertEigenToMat(arm_2__cam_r, Cam_right_arm_2);

	ROS_INFO_STREAM("Cam_left_arm_1: " << Cam_left_arm_1);
	ROS_INFO_STREAM("Cam_right_arm_1: " << Cam_right_arm_1);

	projectionMat_subscriber_r = node_handle.subscribe("/davinci_endo/right/camera_info", 1, &ParticleFilter::projectionRightCB, this);
	projectionMat_subscriber_l = node_handle.subscribe("/davinci_endo/left/camera_info", 1, &ParticleFilter::projectionLeftCB, this);

	toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);  //left rendered Image for ARM 1
	toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3); //right rendered Image for ARM 1

	toolImage_left_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3); //left rendered Image for ARM 2
	toolImage_right_arm_2 = cv::Mat::zeros(480, 640, CV_8UC3); //right rendered Image for ARM 2

	toolImage_left_temp = cv::Mat::zeros(480, 640, CV_8UC3); //left rendered Image
	toolImage_right_temp = cv::Mat::zeros(480, 640, CV_8UC3); //right rendered Image

	raw_image_left = cv::Mat::zeros(480, 640, CV_8UC3);
	raw_image_right = cv::Mat::zeros(480, 640, CV_8UC3);

	P_left = cv::Mat::zeros(3,4,CV_64FC1);
	P_right = cv::Mat::zeros(3,4,CV_64FC1);

	freshCameraInfo = false;

	pos_thresh = 0.001;
	rot_thresh = 0.01;
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

	//Compute initial guess from FK and generate particles around initial guess
    kinematics = Davinci_fwd_solver();
    getCoarseGuess();
};

void ParticleFilter::getCoarseGuess(){
	//Initialize joint feedback data
    davinci_interface::init_joint_feedback(node_handle);
	//store robot joint angles
    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[0];
        sensor_2 = tmp[1];
    }

	//Compute position variable a1_rvec representing joints 1-4 (3x1 cv::mat) using FK and DH parameters
	Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
    Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );
    Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
    Eigen::Vector3d a1_trans = a1_pos.translation();
    cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a1_pos, a1_rvec);

	//toolModel representation of initial guess of configuration of joints 1-4 computed above
    initial.tvec_cyl(0) = a1_trans[0] + 0.008;  //left and right (image frame)
    initial.tvec_cyl(1) = a1_trans[1]+ 0.007;  //up and down
    initial.tvec_cyl(2) = a1_trans[2];// + 0.004;
    initial.rvec_cyl(0) = a1_rvec.at<double>(0,0) + 0.04;
    initial.rvec_cyl(1) = a1_rvec.at<double>(1,0) + 0.03;
    initial.rvec_cyl(2) = a1_rvec.at<double>(2,0);

    double theta_cylinder = sensor_1[4]; //guess from sensor
    double theta_oval = sensor_1[5]; //guess from sensor
    double theta_open = sensor_1[6]; //guess from sensor

	//Randomly generate particles around initial guess
	//Particles stored as a vector of toolModels
    for (int i = 0; i < numParticles; i++) {
        particles_arm_1[i] = newToolModel.setRandomConfig(initial, theta_cylinder, theta_oval, theta_open, downsample_rate_pos, downsample_rate_rot);
    }

	//Update initial for compute errors in gazebo
	initial.tvec_cyl(0) = a1_trans[0];  //left and right (image frame)
	initial.tvec_cyl(1) = a1_trans[1];  //up and down
	initial.tvec_cyl(2) = a1_trans[2];  // + 0.004;
	initial.rvec_cyl(0) = a1_rvec.at<double>(0,0);
	initial.rvec_cyl(1) = a1_rvec.at<double>(1,0);
	initial.rvec_cyl(2) = a1_rvec.at<double>(2,0);
	newToolModel.computeEllipsePose(initial, theta_cylinder, theta_oval, theta_open);

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
//	double t_step = clock();
    ros::spinOnce();
	ROS_INFO("---- in tracking function ---");
	std::vector<cv::Mat> trackingImages;
	trackingImages.resize(2);

	//Update according to the max score
	double maxScore_1 = 0.0;
	int maxScoreIdx_1 = -1; //maximum scored particle index
	double totalScore_1 = 0.0; //total score

	toolImage_left_temp.setTo(0);
	toolImage_right_temp.setTo(0);
/*+ 0.003*/
    ////////////////////////PART 1: Measurement model//////////////////////////////
    for (int i = 0; i < numParticles; ++i) {
        //get matching score of particle i
		matchingScores_arm_1[i] = measureFuncSameCam(toolImage_left_arm_1,toolImage_right_arm_1,particles_arm_1[i],segmented_left, segmented_right,Cam_left_arm_1,Cam_right_arm_1);

        //render tool in left and right camera frame on temp image
		newToolModel.renderTool(toolImage_left_temp, particles_arm_1[i], Cam_left_arm_1, P_left);
		newToolModel.renderTool(toolImage_right_temp, particles_arm_1[i], Cam_right_arm_1, P_right);

        //store total match score and largest discovered match score/index
		if (matchingScores_arm_1[i] >= maxScore_1) {
			maxScore_1 = matchingScores_arm_1[i];
			maxScoreIdx_1 = i;
		}
		totalScore_1 += matchingScores_arm_1[i];
	}

    //show composite of all particles in left/right camera on temp images
//	cv::imshow("temp image arm_1 left: " , toolImage_left_temp);
//	cv::imshow("temp image arm_1 right:  " , toolImage_right_temp);
//	ROS_INFO_STREAM("Maxscore arm 1: " << maxScore_1);  //debug

    //compute weights as normalized matching scores
	for (int j = 0; j < numParticles; ++j) {
		particleWeights_arm_1[j] = (matchingScores_arm_1[j] / totalScore_1);
		//ROS_INFO_STREAM("weights" << particleWeights[j]);
	}

    //store the best particle (that with highest weight)
	ToolModel::toolModel best_particle = particles_arm_1[maxScoreIdx_1];
//	ROS_INFO("Real tool at (%f %f %f): %f %f %f, ", initial.tvec_cyl(0), initial.tvec_cyl(1),initial.tvec_cyl(2),initial.rvec_cyl(0),initial.rvec_cyl(1), initial.rvec_cyl(2));
//	ROS_WARN("Particle ARM AT (%f %f %f): %f %f %f, ",best_particle.tvec_cyl(0), best_particle.tvec_cyl(1),best_particle.tvec_cyl(2),best_particle.rvec_cyl(0),best_particle.rvec_cyl(1), best_particle.rvec_cyl(2));

	///render the best particle on real images and show
	newToolModel.renderTool(raw_image_left, particles_arm_1[maxScoreIdx_1], Cam_left_arm_1, P_left);
	newToolModel.renderTool(raw_image_right, particles_arm_1[maxScoreIdx_1], Cam_right_arm_1, P_right);
	trackingImages[0] = raw_image_left.clone();
	trackingImages[1] = raw_image_right.clone();
	cv::imshow("trackingImages left", trackingImages[0]);
	cv::imshow("trackingImages right", trackingImages[1]);

	//////////////////////PART 2: Resampling/////////////////////////////////////
	std::vector<ToolModel::toolModel> oldParticles = particles_arm_1;
	resamplingParticles(oldParticles, particleWeights_arm_1, particles_arm_1);

//	double t1_step = clock();
//
//	float sec1 = (float) t1_step / CLOCKS_PER_SEC;
//	float sec = (float) t_step / CLOCKS_PER_SEC;
//	ROS_INFO_STREAM("Delta t = " << sec1 - sec);

	showGazeboToolError(initial, best_particle);
	/////////////////////////PART 3: Motion model////////////////////////////////
	updateParticles(particles_arm_1);
	cv::waitKey(20);
	return trackingImages;
};

void ParticleFilter::showGazeboToolError(ToolModel::toolModel &real_pose, ToolModel::toolModel &bestParticle){

	//Get matrix representations of the toolModels
	int dim = 24;
	cv::Mat renderedMat = cv::Mat::zeros(dim,1,CV_64FC1);
	cv::Mat real_tool_vector = cv::Mat::zeros(dim,1,CV_64FC1);
	convertToolModeltoMatrix(bestParticle, renderedMat);
	convertToolModeltoMatrix(real_pose, real_tool_vector);

	//Find difference in pos/orientation for real_pose and particle_pose
	cv::Mat position = real_tool_vector.rowRange(0, 12) - renderedMat.rowRange(0,12);
	cv::Mat orientation = real_tool_vector.rowRange(12, dim) - renderedMat.rowRange(12,dim);
	double total_pos = position.dot(position);
	double total_ori = orientation.dot(orientation);
	error_pos = sqrt(total_pos / 12);
	error_ori = sqrt(total_ori / 12);
	ROS_WARN_STREAM("Position  error: " << error_pos);
	ROS_WARN_STREAM("orientation  error: " << error_ori);
};

void ParticleFilter::updateParticles(std::vector<ToolModel::toolModel> &updatedParticles) {
	ROS_INFO_STREAM("downsample_rate_pos " << downsample_rate_pos); //annealing coefficient
	ROS_INFO_STREAM("downsample_rate_rot " << downsample_rate_rot); //annealing coefficient
	//decrement annealing coefficient every time function is called
	downsample_rate_pos -= 0.0004;
	if(downsample_rate_pos < 0.0002){
		downsample_rate_pos = 0.0002;
	};

	downsample_rate_rot -= 0.004;
	if(downsample_rate_rot < 0.004){
		downsample_rate_rot = 0.0003;
	};

	//If error is small decrement it even more
	if(error_pos < pos_thresh){
		downsample_rate_pos = 0.0001;
	}
	if(error_ori < rot_thresh){
		downsample_rate_rot = 0.0001;
	}

	//Move particles through gaussian noise motion model
	//after resampling, first particle is best -> save it in the searching pool
    for (int i = 1; i < numParticles; ++i) {
		updatedParticles[i] = newToolModel.gaussianSampling(updatedParticles[i], downsample_rate_pos, downsample_rate_rot);
    }

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

	initial.tvec_cyl(0) = a1_trans[0];
	initial.tvec_cyl(1) = a1_trans[1];
	initial.tvec_cyl(2) = a1_trans[2];
	initial.rvec_cyl(0) = a1_rvec.at<double>(0,0);
	initial.rvec_cyl(1) = a1_rvec.at<double>(1,0);
	initial.rvec_cyl(2) = a1_rvec.at<double>(2,0);

	double theta_oval = sensor_1[4];
	double theta_grip = sensor_1[5];
	double theta_open = sensor_1[6];

	newToolModel.computeEllipsePose(initial, theta_oval, theta_grip, theta_open);

};

double ParticleFilter::measureFuncSameCam(cv::Mat & toolImage_left, cv::Mat & toolImage_right, ToolModel::toolModel &toolPose,
		const cv::Mat &segmented_left, const cv::Mat &segmented_right, cv::Mat &Cam_left, cv::Mat &Cam_right) {

	toolImage_left.setTo(0); //clear the left and right images
	toolImage_right.setTo(0);

	newToolModel.renderTool(toolImage_left, toolPose, Cam_left, P_left); //render particle's pose of tool on left image in left camera's frame
	double left = newToolModel.calculateChamferScore(toolImage_left, segmented_left);  //get the matching score between particle pose and segmented pose

	newToolModel.renderTool(toolImage_right, toolPose, Cam_right, P_right);//render particle's pose of tool on right image in right camera's frame
	double right = newToolModel.calculateChamferScore(toolImage_right, segmented_right); //get the matching score between particle pose and segmented pose

	double matchingScore = sqrt(pow(left, 2) + pow(right, 2));

	return matchingScore;
};

void ParticleFilter::resamplingParticles(const std::vector<ToolModel::toolModel> &sampleModel,
										 const std::vector<double> &particleWeight,
										 std::vector<ToolModel::toolModel> &update_particles) {
	int M = sampleModel.size(); //total number of particles
	double max = 1.0 / M; //uniform probability of drawing a particle

	double r = newToolModel.randomNum(0.0, max); //random double between 0 and max
	double w = particleWeight[0]; //first particle weight
	int idx = 0;

	update_particles.clear(); //clear new particles

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