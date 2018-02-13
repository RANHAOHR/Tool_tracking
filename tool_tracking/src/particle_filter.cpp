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
using cv_projective::reprojectPoint;
using cv_projective::transformPoints;

ParticleFilter::ParticleFilter(ros::NodeHandle *nodehandle):
        node_handle(*nodehandle), numParticles(800), downsample_rate_pos(0.003), downsample_rate_rot(0.003){

    error_vec.resize(2);

    error_vec[0] =10;
    error_vec[1] =10;
    initial_error_vec.resize(2);

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

	projectionMat_subscriber_r = node_handle.subscribe("/davinci_endo/unsynched/right/camera_info", 1, &ParticleFilter::projectionRightCB, this);
	projectionMat_subscriber_l = node_handle.subscribe("/davinci_endo/unsynched/left/camera_info", 1, &ParticleFilter::projectionLeftCB, this);

	toolImage_left_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3);  //left rendered Image for ARM 1
	toolImage_right_arm_1 = cv::Mat::zeros(480, 640, CV_8UC3); //right rendered Image for ARM 1

	toolImage_left_temp = cv::Mat::zeros(480, 640, CV_8UC3); //left rendered Image
	toolImage_right_temp = cv::Mat::zeros(480, 640, CV_8UC3); //right rendered Image

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

	particles_arm_1.resize(numParticles); //initialize particle array
	particleWeights_arm_1.resize(numParticles); //initialize particle weight array

	//Compute initial guess from FK and generate particles around initial guess
    kinematics = Davinci_fwd_solver();

	pos_noise.resize(3);
	rot_noise.resize(3);
    getCoarseGuess();
};

void ParticleFilter::getCoarseGuess(){
	//Initialize joint feedback data
    davinci_interface::init_joint_feedback(node_handle);
	//store robot joint angles
    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[1];
        sensor_2 = tmp[0];
    }

    /* first get the real pose */
   	Eigen::Matrix<double, 7, 1> temp_sensor_1;
	for (int i = 0; i < 7; ++i)
	{
		temp_sensor_1[i] = sensor_1[i];
	}

    end_effector = kinematics.fwd_kin_solve(temp_sensor_1);
	// //Compute position variable a1_rvec representing joints 1-4 (3x1 cv::mat) using FK and DH parameters
	Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
    Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );
    Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
    Eigen::Vector3d a1_trans = a1_pos.translation();
    cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a1_pos, a1_rvec);
	//Update initial for compute errors in gazebo, for static tracking
	real_tool.tvec_cyl(0) = a1_trans[0];  //left and right (image frame)
	real_tool.tvec_cyl(1) = a1_trans[1];  //up and down
	real_tool.tvec_cyl(2) = a1_trans[2];  // + 0.004;
	real_tool.rvec_cyl(0) = a1_rvec.at<double>(0,0);
	real_tool.rvec_cyl(1) = a1_rvec.at<double>(1,0);
	real_tool.rvec_cyl(2) = a1_rvec.at<double>(2,0);

    double theta_caudier = sensor_1[4]; //guess from sensor
    double theta_gripper = sensor_1[5]; //guess from sensor
    double theta_open = sensor_1[6]; //guess from sensor

	newToolModel.computeEllipsePose(real_tool, theta_caudier, theta_gripper, theta_open);


	/*get the noised pose*/
    sensor_1[0] = sensor_1[0] * 1.1;
	//Compute position variable a1_rvec representing joints 1-4 (3x1 cv::mat) using FK and DH parameters
	a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0], sensor_1[0] + DH_q_offset0 );
    a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1], sensor_1[1] + DH_q_offset1 );
    a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2, DH_alpha_params[2], 0.0 );
    a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3],  DH_d4, DH_alpha_params[3], sensor_1[3] + DH_q_offset3 );
    a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3* a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
    a1_trans = a1_pos.translation();
    a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a1_pos, a1_rvec);

	//toolModel representation of initial guess of configuration of joints 1-4 computed above
    initial.tvec_cyl(0) = a1_trans[0];// + pos_noise[0];  //left and right (image frame)
    initial.tvec_cyl(1) = a1_trans[1];//+ pos_noise[1];  //up and down
    initial.tvec_cyl(2) = a1_trans[2];//+ pos_noise[2];
    initial.rvec_cyl(0) = a1_rvec.at<double>(0,0);// + rot_noise[0];
    initial.rvec_cyl(1) = a1_rvec.at<double>(1,0);// + rot_noise[1];
    initial.rvec_cyl(2) = a1_rvec.at<double>(2,0);// + rot_noise[2];

    theta_caudier = sensor_1[4]; //guess from sensor
    theta_gripper = sensor_1[5]; //guess from sensor
    theta_open = sensor_1[6]; //guess from sensor

    newToolModel.computeEllipsePose(initial, theta_caudier, theta_gripper, theta_open);

    initial_error_vec = endEffectorError(end_effector, real_tool, initial);
    // initial_error_vec = showGazeboToolError(real_tool, initial);

	//Randomly generate particles around initial guess,	Particles stored as a vector of toolModels
    for (int i = 0; i < numParticles; i++) {
        particles_arm_1[i] = newToolModel.setRandomConfig(initial, theta_caudier, theta_gripper, theta_open, downsample_rate_pos, downsample_rate_rot);
    }

	ROS_INFO_STREAM("real_tool tvec cly" << cv::Mat(real_tool.tvec_cyl));
	last_tool = real_tool;
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

	// ROS_INFO_STREAM("right: " << P_right);
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

	// ROS_INFO_STREAM("left: " << P_left);
	freshCameraInfo = true;
};

void ParticleFilter::trackingTool(const cv::Mat &segmented_left, const cv::Mat &segmented_right) {

    ros::spinOnce();

    if(freshCameraInfo){
	    /**
	     * If necessary to check the model params
	     */
	    // testRenderedModel(initial, segmented_left, segmented_right);

	    std::vector<std::vector<double> > tmp;
	    tmp.resize(2);
	    if (davinci_interface::get_fresh_robot_pos(tmp)) {
	        sensor_1 = tmp[1];
	        sensor_2 = tmp[0];
	    }

	   	Eigen::Matrix<double, 7, 1> temp_sensor_1;
		for (int i = 0; i < 7; ++i)
		{
			temp_sensor_1[i] = sensor_1[i];
		}

	    end_effector = kinematics.fwd_kin_solve(temp_sensor_1);

		Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0],
                                                            sensor_1[0] + DH_q_offset0);
	    Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1],
	                                                            sensor_1[1] + DH_q_offset1);
	    Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2,
	                                                            DH_alpha_params[2], 0.0);
	    Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3], DH_d4, DH_alpha_params[3],
	                                                            sensor_1[3] + DH_q_offset3);

	    Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3 *
	                             a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
	    Eigen::Vector3d a1_trans = a1_pos.translation();

	    cv::Mat a1_rvec = cv::Mat::zeros(3, 1, CV_64FC1);
	    computeRodriguesVec(a1_pos, a1_rvec);

		real_tool.tvec_cyl(0) = a1_trans[0];
		real_tool.tvec_cyl(1) = a1_trans[1];
		real_tool.tvec_cyl(2) = a1_trans[2];
		real_tool.rvec_cyl(0) = a1_rvec.at<double>(0, 0);
		real_tool.rvec_cyl(1) = a1_rvec.at<double>(1, 0);
		real_tool.rvec_cyl(2) = a1_rvec.at<double>(2, 0);

	    double theta_oval = sensor_1[4];
	    double theta_grip = sensor_1[5];
	    double theta_open = sensor_1[6];

	    newToolModel.computeEllipsePose(real_tool, theta_oval, theta_grip, theta_open);


		ROS_INFO("---- in tracking function ---");
		//Update according to the max score
		double maxScore_1 = 0.0;
		int maxScoreIdx_1 = -1; //maximum scored particle index
		double totalScore_1 = 0.0; //total score

		toolImage_left_temp.setTo(0);
		toolImage_right_temp.setTo(0);
	    ////////////////////////PART 1: Measurement model//////////////////////////////
	    for (int i = 0; i < numParticles; ++i) {
	        //get matching score of particle i
			matchingScores_arm_1[i] = measureFuncSameCam(toolImage_left_arm_1,toolImage_right_arm_1,particles_arm_1[i],segmented_left, segmented_right,Cam_left_arm_1,Cam_right_arm_1);

	        //render tool in left and right camera frame on temp image
			newToolModel.renderTool(toolImage_left_temp, particles_arm_1[i], Cam_left_arm_1, P_left);
			newToolModel.renderTool(toolImage_right_temp, particles_arm_1[i], Cam_right_arm_1, P_right);
			// cv::imshow("temp image arm_1 left: " , toolImage_left_temp);
			// cv::imshow("temp image arm_1 right:  " , toolImage_right_temp);
	        //store total match score and largest discovered match score/index
			if (matchingScores_arm_1[i] >= maxScore_1) {
				maxScore_1 = matchingScores_arm_1[i];
				maxScoreIdx_1 = i;
			}
			totalScore_1 += matchingScores_arm_1[i];
		}

	    //show composite of all particles in left/right camera on temp images

		ROS_INFO_STREAM("Maxscore arm 1: " << maxScore_1);  //debug

	    //compute weights as normalized matching scores
		for (int j = 0; j < numParticles; ++j) {
			particleWeights_arm_1[j] = (matchingScores_arm_1[j] / totalScore_1);
			//ROS_INFO_STREAM("weights" << particleWeights[j]);
		}

	    //store the best particle (that with highest weight)
		ToolModel::toolModel best_particle = particles_arm_1[maxScoreIdx_1];
		ROS_INFO("Real tool at (%f %f %f): %f %f %f, ", initial.tvec_cyl(0), initial.tvec_cyl(1),initial.tvec_cyl(2),initial.rvec_cyl(0),initial.rvec_cyl(1), initial.rvec_cyl(2));
		ROS_WARN("Particle ARM AT (%f %f %f): %f %f %f, ",best_particle.tvec_cyl(0), best_particle.tvec_cyl(1),best_particle.tvec_cyl(2),best_particle.rvec_cyl(0),best_particle.rvec_cyl(1), best_particle.rvec_cyl(2));

		///render the best particle on real images and show
		cv::Mat show_raw_left = raw_image_left.clone();
		cv::Mat show_raw_right = raw_image_right.clone();
		newToolModel.renderTool(show_raw_left, best_particle, Cam_left_arm_1, P_left);
		newToolModel.renderTool(show_raw_right, best_particle, Cam_right_arm_1, P_right);

		//////////////////////PART 2: Resampling/////////////////////////////////////
		std::vector<ToolModel::toolModel> oldParticles = particles_arm_1;
		resamplingParticles(oldParticles, particleWeights_arm_1, particles_arm_1);


		// showGazeboToolError(real_tool, best_particle);
		error_vec = endEffectorError(end_effector, real_tool, best_particle);
		// cv::imshow("trackingImages left", show_raw_left);
		// cv::imshow("trackingImages right", show_raw_right);
		// cv::waitKey(20);

		/////////////////////////PART 3: Motion model////////////////////////////////
		updateParticles(particles_arm_1, best_particle);
    }

};

std::vector<double> ParticleFilter::endEffectorError(Eigen::Affine3d& end_effector, ToolModel::toolModel &real_pose, ToolModel::toolModel &bestParticle){

	std::vector<double> error_vec;
	error_vec.resize(2);

	ToolModel::toolModel temp_pose;
	temp_pose.rvec_grip1 = (bestParticle.rvec_grip1 + bestParticle.rvec_grip2); 
	temp_pose.rvec_grip1(0) = temp_pose.rvec_grip1(0) / 2;
	temp_pose.rvec_grip1(1) = temp_pose.rvec_grip1(1) / 2;
	temp_pose.rvec_grip1(2) = temp_pose.rvec_grip1(2) / 2;

	cv::Mat rot_hat(3,3,CV_64FC1);

    cv::Rodrigues(temp_pose.rvec_grip1, rot_hat);
	cv::Mat rvec_hat(3,1,CV_64FC1);
	cv::Rodrigues(rot_hat, rvec_hat);

	temp_pose.tvec_grip1 = bestParticle.tvec_grip1;
    cv::Mat q_end(4, 1, CV_64FC1);
    cv::Mat q_tip =cv::Mat::zeros(4,1,CV_64FC1);
    q_tip.at<double>(1,0)=0.0102;
    q_tip.at<double>(3,0)=1;
    q_end = transformPoints(q_tip, cv::Mat(temp_pose.rvec_grip1),
                             cv::Mat(bestParticle.tvec_grip1));
	ROS_WARN_STREAM("cv::Mat(bestParticle.tvec_grip1) " << cv::Mat(bestParticle.tvec_grip1));
	ROS_WARN_STREAM("cv::Mat(temp_pose.tvec_grip1) " << cv::Mat(temp_pose.rvec_grip1));
    Eigen::Vector3d q_real = end_effector.translation();
    cv::Mat q_end_real =cv::Mat::zeros(3,1,CV_64FC1);
    q_end_real.at<double>(0,0) = q_real[0];
    q_end_real.at<double>(1,0) = q_real[1];
    q_end_real.at<double>(2,0) = q_real[2];
	/*********/
	ToolModel::toolModel temp_real;
	temp_real.rvec_grip1 = (real_pose.rvec_grip1 + real_pose.rvec_grip2); 
	temp_real.rvec_grip1(0) = temp_real.rvec_grip1(0) / 2;
	temp_real.rvec_grip1(1) = temp_real.rvec_grip1(1) / 2;
	temp_real.rvec_grip1(2) = temp_real.rvec_grip1(2) / 2;

	cv::Mat rot_real(3,3,CV_64FC1);
    cv::Rodrigues(temp_real.rvec_grip1, rot_real);
	cv::Mat rvec_real(3,1,CV_64FC1);
	cv::Rodrigues(rot_real, rvec_real);

	// temp_pose.tvec_grip1 = bestParticle.tvec_grip1;
 //    cv::Mat q_end(4, 1, CV_64FC1);
 //    cv::Mat q_tip =cv::Mat::zeros(4,1,CV_64FC1);
 //    q_tip.at<double>(1,0)=0.0102;
 //    q_tip.at<double>(3,0)=1;
 //    q_end = transformPoints(q_tip, cv::Mat(temp_pose.rvec_grip1),
 //                             cv::Mat(bestParticle.tvec_grip1));

// ROS_WARN_STREAM("cv::Mat(bestParticle.rvec_grip1)" << cv::Mat(bestParticle.rvec_grip1));
// ROS_WARN_STREAM("cv::Mat(bestParticle.rvec_grip2)" << cv::Mat(bestParticle.rvec_grip2));

// ROS_WARN_STREAM("cv::Mat(real_tool.rvec_grip1)" << cv::Mat(real_tool.rvec_grip1));
// ROS_WARN_STREAM("cv::Mat(real_tool.rvec_grip2)" << cv::Mat(real_tool.rvec_grip2));

	//Find difference in pos/orientation for real_pose and particle_pose
	cv::Mat position = q_end.rowRange(0,3) - q_end_real;
	cv::Mat orientation = rvec_hat - rvec_real;
	double total_pos = position.dot(position);
	double total_ori = orientation.dot(orientation);
	double error_pos = sqrt(total_pos /3);
	double error_ori = sqrt(total_ori /3);

	error_pos = error_pos * 1000;
	error_ori = error_ori * (180 / M_PI);
	ROS_WARN_STREAM("Position  error in /mm: " << error_pos);
	ROS_WARN_STREAM("orientation  error in /degree: " << error_ori);

	error_vec[0] = error_pos;
	error_vec[1] = error_ori;
	
	return error_vec;

};

std::vector<double> ParticleFilter::showGazeboToolError(ToolModel::toolModel &real_pose, ToolModel::toolModel &bestParticle){

	std::vector<double> error_vec;
	error_vec.resize(2);
	//Get matrix representations of the toolModels
	int dim = 6;
	cv::Mat renderedMat = cv::Mat::zeros(dim,1,CV_64FC1);
	cv::Mat real_tool_vector = cv::Mat::zeros(dim,1,CV_64FC1);
	convertToolModeltoMatrix(bestParticle, renderedMat);
	convertToolModeltoMatrix(real_pose, real_tool_vector);

	//Find difference in pos/orientation for real_pose and particle_pose
	cv::Mat position = real_tool_vector.rowRange(0, 3) - renderedMat.rowRange(0,3);
	cv::Mat orientation = real_tool_vector.rowRange(3, dim) - renderedMat.rowRange(3,dim);
	double total_pos = position.dot(position);
	double total_ori = orientation.dot(orientation);

	double error_pos = sqrt(total_pos /3);
	double error_ori = sqrt(total_ori /3);

	error_pos = error_pos * 1000;
	error_ori = error_ori * (180 / M_PI);
	ROS_WARN_STREAM("Position  error in /mm: " << error_pos);
	ROS_WARN_STREAM("orientation  error in /degree: " << error_ori);

	error_vec[0] = error_pos;
	error_vec[1] = error_ori;

	return error_vec;
};

void ParticleFilter::updateParticles(std::vector<ToolModel::toolModel> &updatedParticles, ToolModel::toolModel &bestParticle) {
    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if (davinci_interface::get_fresh_robot_pos(tmp)) {
        sensor_1 = tmp[1];
        sensor_2 = tmp[0];
    }

 //    /* add noise */
	// sensor_1[1] = sensor_1[1]*1.2;
	// sensor_1[2] = sensor_1[2]*1.03;
	// sensor_1[4] = sensor_1[4]*1.3;
	// sensor_1[5] = sensor_1[5]*1.07;

 //    Eigen::Affine3d a1_pos_1 = kinematics.computeAffineOfDH(DH_a_params[0], DH_d1, DH_alpha_params[0],
 //                                                            sensor_1[0] + DH_q_offset0);
 //    Eigen::Affine3d a1_pos_2 = kinematics.computeAffineOfDH(DH_a_params[1], DH_d2, DH_alpha_params[1],
 //                                                            sensor_1[1] + DH_q_offset1);
 //    Eigen::Affine3d a1_pos_3 = kinematics.computeAffineOfDH(DH_a_params[2], sensor_1[2] + DH_q_offset2,
 //                                                            DH_alpha_params[2], 0.0);
 //    Eigen::Affine3d a1_pos_4 = kinematics.computeAffineOfDH(DH_a_params[3], DH_d4, DH_alpha_params[3],
 //                                                            sensor_1[3] + DH_q_offset3);

 //    Eigen::Affine3d a1_pos = kinematics.affine_frame0_wrt_base_ * a1_pos_1 * a1_pos_2 * a1_pos_3 *
 //                             a1_pos_4;// *a1_5 * a1_6 * a1_7 * kinematics.affine_gripper_wrt_frame6_ ;
 //    Eigen::Vector3d a1_trans = a1_pos.translation();

 //    cv::Mat a1_rvec = cv::Mat::zeros(3, 1, CV_64FC1);
 //    computeRodriguesVec(a1_pos, a1_rvec);

	// ToolModel::toolModel new_pose;
	// new_pose.tvec_cyl(0) = a1_trans[0];
	// new_pose.tvec_cyl(1) = a1_trans[1];
	// new_pose.tvec_cyl(2) = a1_trans[2];
	// new_pose.rvec_cyl(0) = a1_rvec.at<double>(0, 0);
	// new_pose.rvec_cyl(1) = a1_rvec.at<double>(1, 0);
	// new_pose.rvec_cyl(2) = a1_rvec.at<double>(2, 0);

    double theta_caudier = sensor_1[4];
    double theta_grip = sensor_1[5];
    double theta_open = sensor_1[6];

 //    newToolModel.computeEllipsePose(new_pose, theta_caudier, theta_grip, theta_open);


	// cv::Mat diff_1 = cv::Mat(new_pose.tvec_cyl) - cv::Mat(last_tool.tvec_cyl);
	// cv::Mat diff_2 = cv::Mat(new_pose.rvec_cyl) - cv::Mat(last_tool.rvec_cyl);
	// double diff_cly = sqrt(diff_1.dot(diff_1)) + sqrt(diff_2.dot(diff_2));

	// cv::Mat diff_3 = cv::Mat(new_pose.tvec_elp) - cv::Mat(last_tool.tvec_elp);
	// cv::Mat diff_4 = cv::Mat(new_pose.rvec_elp) - cv::Mat(last_tool.rvec_elp);
	// double diff_caudier = sqrt(diff_3.dot(diff_3)) + sqrt(diff_4.dot(diff_4));

	// cv::Mat diff_5 = cv::Mat(new_pose.tvec_grip1) - cv::Mat(last_tool.tvec_grip1);
	// cv::Mat diff_6 = cv::Mat(new_pose.rvec_grip1) - cv::Mat(last_tool.rvec_grip1);

	// cv::Mat diff_7 = cv::Mat(new_pose.tvec_grip2) - cv::Mat(last_tool.tvec_grip2);
	// cv::Mat diff_8 = cv::Mat(new_pose.rvec_grip2) - cv::Mat(last_tool.rvec_grip2);

	// double diff_gripper = sqrt(diff_5.dot(diff_5)) + sqrt(diff_6.dot(diff_6));

	// ROS_INFO_STREAM("downsample_rate_pos " << downsample_rate_pos); //annealing coefficient
	// ROS_INFO_STREAM("downsample_rate_rot " << downsample_rate_rot); //annealing coefficient

	// ROS_INFO_STREAM("new_pose tvec cly" << cv::Mat(new_pose.tvec_cyl));
	//Move particles through gaussian noise motion model
	//after resampling, first particle is best -> save it in the searching pool

	//If error is small decrement it even more

	downsample_rate_pos -= 0.0002;
	if(downsample_rate_pos < 0.0002){
		downsample_rate_pos = 0.0002;
	};

	downsample_rate_rot -= 0.0002;
	if(downsample_rate_rot < 0.0005){
		downsample_rate_rot = 0.0005;
	};


	if(error_vec[0] < 5){
		downsample_rate_pos = 0.0002;
	}
	if(error_vec[1] < 3.0){
		downsample_rate_rot = 0.0005;
	}

	ROS_WARN_STREAM("downsample_rate_pos " << downsample_rate_pos);
	ROS_WARN_STREAM("downsample_rate_rot " << downsample_rate_rot);
	// ROS_WARN_STREAM("diff_caudier " << fabs(diff_caudier));
	// ROS_WARN_STREAM("diff_cly " << fabs(diff_cly));
	// ROS_WARN_STREAM("diff_gripper " << fabs(diff_gripper));
	// if (fabs(diff_cly) >0.01 || fabs(diff_caudier) >0.01 || fabs(diff_gripper) >0.01 ){ROS_WARN_STREAM("UPDATE!");} 

    for (int i = 0; i < numParticles; ++i) {
    	// if (fabs(diff_cly) >0.01 || fabs(diff_caudier) >0.01 || fabs(diff_gripper) >0.01 )
    	// {			
    	// 	// downsample_rate_pos = 0.0006;
    	// 	// downsample_rate_rot = 0.001;
    	//  	updatedParticles[i] = newToolModel.gaussianSampling(new_pose, downsample_rate_pos, downsample_rate_rot, theta_caudier, theta_grip, theta_open );
    	// }else{
    		updatedParticles[i] = newToolModel.gaussianSampling(updatedParticles[i], downsample_rate_pos, downsample_rate_rot, theta_caudier, theta_grip, theta_open);
    	// }
    }

    // last_tool = new_pose;
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

	//the position errors
	toolMatrix.at<double>(0,0) = inputToolModel.tvec_cyl(0);
	toolMatrix.at<double>(1,0) = inputToolModel.tvec_cyl(1);
	toolMatrix.at<double>(2,0) = inputToolModel.tvec_cyl(2);

	//the orientation errors
	toolMatrix.at<double>(3,0) = inputToolModel.rvec_cyl(0);
	toolMatrix.at<double>(4,0) = inputToolModel.rvec_cyl(1);
	toolMatrix.at<double>(5,0) = inputToolModel.rvec_cyl(2);

};

void ParticleFilter::testRenderedModel(ToolModel::toolModel &inputModel, const cv::Mat &segmented_left, const cv::Mat &segmented_right){

    cv::Mat left_test = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::Mat right_test = cv::Mat::zeros(480, 640, CV_8UC3);

    double test_score = measureFuncSameCam(left_test,right_test, inputModel ,segmented_left, segmented_right, Cam_left_arm_1, Cam_right_arm_1);
    ROS_INFO_STREAM("Matching is: " << test_score);

    cv::Mat left_test_show = raw_image_left.clone();
    cv::Mat right_test_show = raw_image_right.clone();

    newToolModel.renderTool(left_test_show, inputModel, Cam_left_arm_1, P_left);
    newToolModel.renderTool(right_test_show, inputModel, Cam_right_arm_1, P_right);

    cv::imshow("left_test ", left_test_show);
    cv::imshow("right_test ", right_test_show);

	endEffectorError(end_effector, real_tool, inputModel);

    cv::waitKey();
};

void ParticleFilter::dataCollection(const cv::Mat &segmented_left, const cv::Mat &segmented_right){
	/**
	 * Noise set 1
	 */
 	ofstream datafile_1 ("/home/ranhao/Desktop/joint_set1.txt", std::ios_base::app);
 	if (datafile_1.is_open())
 	{
 		// initializeParticles(); //every time restart the particles. if in for loop
		datafile_1 << initial_error_vec[0];
		datafile_1 << "  ";
		datafile_1 << initial_error_vec[1];
 		datafile_1 << "  ";
 		for (int k = 0; k < 24; ++k) { 
 			ROS_ERROR_STREAM("POSE new " << k << " th round");  
 			trackingTool(segmented_left, segmented_right);

			datafile_1 << error_vec[0];
			datafile_1 << "  ";
			datafile_1 << error_vec[1];
	 		datafile_1 << "  ";
	 		
 		}
 		datafile_1 << "\n";
		datafile_1.close();
 	}
 	else cout << "Unable to open file";


  //  pos_noise[0] = 0.005;
	 // pos_noise[1] = 0.008;
	 // pos_noise[2] = 0.0;

	 // rot_noise[0] = 0.05;
	 // rot_noise[1] = 0.07;
	 // rot_noise[2] = 0.06;

	 // pos_thresh = 4;
	 // numParticles = 700;

	 // for (int j = 0; j < 50; ++j) {  //each pose have 50 data sets to collect
	 // 	ofstream datafile_3("/home/ranhao/Desktop/new_raw_pose.txt", std::ios_base::app);
	 // 	if (datafile_3.is_open()) {

	 // 		ROS_ERROR_STREAM("POSE new " << j << " th round");
	 // 		downsample_rate_pos = 0.004;
	 // 		downsample_rate_rot = 0.005;
	 // 		initializeParticles();

	 // 		for (int k = 0; k < 15; ++k) {   //10 iterations
	 // 			trackingTool(segmented_left, segmented_right);

  //               datafile_3 << error_pos;
  //               datafile_3 << "  ";
  //               datafile_3 << error_ori;
  //               datafile_3 << "  ";
	 // 		}


	 // 		datafile_3 << "\n";
	 // 		datafile_3.close();
	 // 	}
	 // 	else cout << "Unable to open pose 3 file";
	 // }

	 ROS_INFO("FINISHED !");
};