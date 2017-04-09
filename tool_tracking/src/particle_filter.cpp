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
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

ParticleFilter::ParticleFilter(ros::NodeHandle *nodehandle) :
        node_handle(*nodehandle), numParticles(50), Downsample_rate(0.02), toolSize(2){

	/****need to subscribe this***/
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


	Cam_left = Cam_left_arm_1.clone();  ///should be camera extrinsic parameter relative to the tools
	Cam_right = Cam_right_arm_1.clone();

	initializeParticles();

	// initialization, just basic black image ??? how to get the size of the image
	toolImage_left = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right = cv::Mat::zeros(480, 640, CV_8UC3);

	toolImage_left_temp = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right_temp = cv::Mat::zeros(480, 640, CV_8UC3);


};

ParticleFilter::~ParticleFilter() {

};

void ParticleFilter::initializeParticles() {
	ROS_INFO("---- Initialize particle is called---");
	particles.resize(numParticles); //initialize particle array
	matchingScores.resize(numParticles); //initialize matching score array
	particleWeights.resize(numParticles); //initialize particle weight array


    /******Find and convert our various params and inputs******/
    //Get sensor update.
    kinematics = Davinci_fwd_solver();

    //Pull in our first round of sensor data.
    davinci_interface::init_joint_feedback(node_handle);

    std::vector<std::vector<double> > tmp;
    tmp.resize(2);
    if(davinci_interface::get_fresh_robot_pos(tmp)){
        sensor_1 = tmp[0];
        sensor_2 = tmp[1];
    }

    Eigen::Affine3d a1_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_1.data()));

    Eigen::Vector3d a1_trans = a1_pos.translation();
    cv::Mat a1_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a1_pos, a1_rvec);


    Eigen::Affine3d a2_pos = kinematics.fwd_kin_solve(Vectorq7x1(sensor_2.data()));
    Eigen::Vector3d a2_trans = a2_pos.translation();
    cv::Mat a2_rvec = cv::Mat::zeros(3,1,CV_64FC1);
    computeRodriguesVec(a2_pos, a2_rvec);

	initial.tvec_grip2(0) = a1_trans[0];  //left and right (image frame)
	initial.tvec_grip2(1) = a1_trans[1];  //up and down
	initial.tvec_grip2(2) = a1_trans[2];
	initial.rvec_grip2(0) = a1_rvec.at<double>(0,0);
	initial.rvec_grip2(1) = a1_rvec.at<double>(1,0);
	initial.rvec_grip2(2) = a1_rvec.at<double>(2,0);

    double theta_cylinder = tmp[0][4]; //initial guess
    double theta_oval = tmp[0][5]; //initial guess
    double theta_open = tmp[0][6]; //initial guess

	for (int i = 0; i < numParticles; i++) {
		particles[i] = newToolModel.setRandomConfig(initial, theta_cylinder, theta_oval, theta_open );
	}

};


std::vector<cv::Mat>
ParticleFilter::trackingTool(const cv::Mat &segmented_left, const cv::Mat &segmented_right,
							 const cv::Mat &P_left, const cv::Mat &P_right) {

    ros::spinOnce();

    // ROS_INFO("---- Inside tracking function ---");
	std::vector<cv::Mat> trackingImages;
	trackingImages.resize(2);

	double maxScore = 0.0; //track the maximum scored particle
    int maxScoreIdx = -1; //maximum scored particle index
	/***Update according to the max score***/

	while (maxScore < 1) {

        maxScoreIdx = -1; //maximum scored particle index
        maxScore = 0.0;
        double totalScore = 0.0; //total score

		cv::Mat segmentedImage_left = segmented_left.clone();
		cv::Mat segmentedImage_right = segmented_right.clone();

		toolImage_left_temp.setTo(0);
		toolImage_right_temp.setTo(0);

		/***do the sampling and get the matching score***/
		for (int i = 0; i < particles.size(); ++i) {

			toolImage_left.setTo(0); //reset image for every start of an new loop
			newToolModel.renderTool(toolImage_left, particles[i], Cam_left, P_left); //first get the rendered image using 3d model of the tool

			float left = newToolModel.calculateMatchingScore(toolImage_left, segmented_left);  //get the matching score

			toolImage_right.setTo(0); //reset image
			newToolModel.renderTool(toolImage_right, particles[i], Cam_right, P_right);
			float right = newToolModel.calculateMatchingScore(toolImage_right, segmented_right);

			/***testing***/
			newToolModel.renderTool(toolImage_left_temp, particles[i], Cam_left, P_left);
			newToolModel.renderTool(toolImage_right_temp, particles[i], Cam_right, P_right);

			matchingScores[i] = sqrt(pow(left, 2) + pow(right, 2));

			/////what if just for the left tool, TEST
			//matchingScores[i] = left;

			if (matchingScores[i] >= maxScore) {
				maxScore = matchingScores[i];
				maxScoreIdx = i;
			}
			totalScore += matchingScores[i];
		}

		cv::imshow("temp left", toolImage_left_temp);

		ROS_INFO_STREAM("Maxscore: " << maxScore);  //debug

		/*** calculate weights using matching score and do the resampling ***/
		for (int j = 0; j < particles.size(); ++j) { // normalize the weights
			particleWeights[j] = (matchingScores[j] / totalScore);
			//ROS_INFO_STREAM("weights" << particleWeights[j]);
		}

		//render in segmented image, no need to get the ROI

		newToolModel.renderTool(segmentedImage_left, particles[maxScoreIdx], Cam_left, P_left);
		//newToolModel.renderTool(segmentedImage_right, particles[maxScoreIdx], Cam_right, P_right);

        ToolModel::toolModel best_particle = particles[maxScoreIdx];
//        toolImage_left.setTo(0);
//        newToolModel.renderTool(toolImage_left, best_particle, Cam_left, P_left);
//        cv::imshow("best particle", toolImage_left);
//		ROS_INFO_STREAM("Best particle tvec(0)" << particles[maxScoreIdx].tvec_elp(0) );
//		ROS_INFO_STREAM("Best particle tvec(1)" << particles[maxScoreIdx].tvec_elp(1) );
//		ROS_INFO_STREAM("Best particle tvec(2)" << particles[maxScoreIdx].tvec_elp(2) );
//		ROS_INFO_STREAM("Best particle rvec(0)" << particles[maxScoreIdx].rvec_elp(0) );
//		ROS_INFO_STREAM("Best particle rvec(1)" << particles[maxScoreIdx].rvec_elp(1) );
//		ROS_INFO_STREAM("Best particle rvec(2)" << particles[maxScoreIdx].rvec_elp(2) );
//
//        ROS_INFO_STREAM("Best particle tvec(0)" << particles[maxScoreIdx].tvec_cyl(0) );
//        ROS_INFO_STREAM("Best particle tvec(1)" << particles[maxScoreIdx].tvec_cyl(1) );
//        ROS_INFO_STREAM("Best particle tvec(2)" << particles[maxScoreIdx].tvec_cyl(2) );
//        ROS_INFO_STREAM("Best particle rvec(0)" << particles[maxScoreIdx].rvec_cyl(0) );
//        ROS_INFO_STREAM("Best particle rvec(1)" << particles[maxScoreIdx].rvec_cyl(1) );
//        ROS_INFO_STREAM("Best particle rvec(2)" << particles[maxScoreIdx].rvec_cyl(2) );

		trackingImages[0] = segmentedImage_left.clone();


		ROS_INFO_STREAM("new particles.SIZE" << particles.size());
		//each time will clear the particles and resample them
		std::vector<ToolModel::toolModel> oldParticles = particles;
        //resample using low variance resampling method
		resamplingParticles(oldParticles, particleWeights, particles);
        //std::vector<ToolModel::toolModel> updatedParticles = particles;
        newToolModel.renderTool(segmentedImage_left, best_particle, Cam_left, P_left);
        trackingImages[1] = segmentedImage_left.clone();

        updateParticles(particles, best_particle);

		//cv::imshow("temp right", toolImage_right_temp);
		cv::imshow("trackingImages left", trackingImages[0]);
		//cv::imshow("best particle 2 in same loop",trackingImages[1]);
		cv::waitKey(10);

	}

	/*** UPDATE particles, based on the given body vel and updating rate ***/
	// double dT = 0.02; //sampling rate
//	updateSamples(bodyVel, dT);

	return trackingImages;
};

/***** update particles to find and reach to the best pose ***/
void ParticleFilter::updateParticles(std::vector<ToolModel::toolModel> &updatedParticles,
								   const ToolModel::toolModel &bestParticle) {

	double sampleStep = 0.00001;

	Downsample_rate -= sampleStep;

	updatedParticles.clear();
    updatedParticles.resize(numParticles);
    ///every loop should generate different particle from one base particle k
	updatedParticles[0] = bestParticle;
    for (int i = 1; i < numParticles; ++i) {
        updatedParticles[i] = newToolModel.gaussianSampling(bestParticle, Downsample_rate);

    }

};

double ParticleFilter::measureFuncSameCam(cv::Mat & toolImage_cam, ToolModel::toolModel &toolPose_left, ToolModel::toolModel &toolPose_right,
										  const cv::Mat &segmented_cam, const cv::Mat & Projection_mat, cv::Mat &raw_img, cv::Mat &Cam_matrix_tool_left, cv::Mat &Cam_matrix_tool_right) {

	toolImage_cam.setTo(0);
	/***do the sampling and get the matching score***/
	//first get the rendered image using 3d model of the tool

	newToolModel.renderTool(toolImage_cam, toolPose_left, Cam_matrix_tool_left, Projection_mat);
	newToolModel.renderTool(toolImage_cam, toolPose_right, Cam_matrix_tool_right, Projection_mat);

	newToolModel.renderTool(raw_img, toolPose_left, Cam_matrix_tool_left, Projection_mat);
	newToolModel.renderTool(raw_img, toolPose_right, Cam_matrix_tool_right, Projection_mat);

	double matchingScore = newToolModel.calculateMatchingScore(toolImage_cam, segmented_cam);

	return matchingScore;
};

/*** one possible motion model for tool in image frame ***/
void ParticleFilter::updateSamples(const cv::Mat &bodyVel,
									 double &updateRate) { //get particles and update them based on given spatial velocity

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
		newToolModel.computeModelPose(particles[k], 0.0, 0.0, 0.0); // no need to change relative angles??? TODO:

	}
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