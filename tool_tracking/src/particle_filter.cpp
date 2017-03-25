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
		nh_(*nodehandle), numParticles(100), Downsample_rate(0.02), toolSize(2), L(7) {
	/****initial position guess
	everything here is in meters*****/
	initial.tvec_elp(0) = 0.0;
	initial.tvec_elp(1) = 0.0;
	initial.tvec_elp(2) = 0.0;
	initial.rvec_elp(0) = 0.0;
	initial.rvec_elp(1) = 0.0;   //better make everything zero
	initial.rvec_elp(2) = 0.0;

	/****need to subscribe this***/
	Cam_left = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,   ///meters or millimeters
			0, -1, 0, 0,
			0, 0, -1, 0.2,
			0, 0, 0, 1);  ///should be camera extrinsic parameter relative to the tools

	Cam_right = (cv::Mat_<double>(4, 4) << 1, 0, 0, -0.005,   ///meters or millimeters
			0, -1, 0, 0,
			0, 0, -1, 0.2,
			0, 0, 0, 1);

	initializeParticles();

	// initialization, just basic black image ??? how to get the size of the image
	toolImage_left = cv::Mat::zeros(475, 640, CV_8UC3);
	toolImage_right = cv::Mat::zeros(475, 640, CV_8UC3);

	toolImage_left_temp = cv::Mat::zeros(475, 640, CV_8UC3);
	toolImage_right_temp = cv::Mat::zeros(475, 640, CV_8UC3);

	/***motion model params***/

	com_s1 = nh_.subscribe("/dvrk/PSM1/set_position_joint", 10, &ParticleFilter::newCommandCallback1, this);
	com_s2 = nh_.subscribe("/dvrk/PSM2/set_position_joint", 10, &ParticleFilter::newCommandCallback2, this);

	cmd_green.resize(L);
	cmd_yellow.resize(L);
};

ParticleFilter::~ParticleFilter() {

};

void ParticleFilter::initializeParticles() {
	ROS_INFO("---- Initialize particle is called---");
	particles.resize(numParticles); //initialize particle array
	matchingScores.resize(numParticles); //initialize matching score array
	particleWeights.resize(numParticles); //initialize particle weight array

	///generate random seeds
	initial.tvec_elp(0) = 0.0;  //left and right (image frame)
	initial.tvec_elp(1) = 0.0;  //up and down
	initial.tvec_elp(2) = -0.03;
	initial.rvec_elp(0) = 0.0;
	initial.rvec_elp(1) = 0.0;
	initial.rvec_elp(2) = -1;

    double theta = 0.1;

	for (int i = 0; i < numParticles; i++) {
		particles[i] = newToolModel.setRandomConfig(initial, theta);
	}

};

void ParticleFilter::newCommandCallback1(const sensor_msgs::JointState::ConstPtr& incoming){
	std::vector<double> positions = incoming->position;
	for(int i = 0; i < L; i++){
		cmd_green[i] = positions[i];
	}
};

void ParticleFilter::newCommandCallback2(const sensor_msgs::JointState::ConstPtr& incoming){
	std::vector<double> positions = incoming->position;
	for(int j = 0; j < L; j++){
		cmd_yellow[j] = positions[j];
	}
};

std::vector<cv::Mat>
ParticleFilter::trackingTool(const cv::Mat &bodyVel, const cv::Mat &segmented_left, const cv::Mat &segmented_right,
							 const cv::Mat &P_left, const cv::Mat &P_right) {

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

//            ROS_INFO_STREAM(" particle tvec(0)" << i << " " <<particles[i].tvec_cyl(0) );
//            ROS_INFO_STREAM(" particle tvec(1)" << i << " " <<particles[i].tvec_cyl(1) );
//            ROS_INFO_STREAM(" particle tvec(2)" << i << " " <<particles[i].tvec_cyl(2) );
//            ROS_INFO_STREAM(" particle rvec(0)" << i << " " <<particles[i].rvec_cyl(0) );
//            ROS_INFO_STREAM(" particle rvec(1)" << i << " " <<particles[i].rvec_cyl(1) );
//            ROS_INFO_STREAM(" particle rvec(2)" << i << " " <<particles[i].rvec_cyl(2) );

			toolImage_left.setTo(0); //reset image for every start of an new loop
			newToolModel.renderTool(toolImage_left, particles[i], Cam_left,
									P_left); //first get the rendered image using 3d model of the tool

			float left = newToolModel.calculateChamferScore(toolImage_left, segmented_left);  //get the matching score

			toolImage_right.setTo(0); //reset image
			newToolModel.renderTool(toolImage_right, particles[i], Cam_right, P_right);
			float right = newToolModel.calculateMatchingScore(toolImage_right, segmented_right);

			/***testing***/
			newToolModel.renderTool(toolImage_left_temp, particles[i], Cam_left, P_left);
			newToolModel.renderTool(toolImage_right_temp, particles[i], Cam_right, P_right);

			//matchingScores[i] = sqrt(pow(left, 2) + pow(right, 2));

			/////what if just for the left tool
			matchingScores[i] = left;
//            ROS_INFO_STREAM("matchingScores" << matchingScores[i]);
//            ROS_INFO("--------NEW ONE----------");

			if (matchingScores[i] >= maxScore) {
				maxScore = matchingScores[i];
				maxScoreIdx = i;
			}
			totalScore += matchingScores[i];
		}

		cv::imshow("temp left", toolImage_left_temp);

		ROS_INFO_STREAM("Maxscore: " << maxScore);  //debug
//        ROS_INFO_STREAM("totalScore: " << totalScore);  //debug
//        ROS_INFO_STREAM("maxScoreIdx: " << maxScoreIdx);

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
		cv::waitKey(20);

	}

	/*** UPDATE particles, based on the given body vel and updating rate ***/
	// double dT = 0.02; //sampling rate
//	updateSamples(bodyVel, dT);

	return trackingImages;
};

/***** update particles to find and reach to the best pose ***/
void ParticleFilter::updateParticles(std::vector<ToolModel::toolModel> &updatedParticles,
								   const ToolModel::toolModel &bestParticle) {

//	ROS_INFO_STREAM("assume best tvec(0)" << bestParticle.tvec_elp(0) );
//	ROS_INFO_STREAM("assume best tvec(1)" << bestParticle.tvec_elp(1) );
//	ROS_INFO_STREAM("assume best tvec(2)" << bestParticle.tvec_elp(2) );
//	ROS_INFO_STREAM("assume best rvec(0)" << bestParticle.rvec_elp(0) );
//	ROS_INFO_STREAM("assume best rvec(1)" << bestParticle.rvec_elp(1) );
//	ROS_INFO_STREAM("assume best rvec(2)" << bestParticle.rvec_elp(2) );

	int sampleStep = 0.001;

	Downsample_rate -= sampleStep;

	updatedParticles.clear();
    updatedParticles.resize(numParticles);
    ///every loop should generate different particle from one base particle k
    for (int i = 0; i < numParticles; ++i) {
        updatedParticles[i] = newToolModel.gaussianSampling(bestParticle, 0.1);

    }

};

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

///TODO: for tracking of the Motion model
void ParticleFilter::UnscentedKalmanFilter(const cv::Mat &mu, const cv::Mat &sigma, cv::Mat &update_mu, cv::Mat &update_sigma, cv::Mat &zt, cv::Mat &ut){

	//L is the dimension of the joint space for single arm
	double alpha = 0.005;
	double k = 0.0; //TODO: how much?
	double beta = 2;

	double lamda = alpha * alpha * (L + k) - L;

	double gamma = L + lamda;
	gamma = pow(gamma, 0.5);

	///get the square root for sigma
	cv::Mat square_sigma = cv::Mat::zeros(L, 1, CV_64FC1);

	cv::Mat s = cv::Mat(L, 1, CV_64FC1);  //need the square root for sigma
	cv::Mat vt = cv::Mat(L, L, CV_64FC1);  //need the square root for sigma
	cv::Mat u = cv::Mat(L, L, CV_64FC1);  //need the square root for sigma

	cv::SVD::compute(sigma, s, u, vt);  //s is supposed to be the one we are asking for, the sigular values

	square_sigma = s.clone(); //safe way to pass values to a cv Mat

	std::vector<cv::Mat> state_vecs;
	state_vecs.resize(2*L); ///size is 2L

	state_vecs[0] = mu.clone();   //X_nod
	for (int i = 1; i < L; ++i) {
		state_vecs[i] = state_vecs[0] + gamma * square_sigma;
		state_vecs[i + L] = state_vecs[0] - gamma * square_sigma;
	}

	double weight_mean = lamda / (L + lamda);
	double weight_covariance = weight_mean + 1-alpha * alpha + beta;

	std::vector<double> weight_vec_c;
	std::vector<double> weight_vec_m;
	weight_vec_c.resize(2*L);
	weight_vec_m.resize(2*L);

	weight_vec_c[0] = weight_mean;
	weight_vec_m[0] = weight_covariance;

	for (int l = 1; l < 2*L; ++l) {
		weight_vec_c[l] = 1/(2 * (L + lamda ));
		weight_vec_m[l] = 1/(2 * (L + lamda ));
	}

	/***get the prediction***/
	cv::Mat current_mu = cv::Mat::zeros(L,1,CV_64FC1);
	cv::Mat current_sigma = cv::Mat::zeros(L,L, CV_64FC1);

	std::vector<cv::Mat> currentState_vec;
	currentState_vec.resize(2*L);

	//TODO: missing function to get update state space vector?

	cv::Mat R = cv::Mat::eye(L,L,CV_64FC1);
	R = R * 0.037;

	for (int m = 0; m < 2*L; ++m) {
		cv::Mat temp = weight_vec_m[m] * currentState_vec[m];
		current_mu = current_mu + temp;
	}

	for (int n = 0; n < 2*L; ++n) {
		cv::Mat var_mat = currentState_vec[n] - current_mu;

		cv::Mat temp_mat = weight_vec_c[n] * var_mat * var_mat.t();
		current_sigma = current_sigma + temp_mat;
	}

	current_sigma = current_sigma + R;

	/*****get measurement****/
	std::vector<cv::Mat> updateState_vec;
	updateState_vec.resize(2*L);

	//compute new square root for current sigma
	cv::SVD::compute(current_sigma, s, u, vt);  //s is supposed to be the one we are asking for, the sigular values

	square_sigma = s.clone(); //safe way to pass values to a cv Mat

	updateState_vec[0] = current_mu.clone();   //X_nod
	for (int i = 1; i < L; ++i) {
		updateState_vec[i] = updateState_vec[0] + gamma * square_sigma;
		updateState_vec[i + L] = updateState_vec[0] - gamma * square_sigma;
	}

	std::vector<cv::Mat> current_z_vec;
	current_z_vec.resize(2*L);

	//TODO: get measurement function

	cv::Mat weighted_z = cv::Mat::zeros(2*L,1,CV_64FC1);

	for (int j = 0; j < 2*L; ++j) {
		cv::Mat temp = weight_vec_m[j] * current_z_vec[j];
		weighted_z = weighted_z + temp;
	}

	cv::Mat S_t = cv::Mat::zeros(L, L, CV_64FC1);

	for (int i1 = 0; i1 < 2*L; ++i1) {
		cv::Mat var_z = current_z_vec[i1] - weighted_z;

		cv::Mat temp_mat = weight_vec_c[i1] * var_z * var_z.t();
		S_t = S_t + temp_mat;
	}

	cv::Mat Q = cv::Mat::eye(L,L,CV_64FC1);
	Q = Q * 0.00038;

	S_t = S_t + Q;
	////get cross-covariance
	cv::Mat omega_x_z  = cv::Mat::zeros(2*L, 1, CV_64FC1);

	for (int k1 = 0; k1 < 2*L; ++k1) {
		cv::Mat var_x = updateState_vec[k1] - current_mu;

		cv::Mat var_z = current_z_vec[k1] - weighted_z;

		cv::Mat temp_mat = weight_vec_c[k1] * var_x * var_z.t();

		omega_x_z = omega_x_z + temp_mat;
	}

	///get Kalman factor
	cv::Mat K_t = cv::Mat::zeros(L,L,CV_64FC1);
	K_t = omega_x_z * S_t.inv();

	//update mu and sigma
	update_mu = cv::Mat::zeros(L,1,CV_64FC1);  ////just in case the dimension is not match
	update_sigma = cv::Mat::zeros(L,L,CV_64FC1);

	update_mu = current_mu + K_t * (zt - weighted_z);
	update_sigma = current_sigma - K_t * S_t * K_t.t();

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
