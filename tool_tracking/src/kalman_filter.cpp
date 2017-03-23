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
#include <opencv2/calib3d/calib3d.hpp>
#include <cwru_davinci_interface/davinci_interface.h>

using namespace std;

KalmanFilter::KalmanFilter(ros::NodeHandle *nodehandle) :
		nh_(*nodehandle), numParticles(100), Downsample_rate(0.02), toolSize(2), L(7) {
		
	ROS_INFO("Initializing UKF...");

	/****need to subscribe this***/
	Cam_left = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,   ///meters or millimeters
			0, -1, 0, 0,
			0, 0, -1, 0.2,
			0, 0, 0, 1);  ///should be camera extrinsic parameter relative to the tools

	Cam_right = (cv::Mat_<double>(4, 4) << 1, 0, 0, -0.005,   ///meters or millimeters
			0, -1, 0, 0,
			0, 0, -1, 0.2,
			0, 0, 0, 1);

	//initializeParticles(); Where we're going, we don't need particles.

	// initialization, just basic black image ??? how to get the size of the image
	toolImage_left = cv::Mat::zeros(475, 640, CV_8UC3);
	toolImage_right = cv::Mat::zeros(475, 640, CV_8UC3);

	toolImage_left_temp = cv::Mat::zeros(475, 640, CV_8UC3);
	toolImage_right_temp = cv::Mat::zeros(475, 640, CV_8UC3);

	/***motion model params***/
	com_s1 = nh_.subscribe("/dvrk/PSM1/set_position_joint", 10, &KalmanFilter::newCommandCallback1, this);
	com_s2 = nh_.subscribe("/dvrk/PSM2/set_position_joint", 10, &KalmanFilter::newCommandCallback2, this);

	cmd_green.resize(L);
	cmd_yellow.resize(L);
	
	davinci_interface::init_joint_feedback(nh_);
	std::vector<std::vector<double> > tmp;
	if(davinci_interface::get_fresh_robot_pos(tmp)){
		sensor_green = tmp[0];
		sensor_yellow = tmp[1];
	}
	
	kalman_mu = cv::Mat::zeros(14, 1, CV_32F);
	kalman_sigma = (cv::Mat::eye(14, 14, CV_32F) * 0.037);
};

KalmanFilter::~KalmanFilter() {

};

void KalmanFilter::newCommandCallback1(const sensor_msgs::JointState::ConstPtr& incoming){
	std::vector<double> positions = incoming->position;
	for(int i = 0; i < L; i++){
		cmd_green[i] = positions[i];
	}
};

void KalmanFilter::newCommandCallback2(const sensor_msgs::JointState::ConstPtr& incoming){
	std::vector<double> positions = incoming->position;
	for(int j = 0; j < L; j++){
		cmd_yellow[j] = positions[j];
	}
};

std::vector<cv::Mat> KalmanFilter::trackingTool(
	//Got rid of bodyVel as velocity data will now be coming from internal sources. Or something.
	const cv::Mat &segmented_left,
	const cv::Mat &segmented_right,
	const cv::Mat &P_left,
	const cv::Mat &P_right
) {
					 
	//Looks mostly IP-related; need to resturcture to not be dependant on particles and instead use sigma-points.		

	std::vector<cv::Mat> trackingImages;
	trackingImages.resize(2);
	
	//TODO Sigma points come out as angle vectors. Convert them to limb positions
	
	//Choose the best sigma point from this batch to run IP on.
	//TODO: Convert from particles to sigma points- what will actually need to be changed?
	double maxScore = 0.0;//track the maximum scored sigma point
	int maxScoreIdx = -1;//maximum scored SP index
	double totalScore = 0.0;//total score
	ToolModel::toolModel best_particle;

		cv::Mat segmentedImage_left = segmented_left.clone();
		cv::Mat segmentedImage_right = segmented_right.clone();

		toolImage_left_temp.setTo(0);
		toolImage_right_temp.setTo(0);

		/***do the sampling and get the matching score***/
		for (int i = 0; i < numParticles; ++i) {

			toolImage_left.setTo(0); //reset image for every start of an new loop
			newToolModel.renderTool(toolImage_left, particles[i], Cam_left,
									P_left); //first get the rendered image using 3d model of the tool
			double left = newToolModel.calculateMatchingScore(toolImage_left, segmented_left);  //get the matching score

			toolImage_right.setTo(0); //reset image
			newToolModel.renderTool(toolImage_right, particles[i], Cam_right, P_right);
			double right = newToolModel.calculateMatchingScore(toolImage_right, segmented_right);

			/***testing***/
			newToolModel.renderTool(toolImage_left_temp, particles[i], Cam_left, P_left);
			newToolModel.renderTool(toolImage_right_temp, particles[i], Cam_right, P_right);

			//matchingScores[i] = sqrt(pow(left, 2) + pow(right, 2));

			/////what if just for the left tool
			matchingScores[i] = left;

			if (matchingScores[i] > maxScore) {
				maxScore = matchingScores[i];
				maxScoreIdx = i;

				best_particle = particles[i];
			}
			totalScore += matchingScores[i];
		}

		cv::imshow("temp left", toolImage_left_temp);

		newToolModel.renderTool(segmentedImage_left, particles[maxScoreIdx], Cam_left, P_left);
		newToolModel.renderTool(segmentedImage_right, particles[maxScoreIdx], Cam_right, P_right);

		trackingImages[0] = segmentedImage_left;
		trackingImages[1] = segmentedImage_right;
		
		//cv::imshow("temp right", toolImage_right_temp);

		cv::imshow("trackingImages left", trackingImages[0]);
		//cv::imshow("trackingImages right",trackingImages[1]);
		cv::waitKey(30);

	return trackingImages;
};

cv::Mat KalmanFilter::adjoint(cv::Mat &G) {
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

void KalmanFilter::update(){
	ROS_INFO("UPDATING KALMAN FILTER");
	
	//Get sensor update.
	std::vector<std::vector<double> > tmp;
	if(davinci_interface::get_fresh_robot_pos(tmp)){
		sensor_green = tmp[0];
		sensor_yellow = tmp[1];
	}
	
	//TODO: Convert to cart space using forward kinematics.
	
	//TODO: Figure out how to handle desired positions with respect to our model.
	
	//TODO: Actually update the filter.
}

///TODO: for tracking of the Motion model
//In the new paradigm, this will probably need to be split into two functions/steps. One that computes the sigma points, and one that takes those points and the image data and computes error.
//For our immediate purposes, a magical function that updates a mu and sigma. He kills aliens and doesn't afraid of anything.
void KalmanFilter::UnscentedKalmanFilter(const cv::Mat &mu, const cv::Mat &sigma, cv::Mat &update_mu, cv::Mat &update_sigma, const cv::Mat &zt, const cv::Mat &ut){

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
