/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *	 Ran Hao <rxh349@case.edu>
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
 *   * Neither the name of Case Western Reserve University, nor the names of its
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
 */

#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <string>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tool_model_lib/tool_model.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>

#include <cwru_davinci_interface/davinci_interface.h>
#include <cwru_davinci_kinematics/davinci_kinematics.h>

#include <iostream>
#include <fstream>

//#include <cwru_xform_utils/xform_utils.h>  //for jade (surgical31)
#include <xform_utils/xform_utils.h>  //for indigo

class ParticleFilter {

private:

	ros::NodeHandle node_handle;

    /**
     * The ToolModel class.
     */
	ToolModel newToolModel; 

    /**
     * The initial tool pose, obtained from the forward kinematics, a toolModel= 3x1 cv::Mats tvec_cyl, rvec_cyl, tvec_elp, rvec_elp, tvec_grip1(2), rvec_grip1(2)
     */
	ToolModel::toolModel initial; 

    /**
     * The total number of particles
     */
	unsigned int numParticles; 

    /**
     * Left and right images in left camera frames. Will store each particle vs segmented image to calculate chamfer distance
     */
	cv::Mat toolImage_left_arm_1;
	cv::Mat toolImage_right_arm_1; 

    /**
     * Composite of all rendered particles in left and right camera. This is for debug and showing purpose
     */
	cv::Mat toolImage_left_temp; 
	cv::Mat toolImage_right_temp; 

    /**
     * The particle scores (matching scores), store all the matching scores computed from the chamfer matching algorithm
     */
	std::vector<double> matchingScores_arm_1;

	/**
     * The particles in the form of tool poses
     */
	std::vector<ToolModel::toolModel> particles_arm_1;

    /**
     * particle weights calculated from matching scores
     */
	std::vector<double> particleWeights_arm_1;  

    /**
     * Intermediary representations of Gcb, this is because the FK is using the form of Eigen::Affine3d. Will be converted to cv::mat for image processing purpose.
     */
	Eigen::Affine3d arm_1__cam_l;
	Eigen::Affine3d arm_2__cam_l;
	Eigen::Affine3d arm_1__cam_r;
	Eigen::Affine3d arm_2__cam_r;

    /**
     * All the camera-robot base transformations, computed ffrom the Eigen::Affine3d  matrices (e.g. arm_1__cam_l). Storing the matrices for arm 2 for later use
     */
	cv::Mat Cam_left_arm_1; 
	cv::Mat Cam_right_arm_1;
	cv::Mat Cam_left_arm_2;
	cv::Mat Cam_right_arm_2;

    /**
     * The left and right camera projection matrices
     */
    cv::Mat P_left;
    cv::Mat P_right;

    /**
     * FK for davinci class
     */
	Davinci_fwd_solver kinematics;

    /**
     * Sensor readings from davinci of joint angles 7x1, old version. sensor_1 is arm 1 and sensor_2 is arm 2.
     */
	std::vector<double> sensor_1; 
    std::vector<double> sensor_2; 

    /**
     * The subscriber of the projection matrices for left and right camera. This is subscribing to camera_info.
     */
	ros::Subscriber projectionMat_subscriber_r; 
	ros::Subscriber projectionMat_subscriber_l;

    /**
     * Determine if the projection matices are received
     */
	bool freshCameraInfo; 

    /**
     * Annealing rate for position and orientation noise.  High at start to allow large perturbations and decrease as we converge.
     */
	double downsample_rate_pos;
	double downsample_rate_rot; 

    /**
     * Errors between best particle and gazebo readings for pos and orientation
     */
	double error_pos;
	double error_ori; 

    /**
     * The threshold for local search, when the error of pos is smal and the perturbation is not necessary
     */ 
	double pos_thresh;

    /**
     * The given system position and orientation noises for testing purpose
     */
	std::vector<double> pos_noise;
	std::vector<double> rot_noise;

    /**
     * @brief get P_right from subsriber
     * @param projectionRight
     */
    void projectionRightCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight);

    /**
     * @brief get P_left from subscriber
     * @param projectionLeft
     */
    void projectionLeftCB(const sensor_msgs::CameraInfo::ConstPtr &projectionLeft);

public:

    /**
     * The left and right raw images from image pipeline
     */
	cv::Mat raw_image_left; 
	cv::Mat raw_image_right; 

	/**
	* @brief - The default constructor
	* Call initializeParticles()
	* Compute Gcb as a cv::mat
	* Initialize empty tool images
	*/
	ParticleFilter(ros::NodeHandle *nodehandle);

	/**
	 * @brief- The deconstructor
	 */
	~ParticleFilter();

	/**
	 * @brief- initializing the particles with the following actions:
	 * 		Initialize matchingScores_arm_x, particles_arm_x and particle_weights_arm_x arrays
	 * 		Compute an initial guess and generate particles around it with getCoarseGuess()
	 */
	void initializeParticles();

	/**
     * @brief- Computing an initial guess from FK and generate particles around it
     * Used when particles initialized and there is no prior particle data
     * First get sensor info from all 7 joints
     * Then compute tvec_cyl/rvec_cyl from FK of joints 1-4
     * Then generate particles randomly about tvec_cyl/rvec_cyl/joints 5,6,7
     * Then update toolModel to include joints 5,6,7
     */
    void getCoarseGuess();

	/**
	 * @brief - Main tracking function
	 * called by tracking_particle.cpp
	 * compute matching score of each particle
	 * show composite of all particles and composite of best particle and segmented image
	 * resample particles based on weights
	 * apply gaussian noise motion model
	 * @return perturbed particles after resampling and motion model
	*/
	void trackingTool(const cv::Mat &segmented_left, const cv::Mat &segmented_right);

	/**
	 * @brief Resampling old particles with replacement
	 * Used in resampling step of trackingTool
	 * @param sampleModel old particles to draw from
	 * @param particleWeight particle weights
	 * @param update_particles new collection of weighted particles
	 */
	void resamplingParticles(const std::vector<ToolModel::toolModel> &sampleModel,
							 const std::vector<double> &particleWeight,
							 std::vector<ToolModel::toolModel> &update_particles);

	/**
	 * @brief - Computing the matching score between a particle and a segmented image
	 * Render the particle and segmented image together and compute the chamfer distance between them
	 * Used in measurement model
	 * @param toolImage_left image of left segmented image and particle transformed under left camera's frame
	 * @param toolImage_right image of right segmented image and particle transformed under right camera's frame
	 * @param toolPose toolModel representation of a particle (to compare with segmented images)
	 * @param segmented_left segmented image from left camera (to compare with particle)
	 * @param segmented_right segmented image from right camera (to compare with particle)
	 * @param Cam_left left camera transform used to transform particle
	 * @param Cam_right right camera transform used to transform particle
	 * @return chamfer score between particle and segmented images
	 */
	double measureFuncSameCam(cv::Mat & toolImage_left, cv::Mat & toolImage_right, ToolModel::toolModel &toolPose,
							  const cv::Mat &segmented_left, const cv::Mat &segmented_right, cv::Mat &Cam_left, cv::Mat &Cam_right);

	/**
	 * @brief - Modifying annealing coefficient and call motion model gaussianSampling()
	 * Used after resampling before motion model
	 * @param updateParticles particles to go through motion model
	 */
	void updateParticles(std::vector<ToolModel::toolModel> &updateParticles);

	/**
 	* @brief - Computing the error between the real pose from Gazebo and the best particles guess of the pose
	* Used in simulation to determine accuracy of particle
 	* @param real_pose pose from Gazebo
 	* @param bestParticle
 	*/
	void showGazeboToolError(ToolModel::toolModel &real_pose, ToolModel::toolModel &bestParticle);

	/**
	 * @brief - Converting a toolModel representation to a 6x1 matrix representation.
	 * Used to find the error in simulation (showGazeboToolError)
	 * @param inputToolModel
	 * @param toolMatrix output matrix representation
	 */
	void convertToolModeltoMatrix(const ToolModel::toolModel &inputToolModel, cv::Mat &toolMatrix);

	/**
 	* @brief - Converting a rotation matrix to a rotation vector.
	* Used in forward kinematics of getCoraseGuess()
 	* @param trans - input SE(3)
 	* @param rot_vec output 3x1 cv::mat
 	*/
	void computeRodriguesVec(const Eigen::Affine3d & trans, cv::Mat rot_vec);

	/**
	 * @brief - Converting from an SE(3) representation to a cv::Mat representation.
	 * Used to get a matrix representation of Gcb in the constructor
	 * @param trans
	 * @param outputMatrix output cv::Mat
	 */
	void convertEigenToMat(const Eigen::Affine3d & trans, cv::Mat & outputMatrix);

    /**
     * @brief - Testing the model params based on the rendered image and the matching score.
     * Used in the trackingTool function
     * @param inputModel
     * @param segmented_left
     * @param segmented_right
     */
    void testRenderedModel(ToolModel::toolModel &inputModel, cv::Mat &segmented_left, cv::Mat &segmented_right);

    /**
     * @brief - Collecting data sets, for 4 pre-defined poses, 50 data sets for each pose. SHould specify the path of the file first.
     * @param segmented_left
     * @param segmented_right
     */
	void dataCollection(const cv::Mat &segmented_left, const cv::Mat &segmented_right);
};

#endif
