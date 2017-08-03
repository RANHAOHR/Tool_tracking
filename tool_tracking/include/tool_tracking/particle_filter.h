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

#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <stdlib.h>
#include <math.h>

#include <string>
#include <cstring>

#include <tool_model_lib/tool_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <vesselness_image_filter_cpu/vesselness_lib.h>
#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Transform.h>
#include <cwru_davinci_interface/davinci_interface.h>

#include <tf/transform_listener.h>
#include <cwru_davinci_interface/davinci_interface.h>
#include <cwru_davinci_kinematics/davinci_kinematics.h>

#include <cwru_xform_utils/xform_utils.h>
//#include <xform_utils/xform_utils.h>

class ParticleFilter {

private:

    ros::NodeHandle node_handle;

    ToolModel newToolModel;
/**
 * @brief predicted_real_pose is used to get the predicted real tool pose from using the forward kinematics
 */
    ToolModel::toolModel next_step_pose;

    unsigned int numParticles; //total number of particles

/**
 * @brief left and right rendered Images for ARM 1, used for calculating matching score
 */
    cv::Mat toolImage_left_arm_1;
    cv::Mat toolImage_right_arm_1;

/**
 * @brief left and right rendered Images for showing all particles
 */
    cv::Mat toolImage_left_temp;
    cv::Mat toolImage_right_temp;

    std::vector<double> matchingScores_arm_1; // particle scores (matching scores)

    std::vector<std::vector<double> > particles_arm_1; // particles
    std::vector<double> particleWeights_arm_1; // particle weights calculated from matching scores

/**
 * @brief The transformation between the left and right camera matrices
 */
    cv::Mat g_cr_cl;

/**
 * @brief The camera to robot base transformation.
 * The left one is the standard, the right right one is computed using the g_cr_cl according to left one
 */
    cv::Mat Cam_left_arm_1;
    cv::Mat Cam_right_arm_1;

    Davinci_fwd_solver kinematics;

/**
 * @brief sensor information computed by the joint sensor of the the robot
 */
    std::vector<double> sensor_1;
    std::vector<double> sensor_2;

    void projectionRightCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight);

    void projectionLeftCB(const sensor_msgs::CameraInfo::ConstPtr &projectionLeft);

    ros::Subscriber projectionMat_subscriber_r;
    ros::Subscriber projectionMat_subscriber_l;

/**
 * @brief flag for getting new camera info
 */
    bool freshCameraInfo;

/**
 * @brief The time stamp to track the velocity for motion model
 */
    double t_step;
    double t_1_step;

/**
 * @brief The noises for perturbation
 */
    double down_sample_joint;
    double down_sample_cam;

/**
 * @brief The dimension of the state vector
 */
    int L;

public:

/**
 * @brief The code for pictures
 */
     int index_pic;

/**
 * @brief for comparing and testing
 */
    cv::Mat raw_image_left; //left rendered Image
    cv::Mat raw_image_right; //right rendered Image

/**
 * @brief The projection matrices
 */
    cv::Mat P_left;
    cv::Mat P_right;

/**
* @brief The default constructor
*/
    ParticleFilter(ros::NodeHandle *nodehandle);

/**
 * @brief The deconstructor
 */
    ~ParticleFilter();

/**
 * @brief The initializeParticles initializes the particles by setting the total number of particles, initial
 * guess and randomly generating the particles around the initial guess.
 */
    void initializeParticles();

/**
 * @brief get a coarse initialzation using forward kinematics
 */
    void getCoarseGuess();

/**
 * @brief Main tracking function
 * @param segmented_left : segmented image for left camera
 * @param segmented_right : segmented image for right camera
 * @return
 */
    void trackingTool(const cv::Mat &segmented_left, const cv::Mat &segmented_right);

/**
 * @brief low variance resampling
 * @param sampleModel : input particles
 * @param particleWeight : input normalized weights
 * @param update_particles : output particles
 */
    void resamplingParticles(const std::vector<std::vector<double> > &sampleModel,
                             const std::vector<double> &particleWeight,
                             std::vector<std::vector<double> > &update_particles);

/**
 * @brief get the p(z_t|x_t), compute the matching score based on the camera view image and rendered image
 * @param toolImage_left
 * @param toolImage_right
 * @param toolPose
 * @param segmented_left
 * @param segmented_right
 * @param Cam_left
 * @param Cam_right
 * @return matching score using matching functions: chamfer matching
 */
    double measureFuncSameCam(cv::Mat &toolImage_left, cv::Mat &toolImage_right, ToolModel::toolModel &toolPose,
                              const cv::Mat &segmented_left, const cv::Mat &segmented_right, cv::Mat &Cam_left,
                              cv::Mat &Cam_right);

/**
 * @brief Motion model, propagte the particles using velocity computed from joint sensors
 * @param best_particle_last: last time step best particle, used to compute the nominal velocity
 * @param updatedParticles : input and output particles
 */
    void updateParticles(std::vector<double> &best_particle_last, double &maxScore,
                         std::vector<std::vector<double> > &updatedParticles,
                         ToolModel::toolModel &predicted_real_pose);

/**
 * @brief Getting the particles by addding Gaussain noise to the initialization
 * @param inputParticle
 * @param noisedParticles
 */
    void computeNoisedParticles(std::vector<double> &inputParticle, std::vector<std::vector<double> > &noisedParticles);

/**
 * @brief Extract the Rodrigues vector (the rotation part) given an Eigen::Affine3d
 * @param trans
 * @param rot_vec
 */
    void computeRodriguesVec(const Eigen::Affine3d &trans, cv::Mat &rot_vec);

/**
 * @brief Compute the SE(3) Matrix when given an Eigen::Affine3d
 * @param trans
 * @param outputMatrix
 */
    void convertEigenToMat(const Eigen::Affine3d &trans, cv::Mat &outputMatrix);
};

#endif
