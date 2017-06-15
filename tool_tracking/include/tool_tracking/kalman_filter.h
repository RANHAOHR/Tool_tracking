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

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <string>
#include <cstring>

#include <tool_model_lib/tool_model.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <vesselness_image_filter_cpu/vesselness_lib.h>
#include <boost/random/normal_distribution.hpp>

#include <geometry_msgs/Transform.h>
#include <tf/transform_listener.h>
#include <cwru_davinci_interface/davinci_interface.h>
#include <cwru_davinci_kinematics/davinci_kinematics.h>
#include <sensor_msgs/image_encodings.h>
#include <cwru_opencv_common/projective_geometry.h>

/**
 * @brief cwru_xform_utils is for running in the Jade version
 */
//#include <cwru_xform_utils/xform_utils.h>

/**
 * @brief xform_utils is for running in the Indigo version
 */
#include <xform_utils/xform_utils.h>

class KalmanFilter {

private:
    ros::NodeHandle nh_;

/**
 * @brief instantiate the ukf tool geometry
 */
    ToolModel ukfToolModel;

/**
 * @brief initial tool pose
 */
    ToolModel::toolModel initial;

/**
 * @brief image for showing the arm_1 on the left and right camera
 */
    cv::Mat toolImage_left_arm_1;
    cv::Mat toolImage_right_arm_1;

/**
 * @brief image for showing the arm_2 on the left and right camera
 */
    cv::Mat toolImage_left_arm_2;
    cv::Mat toolImage_right_arm_2;

/**
 * @brief camera extrinsic matrix for arm_1 to left and right camera
 */
	cv::Mat Cam_left_arm_1;
    cv::Mat Cam_right_arm_1;

/**
 * @brief camera extrinsic matrix for arm_2 to left and right camera
 */
    cv::Mat Cam_left_arm_2;
    cv::Mat Cam_right_arm_2;

/**
 * @brief  a image for showing some test results
 */
    cv::Mat resulting_image;

/**
 * @brief Degree of freedom for single tool pose
 */
    int L;

/**
 * @brief dimension for measurement vector, get this from getMeasurementModel
 */
	int measurement_dimension;

/**
 * time step for time t and t+ delta_t
 */
	double t_1_step;
	double t_step;
/**
 * @brief Unscented Kalman filter parameters
 */
    const static double alpha = 0.001;
    const static double k = 0.0;
    const static double beta = 2;

/**
 * @brief joint sensor feedback, gives 7 joint angles. sensor_1 for green arm, sensor_2 for yellow arm
 */
    std::vector<double> sensor_1;
    std::vector<double> sensor_2;

/**
 * @brief the mean and covariance for arm 1
 */
    cv::Mat kalman_mu_arm1;
    cv::Mat kalman_sigma_arm1;

/**
 * @brief computed from the forward kinematics, this is only useful for gazebo and evaluating
 */
	cv::Mat real_mu;

/**
 * @brief the mean and covariance for arm 2
 */
	cv::Mat kalman_mu_arm2;
	cv::Mat kalman_sigma_arm2;

/**
 * @brief instantiate a fwd_solver, find the definition in pkg: cwru_davinci_kinematics, davinci_kinematics.cpp
 */
    Davinci_fwd_solver kinematics;

/**
 * @brief camera extrinsic matrix, getting from tf::listener, for computing from tf to Eigen matrix in Constructor
 */
    Eigen::Affine3d arm_1__cam_l;
    Eigen::Affine3d arm_2__cam_l;
    Eigen::Affine3d arm_1__cam_r;
    Eigen::Affine3d arm_2__cam_r;

    ros::Subscriber projectionMat_subscriber_r;
    ros::Subscriber projectionMat_subscriber_l;

    void projectionRightCB(const sensor_msgs::CameraInfo::ConstPtr &projectionRight);
    void projectionLeftCB(const sensor_msgs::CameraInfo::ConstPtr &projectionLeft);

/**
 * @brief left and right projection matrix
 */
    cv::Mat P_left;
    cv::Mat P_right;

/**
 * @brief motion model
 * @param sigma_point_out:  output sigma point
 * @param sigma_point_in:  input sigma point
 */
	void g(cv::Mat & sigma_point_out, const cv::Mat & sigma_point_in, const cv::Mat & u_t);

/**
 * @brief predicted observation model
 * @param sigma_point_out
 * @param sigma_point_in
 * @param left_image
 * @param right_image
 * @param cam_left: left camera extrinsic matrix
 * @param cam_right
 * @param normal_measurement:  input normals for computing the predicted observation vector, this is obtain via getMeasurementModel function
 */
	void h(cv::Mat & sigma_point_out, const cv::Mat_<double> & sigma_point_in,
			cv::Mat &left_image,cv::Mat &right_image,
			cv::Mat &cam_left, cv::Mat &cam_right, cv::Mat &normal_measurement);

	/**
	 * @brief compute joint velocities for motion model
	 */
	void computeJointVelocity(cv::Mat & u_t);

/**
 * @brief image subscribing part
 */
	cv::Mat seg_left;
	cv::Mat seg_right;

/**
 * @brief using Canny edge detector for segmentation
 * @param InputImg
 * @return
 */
	cv::Mat segmentation(cv::Mat &InputImg);

	bool freshSegImage;
	bool freshCameraInfo;

public:

/**
 * @brief left and right camera raw image
 */
	cv::Mat tool_rawImg_left;
	cv::Mat tool_rawImg_right;

/**
 * @brief The default constructor
 */
    KalmanFilter(ros::NodeHandle *nodehandle);

/**
 * @brief The deconstructor
 */
    ~KalmanFilter();

/**
 * @brief showing the affine matrix
 * @param affine
 */
    void print_affine(Eigen::Affine3d &affine);

/**
 * @brief test the obtained normals for measurement model
 * @param temp_point: input image points
 * @param temp_normal: input image point normals
 * @param inputImage
 */
    void showNormals(cv::Mat &temp_point, cv::Mat &temp_normal, cv::Mat &inputImage );

/**
 * @brief double arm tracking function
 */
	void UKF_double_arm();

/**
 * @brief update mean and covariance, main UKF structure
 * @param kalman_mu: mean vector
 * @param kalman_sigma: covariance matrix
 * @param left_image
 * @param right_image
 */
    void update(cv::Mat & kalman_mu, cv::Mat & kalman_sigma,
				cv::Mat &left_image,cv::Mat &right_image, const cv::Mat & u_t);

/**
 * @brief convert the Kalman mu or sigma points to a tool model
 * @param trans: this is the input matrix, can be a kalman_mu or a sigma point
 * @param toolModel: return the tool geometry
 */
	void convertToolModel(const cv::Mat & trans, ToolModel::toolModel &toolModel);

/**
 *
 * @param arm_pose - input vector containing tool pose, left and right camera matrices <19,1>
 * @param toolModel : return a tool geometry
 * @param cam_mat_l : left camera to base transformation
 * @param cam_mat_r : right camera to base transformation
 */
	void computeToolPose(const cv::Mat & arm_pose, ToolModel::toolModel &toolModel, cv::Mat & cam_mat_l, cv::Mat & cam_mat_r);


/**
 * @brief get a coarse guess using forward kinematics
 */
    void getCoarseEstimation();

/**
 * @brief get measurement model using only one camera feedback, usually left camera
 * @param coarse_guess_vector: input the coarse guess
 * @param segmentation_img: segmented image
 * @param zt: output observation vector
 * @param normal_measurement: output normals for computing the predicted observation vector
 */
	void getMeasurementModel(const ToolModel::toolModel &coarse_guess_model, const cv::Mat &segmentation_img, const cv::Mat &projection_mat, cv::Mat &Cam_matrix, cv::Mat & rawImage, cv::Mat &zt, cv::Mat &normal_measurement);

/**
 * @brief get the measurement model using stereo vision
 * @param coarse_guess_vector: input the coarse guess
 * @param zt: output observation vector
 * @param normal_measurement: output normals for computing the predicted observation vector
 */
	void getStereoMeasurement(const cv::Mat & coarse_guess_vector, cv::Mat &zt, cv::Mat &normal_measurement, cv::Mat & cam_left, cv::Mat & cam_right);

/**
 * @brief convert a affine matrix to opencv matrix
 * @param arm_pose: the SE(3) group matrix in Eigen::Affine
 * @param outputMatrix: the SE(3) group matrix in opencv matrix
 */
    void convertEigenToMat(const Eigen::Affine3d & arm_pose, cv::Mat & outputMatrix);

/**
 * @brief compute a rodrigues vector from a affine matrix
 * @param arm_pose: the SE(3) group matrix with format of Eigen::Affine
 * @param rot_vec: the 3x1 rotation vector with format of opencv matrix
 */
	void computeRodriguesVec(const Eigen::Affine3d & arm_pose, cv::Mat & rot_vec);

/**
 * @brief Cholesky decomposition for square root of covariance
 * @param A: the input matrix
 * @param S: output the decomposed matrix
 */
	void Cholesky( const cv::Mat& A, cv::Mat& S );

/**
 * @brief
 * @param real_pose: the tool pose obtained from forward kinematics in Gazebo
 * @param KalmanMu:
 */
	void showGazeboToolError(cv::Mat &real_pose, cv::Mat &KalmanMu);

/**
 * @brief show the rendered tool image using input tool pose
 * @param inputToolPose: can be kalman mu or "real tool" in Gazebo
 */
	void showRenderedImage(cv::Mat &inputToolPose);

};
#endif
