#ifndef PARTICLEFILTER_H
#define PARTICLEFILTER_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
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

 class ParticleFilter{

 private:

 	cv::Mat Cam;

 	ToolModel newToolModel;  /// it should be set up the first time, probably need updates of the camera poses

 	ToolModel::toolModel initial; //initial configuration

    int toolSize; //size of the needle to be rendered

    double perturbStd; //standard deviation for perturbing, if we obtain good matching it gets smaller to stay in the region

 	int numParticles; //total number of particles
    cv::Mat toolImage_left; //needleImage
    cv::Mat toolImage_right; //needleImage
    cv::Mat segmentedImage; //segmented needle image (retrieved from vessellness)
    cv::Rect ROI_left; //ROI for the left image containing needle geometry
    cv::Rect ROI_right; //ROI for the right image containing needle geometry

 	std::vector<ToolModel::toolModel> particles; // particles
 //	std::vector<needle_detection::needleGeometry> oldParticles; //this is used while resampling
 	std::vector<double> mathcingScores; // particle scores (matching scores)
 	std::vector<double> particleWeights; // particle weights calculated from matching scores


 public:

     /*
     * The default constructor 
     */
    ParticleFilter();

    /*
     * The deconstructor 
     */
    ~ParticleFilter();

     /*
     * The initializeParticles initializes the particles by setting the total number of particles, initial
     * guess and randomly generating the particles around the initial guess.
     */
    void initializeParticles();

 //    /*
 //    * This is the main function for tracking the needle. This function is called and it syncs all of the functions
 //    */
 //    std::vector<cv::Mat> trackTool(const cv::Mat &bodyVel,const cv::Mat &segmented_left, const cv::Mat &segmented_right,const cv::Mat &P_left, const cv::Mat &P_right);
	// /*
	// * resampling method
	// */
 //    void resampleLowVariance(const std::vector<ToolModel::toolModel> &initial, const std::vector<double> &particleWeight,  std::vector<ToolModel::toolModel> &results);
	// /*
	// * perturb the paritlces for more usable poses
	// */
 //    std::vector<ToolModel::toolModel> perturb(const std::vector<ToolModel::toolModel> &particles, double stdev, double mean);





};

#endif