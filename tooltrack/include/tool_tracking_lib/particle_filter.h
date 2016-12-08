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

#include <cwru_needle_tracking/needle_model.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
 #include <cv_bridge/cv_bridge.h>

 class ParticleFilter{
 private:
 	needle_detection::needleGeometry initial; //initial needle guess

    int needleSize; //size of the needle to be rendered

    double perturbStd; //standard deviation for perturbing, if we obtain good matching it gets smaller to stay in the region

 	int numParticles; //total number of particles
    cv::Mat needleImage_left; //needleImage
    cv::Mat needleImage_right; //needleImage
    cv::Mat segmentedImage; //segmented needle image (retrieved from vessellness)
    cv::Rect ROI_left; //ROI for the left image containing needle geometry
    cv::Rect ROI_right; //ROI for the right image containing needle geometry

 	std::vector<needle_detection::needleGeometry> particles; // particles
 //	std::vector<needle_detection::needleGeometry> oldParticles; //this is used while resampling
 	std::vector<double> mathcingScores; // particle scores (matching scores)
 	std::vector<double> particleWeights; // particle weights calculated from matching scores


 public:
 	 /*
     * This constructor uses an input charactor to set who it subscribes to
     * It also then initializes the publisher.
     */
    ParticleFilter(const char*,const char*);

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

    /*
    * Calculates particle weights from matching score
    */
    std::vector<double> calculateWeights(std::vector<double> &);

    /*
    * This is the main function for tracking the needle. This function is called and it syncs all of the functions
    */
    std::vector<cv::Mat> trackNeedle(const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Mat &, const cv::Mat &);



};

#endif