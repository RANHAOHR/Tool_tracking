/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016 Case Western Reserve University
 *
 *    Ran Hao <rxh349@case.edu>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve Univeristy, nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
        nh_(*nodehandle), numParticles(300), toolSize(2), perturbStd(0.001) {
    /****initial position guess
	everything here is in meters*****/
    initial.tvec_elp(0) = 0.0;
    initial.tvec_elp(1) = 0.0;
    initial.tvec_elp(2) = 0.0;
    initial.rvec_elp(0) = 0.0;
    initial.rvec_elp(1) = 0.0;   //better make everything zero
    initial.rvec_elp(2) = 0.0;

    /****need to subscribe this***/
    Cam = cv::Mat(4, 4, CV_64FC1);  ///should be camera extrinsic parameter relative to the tools
    Cam.at<double>(0, 0) = 1;
    Cam.at<double>(1, 0) = 0;
    Cam.at<double>(2, 0) = 0;
    Cam.at<double>(3, 0) = 0;

    Cam.at<double>(0, 1) = 0;
    Cam.at<double>(1, 1) = -1;
    Cam.at<double>(2, 1) = 0;
    Cam.at<double>(3, 1) = 0;

    Cam.at<double>(0, 2) = 0;
    Cam.at<double>(1, 2) = 0;
    Cam.at<double>(2, 2) = -1;
    Cam.at<double>(3, 2) = 0;

    Cam.at<double>(0, 3) = 0.0;   //should be in meters
    Cam.at<double>(1, 3) = 0.1;
    Cam.at<double>(2, 3) = 0.2;  // cannot have z = 0 for reprojection, camera_z must be always point to object
    Cam.at<double>(3, 3) = 1;

    //initialize particles by randomly assigning around the initial guess
    initializeParticles();

    // initialization, just basic black image ??? how to get the size of the image
    toolImage_left = cv::Mat::zeros(480, 640, CV_8UC3);
    toolImage_right = cv::Mat::zeros(480, 640, CV_8UC3);

};

ParticleFilter::~ParticleFilter() {

};

void ParticleFilter::initializeParticles() {
    ROS_INFO("---- Initialize particle is called---");
    particles.resize(numParticles); //initialize particle array
    matchingScores.resize(numParticles); //initialize matching score array
    particleWeights.resize(numParticles); //initialize particle weight array
    //generate random needle
    for (int i(0); i < numParticles; i++) {
        particles[i] = newToolModel.setRandomConfig(initial, Cam, 1, 0);
    }

};

std::vector<cv::Mat>
ParticleFilter::trackingTool(const cv::Mat &bodyVel, const cv::Mat &segmented_left, const cv::Mat &segmented_right,
                             const cv::Mat &P_left, const cv::Mat &P_right) {

    // ROS_INFO("---- Inside tracking function ---");
    cv::Mat segmentedImage_left = segmented_left.clone();
    cv::Mat segmentedImage_right = segmented_right.clone();

    std::vector<cv::Mat> trackingImages;
    trackingImages.resize(2);

    double maxScore = -1.0; //track the maximum scored particle
    int maxScoreIdx = -1; //maximum scored particle index
    double totalScore = 0.0; //total score


    /***do the sampling and get the matching score***/
    for (int i = 0; i < numParticles; ++i) {

        toolImage_left.setTo(0); //reset image for every start of an new loop
        newToolModel.renderTool(toolImage_left, particles[i], Cam,
                                           P_left); //first get the rendered image using 3d model of the tool
        double left = newToolModel.calculateMatchingScore(toolImage_left, segmented_left);  //get the matching score

        toolImage_right.setTo(0); //reset image
        newToolModel.renderTool(toolImage_right, particles[i], Cam, P_right);
        double right = newToolModel.calculateMatchingScore(toolImage_right, segmented_right);

        matchingScores[i] = sqrt(pow(left, 2) + pow(right, 2));

        if (matchingScores[i] > maxScore) {
            maxScore = matchingScores[i];
            maxScoreIdx = i;
        }
        totalScore += matchingScores[i];

    }

//    cv::imshow("current left: ", toolImage_left);
//    cv::imshow("current right: ", toolImage_right);

    /*** you may wanna do this in a different stream, TODO: ***/
    ROS_INFO_STREAM("Maxscore: " << maxScore);  //debug

    newToolModel.renderTool(segmentedImage_left, particles[maxScoreIdx], Cam,
                            P_left);  //render in segmented image, no need to get the ROI
    newToolModel.renderTool(segmentedImage_right, particles[maxScoreIdx], Cam, P_right);

    trackingImages[0] = segmentedImage_left;
    trackingImages[1] = segmentedImage_right;

    /*** calculate weights using matching score and do the resampling ***/
    for (int j = 0; j < numParticles; ++j) { // normalize the weights
        particleWeights[j] = (matchingScores[j] / totalScore);
    }

    //resample using low variance resampling method
    std::vector<ToolModel::toolModel> oldParticles = particles;
    //each time will clear the particles and resample them
    resampleLowVariance(oldParticles, particleWeights, particles);

    /*TODO:testing*/
/*    for (int k = 0; k < numParticles; ++k) {
        ROS_INFO("-----");
        ROS_INFO_STREAM("oldParticles:" << oldParticles[k].tvec_elp);
        ROS_INFO_STREAM("particles:" << particles[k].tvec_elp);
    }*/
    double dT = 0.02; //sampling rate

    /*** UPDATE particles, based on the given body vel and updating rate ***/
//    updateParticles(bodyVel, dT);
//
    //if there is no movement, we need to perturb it
    if (bodyVel.at<double>(0, 0) == 0.0 && bodyVel.at<double>(1, 0) == 0.0 && bodyVel.at<double>(5, 0) == 0.0) {
        //TODO: look for matching score, if it is good assign smaller perturbation standard deviation
        if (maxScore > 0.91) {
            perturbStd = 0.0;
        }

        for (int i(0); i < particles.size(); i++) {
            particles[i] = newToolModel.setRandomConfig(particles[i], Cam, perturbStd, 0.0);
        }
    }

    return trackingImages;
};

/***** update particles to find and reach to the best pose ***/
void ParticleFilter::updateParticles(const cv::Mat &bodyVel,
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
void ParticleFilter::resampleLowVariance(const std::vector<ToolModel::toolModel> &sampleModel,
                                         const std::vector<double> &particleWeight,
                                         std::vector<ToolModel::toolModel> &update_particles) {
    int M = sampleModel.size(); //total number of particles
    double max = 1.0 / M;

    update_particles.clear();
    update_particles.resize(sampleModel.size());
    double r = newToolModel.randomNum(0.0, max);

    double w = particleWeight[0]; //first particle weight

    int idx = 0;
    int new_idx = 0;
    for (int i = 0; i < M; ++i) {
        double U = r + ((double) (i - 1) * max);

        while (U > w){
            idx += 1;
            w = w + particleWeight[idx];
        }
        update_particles[new_idx] = sampleModel[idx];
        new_idx += 1;
    }

};

cv::Mat ParticleFilter::addNoise(cv::Mat &inputMat)   //add noise for the spatial velocity
{
    cv::Mat noiseAddedSrc = cv::Mat::zeros(6, 1, CV_64F);

    for (int i(0); i < 6; i++) {
        double num = inputMat.at<double>(i, 0);
        num = num * 0.001;
        double rnum = newToolModel.randomNumber(fabs(num), 0.0);
        noiseAddedSrc.at<double>(i, 0) = inputMat.at<double>(i, 0) + rnum;
    }

    return noiseAddedSrc;
}
