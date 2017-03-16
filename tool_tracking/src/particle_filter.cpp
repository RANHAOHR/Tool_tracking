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
        nh_(*nodehandle), numParticles(1000), Downsample_rate(0.02), toolSize(2){
    /****initial position guess
	everything here is in meters*****/
    initial.tvec_elp(0) = 0.0;
    initial.tvec_elp(1) = 0.0;
    initial.tvec_elp(2) = 0.0;
    initial.rvec_elp(0) = 0.0;
    initial.rvec_elp(1) = 0.0;   //better make everything zero
    initial.rvec_elp(2) = 0.0;

    /****need to subscribe this***/
    Cam_left = (cv::Mat_<double>(4,4) << 1, 0, 0, 0,   ///meters or millimeters
            0, -1, 0, 0,
            0, 0, -1, 0.2,
            0, 0, 0, 1);  ///should be camera extrinsic parameter relative to the tools

    Cam_right = (cv::Mat_<double>(4,4) << 1, 0, 0, -0.005,   ///meters or millimeters
            0, -1, 0, 0,
            0, 0, -1, 0.2,
            0, 0, 0, 1);

    initializeParticles();

    // initialization, just basic black image ??? how to get the size of the image
    toolImage_left = cv::Mat::zeros(475, 640, CV_8UC3);
    toolImage_right = cv::Mat::zeros(475, 640, CV_8UC3);

    toolImage_left_temp = cv::Mat::zeros(475, 640, CV_8UC3);
    toolImage_right_temp = cv::Mat::zeros(475, 640, CV_8UC3);

};

ParticleFilter::~ParticleFilter() {

};

void ParticleFilter::initializeParticles() {
    ROS_INFO("---- Initialize particle is called---");
    particles.resize(numParticles); //initialize particle array
    matchingScores.resize(numParticles); //initialize matching score array
    particleWeights.resize(numParticles); //initialize particle weight array

    int batch_1 = 500;
    int batch_2 = 1000;

    ///generate random seeds
    initial.tvec_elp(0) = 0.0;  //left and right (image frame)
    initial.tvec_elp(1) = 0.0;  //up and down
    initial.tvec_elp(2) = 0.03;
    initial.rvec_elp(0) = 0.0;
    initial.rvec_elp(1) = 0.0;
    initial.rvec_elp(2) = -2;

    for (int i = 0; i < batch_1; i++) {
        particles[i] = newToolModel.setRandomConfig(initial);
    }

    initial.tvec_elp(0) = 0.0;  //left and right (image frame)
    initial.tvec_elp(1) = 0.0;  //up and down
    initial.tvec_elp(2) = -0.03;
    initial.rvec_elp(0) = 0.0;
    initial.rvec_elp(1) = 0.0;
    initial.rvec_elp(2) = -1;


    for (int i = batch_1; i < batch_2; i++) {
        particles[i] = newToolModel.setRandomConfig(initial);
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
    double totalScore = 0.0; //total score

    /***Update according to the max score***/
    ToolModel::toolModel best_particle;
    while(maxScore> -1){

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

        ROS_INFO_STREAM("Maxscore: " << maxScore);  //debug

        /*** calculate weights using matching score and do the resampling ***/
        for (int j = 0; j < numParticles; ++j) { // normalize the weights
            particleWeights[j] = (matchingScores[j] / totalScore);
            //ROS_INFO_STREAM("weights" << particleWeights[j]);
        }

        std::vector<double> tempWeights = particleWeights;
        //render in segmented image, no need to get the ROI

        newToolModel.renderTool(segmentedImage_left, particles[maxScoreIdx], Cam_left, P_left);
        newToolModel.renderTool(segmentedImage_right, particles[maxScoreIdx], Cam_right, P_right);

        trackingImages[0] = segmentedImage_left;
        trackingImages[1] = segmentedImage_right;

        //resample using low variance resampling method
        std::vector<ToolModel::toolModel> updateParticles;
        std::vector<double> update_weights;

        sort(tempWeights.begin(), tempWeights.end());

        for (int k = 0; k <numParticles; ++k) {

            if(particleWeights[k] >= tempWeights[numParticles - 1] ){
                updateParticles.push_back(particles[k]); //correspondingly
                update_weights.push_back(particleWeights[k]);
            }

        }

        updateSamples(updateParticles, update_weights, particles, best_particle);
        ROS_INFO_STREAM("new particles.SIZE" << particles.size());
        //each time will clear the particles and resample them
        //resamplingParticles(oldParticles, particleWeights, particles);


        //cv::imshow("temp right", toolImage_right_temp);

        cv::imshow("trackingImages left",trackingImages[0]);
        //cv::imshow("trackingImages right",trackingImages[1]);
        cv::waitKey(30);

    }

    /*** UPDATE particles, based on the given body vel and updating rate ***/
    // double dT = 0.02; //sampling rate
//    updateParticles(bodyVel, dT);

    return trackingImages;
};

/***** update particles to find and reach to the best pose ***/
void ParticleFilter::updateSamples(std::vector<ToolModel::toolModel> &oldParticles, std::vector<double> &update_weights,
                                     std::vector<ToolModel::toolModel> &updateParticles,
                                   ToolModel::toolModel &bestParticle){

    int sampleSize = numParticles;
    int sampleStep = 0.001;

    Downsample_rate -= sampleStep;
    ROS_INFO_STREAM("Downsample_rate: " << Downsample_rate);

    double total = 0.0;
    std::vector<int> newSamples;

    newSamples.resize(update_weights.size());

    for (int i = 0; i < update_weights.size() ; ++i) {
        total += update_weights[i];
    }

    //normalized weights
    for (int j = 0; j < update_weights.size() ; ++j) {
        newSamples[j] = (int)(sampleSize * update_weights[j]/total);
        //ROS_INFO_STREAM("newSamples j: " << newSamples[j]);  //debug
    }

    updateParticles.clear();
    for (int k = 0; k <newSamples.size() ; ++k) {
        ///every loop should generate different particle from one base particle k
        for (int i = 0; i <newSamples[k] ; ++i) {
            updateParticles.push_back(newToolModel.gaussianSampling(bestParticle, Downsample_rate));

        }
    }

};

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

/**** resampling method: Low Variance****/
void ParticleFilter::resamplingParticles(const std::vector<ToolModel::toolModel> &sampleModel,
                                         const std::vector<double> &particleWeight,
                                         std::vector<ToolModel::toolModel> &update_particles,
                                         std::vector<double> &update_weights) {

    int M = sampleModel.size(); //total number of particles
    double max = 1.0 / M;

    double r = newToolModel.randomNum(0.0, max);
    double w = particleWeight[0]; //first particle weight
    int idx = 0;

    update_particles.clear(); ///start fresh
    update_weights.clear();

    for (int i = 0; i < M; ++i) {

        double U = r + ((double) (i - 1) * max);

        while (U > w){
            idx += 1;
            w = w + particleWeight[idx];
        }

        update_particles.push_back(sampleModel[idx]);
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
};
