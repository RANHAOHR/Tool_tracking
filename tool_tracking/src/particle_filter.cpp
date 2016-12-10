 
 #include <ros/ros.h>
 #include <tool_tracking/particle_filter.h>
<<<<<<< HEAD
=======
 //#include <tool_model_lib/tool_model.h>
#include <toolModel/tool_model.h>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/highgui/highgui.hpp>
>>>>>>> d24d5f3d83d4939ebf6228dd230caebf6ad72f26
 #include <opencv2/calib3d/calib3d.hpp>

ParticleFilter::ParticleFilter():
    numParticles(2), toolSize(2), perturbStd(0.001), newToolModel(Cam)
{

	// initial needle position guess
	// everything here is in meters.
	initial.tvec_cyl(0) = 0.0;
	initial.tvec_cyl(1) = -0.01;
	initial.tvec_cyl(2) = 0.0;
	initial.rvec_cyl(0) = 0.0;
	initial.rvec_cyl(1) = M_PI/2;
	initial.rvec_cyl(2) = 0.3;

    Cam = cv::Mat(4,4,CV_64FC1);
    Cam.at<double>(0,0) = 1;
    Cam.at<double>(1,0) = 0;
    Cam.at<double>(2,0) = 0;
    Cam.at<double>(3,0) = 0;    

    Cam.at<double>(0,1) = 0;
    Cam.at<double>(1,1) = -1;
    Cam.at<double>(2,1) = 0;
    Cam.at<double>(3,1) = 0;

    Cam.at<double>(0,2) = 0;
    Cam.at<double>(1,2) = 0;
    Cam.at<double>(2,2) = -1;
    Cam.at<double>(3,2) = 0;

    Cam.at<double>(0,3) = 0.0;   //should be in meters
    Cam.at<double>(1,3) = 0.1;
    Cam.at<double>(2,3) = 0.2;  // cannot have z = 0 for reprojection, camera_z must be always point to object
    Cam.at<double>(3,3) = 1;

	//newToolModel(Cam);

	//initialize particles by randomly assigning around the initial guess
	initializeParticles();
	ROS_INFO("---- Initialization is done---");

	//initialize needle image, just basit black image ??? how to get the size of the image
	toolImage_left = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right = cv::Mat::zeros(480, 640, CV_8UC3);
};

ParticleFilter::~ParticleFilter()
{

};


void ParticleFilter::initializeParticles()
{
	ROS_INFO("---- Initialize particle is called---");
	particles.resize(numParticles); //initialize particle array
	mathcingScores.resize(numParticles); //initialize matching score array
	particleWeights.resize(numParticles); //initialize particle weight array
	//generate random needle 
	for(int i(0); i<numParticles; i++)
	{
		particles[i] = newToolModel.setRandomConfig(initial, 1, 0);
	}

}


