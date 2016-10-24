#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <cwru_opencv_common/projective_geometry.h>

#include <tool_tracking_lib/tool_model.h>

using cv_projective::reprojectPoint;
boost::mt19937 rng(time(0));

//constructor
ToolModel::ToolModel(){

	for(int i(0); i<3; i++){
	tvec(i) = 0.0;
	rvec(i) = 0.0; 
	}

	theta_ellipse = 0.0;
	theta_grip_1 = 0.0;
	theta_grip_2 = 0.0;

	for(int i(0); i<3; i++){
		for(int j(0);j<3;j++){
			I(i)(j) = 0.0;
		}
	}
	I(1)(1) = 1;
	I(2)(2) = 1;
	I(3)(3) = 1;

};

double ToolModel::randomNumber(double stdev, double mean){
	boost::normal_distribution<> nd(mean, stdev);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
	double d = var_nor();

	return d;

}

//set zero configuratio for tool points;
ToolModel::load_model_vertices(){

};


ToolModel::setZeroConfig(){
	//std::vector< std::vector<double> > exp_R;
	//exp_R.resize(3);


	cv::Matx<double,3,3> roll_mat;
	cv::Matx<double,3,3> pitch_mat;
	cv::Matx<double,3,3> yaw_mat;

    roll_mat(0)(0) = 0;
    roll_mat(0)(1) = -rvec(3);
    roll_mat(0)(2) = rvec(2);
    roll_mat(1)(0) = rvec(3);
    rvec_mat(1)(1) = 0;
    rvec_mat(1)(2) = -rvec(1);
    rvec_mat(2)(0) = -rvec(2);
    rvec_mat(2)(1) = rvec(1);
    rvec_mat(2)(2) = 0;	

    
    //w is rvec
    rvec_mat(0)(0) = 0;
    rvec_mat(0)(1) = -rvec(3);
    rvec_mat(0)(2) = rvec(2);
    rvec_mat(1)(0) = rvec(3);
    rvec_mat(1)(1) = 0;
    rvec_mat(1)(2) = -rvec(1);
    rvec_mat(2)(0) = -rvec(2);
    rvec_mat(2)(1) = rvec(1);
    rvec_mat(2)(2) = 0;

	cv::Matx<double,3,3> exp_R;
    exp_R = I + rvec_mat * sin(theta_ellipse) + rvec_mat * rvec_mat * (1-cos(theta_ellipse));



};


ToolModel::setRandomConfig(){







};