#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <iostream>
#include <cwru_opencv_common/projective_geometry.h>

#include <tool_tracking_lib/tool_model.h>
#include <math.h>

using cv_projective::reprojectPoint;
boost::mt19937 rng(time(0));

//constructor
ToolModel::ToolModel(){

////////
	for(int i(0); i<3; i++){
		for(int j(0);j<3;j++){
			I(i)(j) = 0.0;
		}
	}
	I(1)(1) = 1;
	I(2)(2) = 1;
	I(3)(3) = 1;
///////
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

toolModel ToolModel::setRandomConfig(const toolModel &initial, double stdev, double mean)
{
	// copy initial toolModel
	toolModel newTool = initial;

	//create normally distributed random number with the given stdev and mean

	//TODO: pertub all translation components
	newTool.tvec(0) += randomNumber(stdev,mean); 
	newTool.tvec(1) += randomNumber(stdev,mean);
	newTool.tvec(2) += randomNumber(stdev,mean);
	
	double angle = randomNumber(stdev,mean);
	newTool.rvec(0) += (angle/10.0)*1000.0; //rotation on x axis +/-5 degrees

	angle = randomNumber(stdev,mean);
	newTool.rvec(0) += (angle/10.0)*1000.0; ////rotation on x axis +/-5 degrees

	angle = randomNumber(stdev,mean);
	newTool.rvec(2) += (angle/10.0)*1000.0; //rotation on z axis +/-5 degrees


    /**************smaple the angles of the joints**************/
	//-90,90//
	angle = randomNumber(stdev,mean);
	newTool.theta_ellipse += (angle/10.0)*1000.0;
	if (newTool.theta_ellipse < -M_PI/2 || newTool.theta_ellipse > M_PI/2)   //use M_PI HERE
		newTool.theta_ellipse = randomNumber(stdev,mean);

	// lets a assign the upside one 1, and set positive as clockwise 
	angle = randomNumber(stdev,mean);
	newTool.theta_grip_1 += (angle/10.0)*1000.0;
	if (newTool.theta_grip_1 < -1.2*M_PI/2 || newTool.theta_grip_1 > 1.2*M_PI/2)   //use M_PI HERE
		newTool.theta_grip_1 = randomNumber(stdev,mean);
	
	// lets a assign the udownside one 2, and set positive as clockwise
	angle = randomNumber(stdev,mean);
	newTool.theta_grip_2 += (angle/10.0)*1000.0;
	if (newTool.theta_grip_1 < -1.2*M_PI/2 || newTool.theta_grip_1 > 1.2*M_PI/2)   //use M_PI HERE
		newTool.theta_grip_1 = randomNumber(stdev,mean);

	///if the two joints get overflow/////
	if (newTool.theta_grip_1 > newTool.theta_grip_2)
		newTool.theta_grip_1 = newTool.theta_grip_2 - randomNumber(stdev,mean);



   /***********compute exponential map for forward kinematics**********/
	cv::Matx<double,3,3> roll_mat;
	cv::Matx<double,3,3> pitch_mat;
	cv::Matx<double,3,3> yaw_mat;

    roll_mat(0)(0) = 1;
    roll_mat(0)(1) = 0;
    roll_mat(0)(2) = 0;
    roll_mat(1)(0) = 0;
    roll_mat(1)(1) = cos(newTool.rvec(0));
    roll_mat(1)(2) = -sin(newTool.rvec(0));
    roll_mat(2)(0) = 0;
    roll_mat(2)(1) = sin(newTool.rvec(0));
    roll_mat(2)(2) = cos(newTool.rvec(0));

    pitch_mat(0)(0) = cos(newTool.rvec(1));
    pitch_mat(0)(1) = 0;
    pitch_mat(0)(2) = sin(newTool.rvec(1));
    pitch_mat(1)(0) = 0;
    pitch_mat(1)(1) = 1;
    pitch_mat(1)(2) = 0;
    pitch_mat(2)(0) = sin(newTool.rvec(1));
    pitch_mat(2)(1) = 0;
    pitch_mat(2)(2) = cos(newTool.rvec(1));

    yaw_mat(0)(0) = cos(newTool.rvec(2));
    yaw_mat(0)(1) = -sin(newTool.rvec(2));
    yaw_mat(0)(2) = 0;
    yaw_mat(1)(0) = sin(newTool.rvec(2));
    yaw_mat(1)(1) = cos(newTool.rvec(2));
    yaw_mat(1)(2) = 0;
    yaw_mat(2)(0) = 0;
    yaw_mat(2)(1) = 0;
    yaw_mat(2)(2) = 1;

    cv::Matx<double,3,3> rotation_mat;
    rotation_mat = yaw_mat * pitch_mat * roll_mat;
    
    //w is z of rotation mat
    
    newTool.w_z(0) = rotation_mat(0)(2);
    newTool.w_z(1) = rotation_mat(1)(2);
    newTool.w_z(2) = rotation_mat(2)(2);

    newTool.w_x(0) = rotation_mat(0)(0);
    newTool.w_x(1) = rotation_mat(1)(0);
    newTool.w_x(2) = rotation_mat(2)(0);
    /*get new tool model info*/

	return newTool;
}


ToolModel::setRandomConfig(double stdev, double mean){

	tvec(0) += randomNumber(stdev,mean);

    cv::Matx<double,3,3> w_mat;

	w_mat(0)(0) = 0;
    w_mat(0)(1) = -w_z(2);
    w_mat(0)(2) = w_z(1);
    w_mat(1)(0) = w_z(2);
    w_mat(1)(1) = 0;
    w_mat(1)(2) = -w_z(0);
    w_mat(2)(0) = -w_z(1);
    w_mat(2)(1) = w_z(0);
    w_mat(2)(2) = 0;

	cv::Matx<double,3,3> exp_R;
    exp_R = I + w_mat * sin(theta_ellipse) + w_mat * w_mat * (1-cos(theta_ellipse));



};