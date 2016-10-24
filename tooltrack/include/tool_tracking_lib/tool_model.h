#ifndef TOOL_MODEL_H
#define TOOL_MODEL_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include <glm/vec3.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class ToolModel{
 private:

        //basic rotaions and traslations
		cv::Matx<double,3,1> tvec;	//translation vector
		cv::Matx<double,3,1> rvec;	//rotation vector

		cv::Matx<double,3,3> I;


		double theta_ellipse;
		double theta_grip_1;
		double theta_grip_2;

		std::vector< glm::vec3 > body_vertices;
		std::vector< glm::vec3 > ellipse_vertices;
		std::vector< glm::vec3 > griper_vertices;

 public:
 		ToolModel();  //cotructor

 		double randomNumber(double stdev, double mean);

 		 void load_model_vertices();



 		void setZeroConfig();

 		void setRandomConfig();

 		cv::Rect renderTool(cv::Mat &, const needleGeometry &, const cv::Mat &, int=1, cv::OutputArray = cv::noArray(), cv::OutputArray = cv::noArray());

 		double calculateMatchingScore(cv::Mat &, const cv::Mat &, cv::Rect &, bool = false);

};

#endif