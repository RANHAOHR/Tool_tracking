#ifndef TOOL_MODEL_H
#define TOOL_MODEL_H

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

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>


class ToolModel{
 private:

		struct toolModel {
			cv::Matx<double,3,1> tvec;	//translation vector
			cv::Matx<double,3,1> rvec;	//rotation vector
			double theta_ellipse;
			double theta_grip_1;
			double theta_grip_2;  //9 DOF

			cv::Matx<double,3,1> w_z;   //for computational purpose
		    cv::Matx<double,3,1> w_x;

			//built in operator that handles copying the toolModel in efficient way
			toolModel& operator =(const toolModel src){
				this->theta_ellipse = src.theta_ellipse;
				this->theta_grip_1 = src.theta_grip_1;
				this->theta_grip_2 = src.theta_grip_2;
				for(int i(0); i<3; i++){
					this->tvec(i) = src.tvec(i);
					this->rvec(i) = src.rvec(i);
					this->w_z(i) = src.w_z(i);
					this->w_x(i) = src.w_x(i);
				}
				return *this;
			}

			// toolModel constructor, creates an empty toolModel
			toolModel(void){
				this->theta_ellipse = 0.0;
				this->theta_grip_1 = 0.0;
				this->theta_grip_2 = 0.0;
				for(int i(0); i<3; i++){
					this->tvec(i) = 0.0;
					this->rvec(i) = 0.0;
					this->w_z(i) = 0.0;
					this->w_x(i) = 0.0; 
				}
			}

		};   //end struct
        
		cv::Matx<double,3,3> I;

		std::vector< glm::vec3 > body_vertices;
		std::vector< glm::vec3 > ellipse_vertices;
		std::vector< glm::vec3 > griper_vertices;

 public:
 		ToolModel();  //cotructor

 		double randomNumber(double stdev, double mean);

 		void load_model_vertices(std::vector< glm::vec3 > &out_vertices );

 		toolModel setRandomConfig(const toolModel &initial, double stdev, double mean);

 		cv::Rect renderTool(const toolModel &initial, double stdev, double mean);

 		// double calculateMatchingScore();

};

#endif