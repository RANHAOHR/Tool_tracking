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


#include <glm/glm.hpp>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>



class ToolModel{

//private:
public:
		struct toolModel {
			cv::Matx<double,3,1> tvec_cyl;	//cylinder translation vector
			cv::Matx<double,3,1> rvec_cyl;	//cylinder rotation vector

			cv::Matx<double,3,1> tvec_elp;	//ellipse translation vector
			cv::Matx<double,3,1> rvec_elp;	//ellipse rotation vector

			cv::Matx<double,3,1> tvec_grip1;	//gripper1 translation vector
			cv::Matx<double,3,1> rvec_grip1;	//gripper1 rotation vector

			cv::Matx<double,3,1> tvec_grip2;	//gripper2 translation vector
			cv::Matx<double,3,1> rvec_grip2;	//gripper2 rotation vector

			double theta_ellipse;
			double theta_grip_1;
			double theta_grip_2;  //9 DOF



			//built in operator that handles copying the toolModel in efficient way
			toolModel& operator =(const toolModel src){
				this->theta_ellipse = src.theta_ellipse;
				this->theta_grip_1 = src.theta_grip_1;
				this->theta_grip_2 = src.theta_grip_2;

				for(int i(0); i<3; i++){

					this->tvec_cyl(i) = src.tvec_cyl(i);
					this->rvec_cyl(i) = src.rvec_cyl(i);

					this->tvec_elp(i) = src.tvec_elp(i);
					this->rvec_elp(i) = src.rvec_elp(i);

					this->tvec_grip1(i) = src.tvec_grip1(i);
					this->rvec_grip1(i) = src.rvec_grip1(i);

					this->tvec_grip2(i) = src.tvec_grip2(i);
					this->rvec_grip2(i) = src.rvec_grip2(i);


				}
				return *this;
			}

			// toolModel constructor, creates an empty toolModel
			toolModel(void){
				this->theta_ellipse = 0.0;
				this->theta_grip_1 = 0.0;
				this->theta_grip_2 = 0.0;
				for(int i(0); i<3; i++){
					this->tvec_cyl(i) = 0.0;
					this->rvec_cyl(i) = 0.0;

					this->tvec_elp(i) = 0.0;
					this->rvec_elp(i) = 0.0;

					this->tvec_grip1(i) = 0.0;
					this->rvec_grip1(i) = 0.0;	

					this->tvec_grip2(i) = 0.0;
					this->rvec_grip2(i) = 0.0;	

				}
			}

		};   //end struct
        

/******************the model points and vecs********************/
		std::vector< glm::vec3 > body_vertices;
		std::vector< glm::vec3 > ellipse_vertices;
		std::vector< glm::vec3 > griper1_vertices;
		std::vector< glm::vec3 > griper2_vertices;

		std::vector< glm::vec3 > body_Vnormal;    //vertex normal, for the computation of the silhouette
		std::vector< glm::vec3 > ellipse_Vnormal;
		std::vector< glm::vec3 > griper1_Vnormal;
		std::vector< glm::vec3 > griper2_Vnormal;

		std::vector< cv::Point3d > body_Vpts;     ////to save time from converting
		std::vector< cv::Point3d > ellipse_Vpts;
		std::vector< cv::Point3d > griper1_Vpts;
		std::vector< cv::Point3d > griper2_Vpts;

		std::vector< cv::Point3d > body_Npts;    //vertex normal, for the computation of the silhouette
		std::vector< cv::Point3d > ellipse_Npts;
		std::vector< cv::Point3d > griper1_Npts;
		std::vector< cv::Point3d > griper2_Npts;

		cv::Mat body_Vmat;
		cv::Mat ellipse_Vmat;
		cv::Mat gripper1_Vmat;
		cv::Mat gripper2_Vmat;
/***************cam view points************************/

		std::vector< cv::Point3d > CamBodyPts;     ////points in camera coord
		std::vector< cv::Point3d > CamEllipPts;
		std::vector< cv::Point3d > CamGripPts_1;
		std::vector< cv::Point3d > CamGripPts_2;

		std::vector< cv::Point3d > CamBodyNorms;    //vertex normal, for the computation of the silhouette
		std::vector< cv::Point3d > CamEllipNorms;
		std::vector< cv::Point3d > CamGripNorms_1;
		std::vector< cv::Point3d > CamGripNorms_2;		
/**************************Faces informations******************************************/

		std::vector< std::vector<int> > body_faces;
		std::vector< std::vector<int> > ellipse_faces;
		std::vector< std::vector<int> > griper1_faces;
		std::vector< std::vector<int> > griper2_faces;

		std::vector< std::vector<int> > body_neighbors;
		std::vector< std::vector<int> > ellipse_neighbors;
		std::vector< std::vector<int> > griper1_neighbors;
		std::vector< std::vector<int> > griper2_neighbors;

		cv::Mat bodyFace_info;
		cv::Mat ellipseFace_info;
		cv::Mat gripper1Face_info;
		cv::Mat gripper2Face_info;

		cv::Mat bodyFace_normal;
		cv::Mat ellipseFace_normal;
		cv::Mat gripper1Face_normal;
		cv::Mat gripper2Face_normal;

		cv::Mat bodyFace_centroid;
		cv::Mat ellipseFace_centroid;
		cv::Mat gripper1Face_centroid;
		cv::Mat gripper2Face_cnetroid;


/******************************************************************/

		cv::Mat CamMat;

		double offset_body;
		double  offset_ellipse; //meter
        double offset_gripper; //

        cv::Mat q_ellipse;  //initial point for ellipse
        cv::Mat q_gripper;  //intial point for girpper 


 		ToolModel(cv::Mat& CamMat);  //constructor

 		double randomNumber(double stdev, double mean);

 		void load_model_vertices(const char * path, std::vector< glm::vec3 > &out_vertices, std::vector< glm::vec3 > &vertex_normal, 
                                 std::vector< std::vector<int> > &out_faces,  std::vector< std::vector<int> > &neighbor_faces );

		void modify_model_(std::vector< glm::vec3 > &input_vertices, std::vector< glm::vec3 > &input_Vnormal, 
                              std::vector< cv::Point3d > &input_Vpts, std::vector< cv::Point3d > &input_Npts, double &offset, cv::Mat &input_Vmat);//there are offsets when loading the model convert 

		cv::Mat collectFacesTranform(const std::vector< std::vector<int> > &input_faces, const std::vector< cv::Point3d > &input_vertices, 
                                    const std::vector< cv::Point3d > &input_Vnormal);

		cv::Mat getFaceNormals(const std::vector< std::vector<int> > &input_faces, const std::vector< cv::Point3d > &input_vertices, const std::vector< cv::Point3d > &input_Vnormal);

		cv::Mat getFaceCentroid(const std::vector< std::vector<int> > &input_faces, const std::vector< cv::Point3d > &input_vertices);

 		toolModel setRandomConfig(const toolModel &initial, double stdev, double mean);

 		//cam view need to be modified
 		cv::Rect renderTool(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P, cv::OutputArray = cv::noArray() );

 		/**************compute silhoutes*****************/
 		void Compute_Silhouette(const std::vector< std::vector<int> > &input_faces, const std::vector< std::vector<int> > &neighbor_faces, 
                                   const std::vector< cv::Point3d > &body_vertices, const std::vector< cv::Point3d > &body_Vnormal, cv::Mat &CamMat,
                                   cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &P, cv::OutputArray jac, cv::Point2d &XY_max, cv::Point2d &XY_min);

		void Compute_Silhouette( const std::vector< std::vector<int> > &input_faces, const std::vector< std::vector<int> > &neighbor_faces, 
                                 const cv::Mat &input_Vmat, cv::Mat &face_normal, cv::Mat &face_centroid, cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec, 
                                 const cv::Mat &P, cv::OutputArray jac, cv::Point2d &XY_max, cv::Point2d &XY_min);

 		int Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2, std::vector<int> &match_vec);
 		cv::Point3d transformation_nrms(const cv::Point3d &vec, const cv::Mat& rvec, const cv::Mat &tvec);
 		cv::Point3d transformation_pts(const cv::Point3d &point, const cv::Mat& rvec, const cv::Mat &tvec);

 		cv::Mat transformMats(const cv::Mat &face_info, cv::Mat &CamMat, const cv::Mat &rvec, const cv::Mat &tvec);
 		/**********************math computation*******************/
 		cv::Point3d crossProduct(cv::Point3d &vec1, cv::Point3d &vec2);
 		double dotProduct(cv::Point3d &vec1, cv::Point3d &vec2);

 		double dotProductMat(cv::Mat &vec1, cv::Mat &vec2);

 		cv::Point3d Normalize(cv::Point3d &vec1);
 		cv::Point3d ConvertCelitoMeters(cv::Point3d &input_pt);
 		void ConvertInchtoMeters(std::vector< cv::Point3d > &input_vertices);
 		cv::Point3d FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                                     cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3);

		void Convert_glTocv_pts(std::vector< glm::vec3 > &input_vertices, std::vector< cv::Point3d > &out_vertices);
 		cv::Point3d Convert_glTocv_pt(glm::vec3 &input_vertex);

 		cv::Mat computeSkew(cv::Mat &w);

 		cv::Point3d convert_MattoPts(cv::Mat &input_Mat);
		/*************camera transforms************************/
 		void camTransformPoints(cv::Mat &cam_mat, std::vector< cv::Point3d > &input_vertices, std::vector< cv::Point3d > &output_vertices);
 		void camTransformVecs(cv::Mat &cam_mat, std::vector< cv::Point3d > &input_normals, std::vector< cv::Point3d > &output_normals);
 		cv::Point3d camTransformPoint(cv::Mat &cam_mat, cv::Point3d &input_vertex);
 		cv::Point3d camTransformVec(cv::Mat &cam_mat, cv::Point3d &input_vec);
 		cv::Mat camTransformMats(cv::Mat &cam_mat, cv::Mat &input_mat );

 		cv::Point2d reproject(const cv::Point3d &point, const cv::Mat &P);



        // double calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage, cv::Rect &ROI, bool displayPause);

 

};

#endif