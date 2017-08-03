#ifndef TOOL_MODEL_H
#define TOOL_MODEL_H

#include <vector>
#include <stdio.h>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include <string>
#include <cstring>

#include <glm/glm.hpp>

#include <glm/vec3.hpp>
#include <glm/vec2.hpp>
#include <cwru_opencv_common/projective_geometry.h>
#include <ros/package.h>

class ToolModel {

private:

/**
 * @brief tool_model_pkg gives the path finding all the tool body parts inside the workspace
 */
std::string tool_model_pkg;

public:
    struct toolModel {
        cv::Matx<double, 3, 1> tvec_cyl;    //cylinder translation vector
        cv::Matx<double, 3, 1> rvec_cyl;    //cylinder rotation vector

        cv::Matx<double, 3, 1> tvec_elp;    //ellipse translation vector
        cv::Matx<double, 3, 1> rvec_elp;    //ellipse rotation vector

        cv::Matx<double, 3, 1> tvec_grip1;    //gripper1 translation vector
        cv::Matx<double, 3, 1> rvec_grip1;    //gripper1 rotation vector

        cv::Matx<double, 3, 1> tvec_grip2;    //gripper2 translation vector
        cv::Matx<double, 3, 1> rvec_grip2;    //gripper2 rotation vector

        //built in operator that handles copying the toolModel in efficient way
        toolModel &operator=(const toolModel src) {

            for (int i(0); i < 3; i++) {

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
        toolModel() {
            for (int i(0); i < 3; i++) {
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

    /**
     * The tool model pieces, including the vertices and vertex normals of cylinder, oval gripper 1 and 2
     */
    std::vector<glm::vec3> body_vertices; ///cylinder
    std::vector<glm::vec3> ellipse_vertices;   ///oval
    std::vector<glm::vec3> griper1_vertices;   ///griper1
    std::vector<glm::vec3> griper2_vertices;   ///griper2

    std::vector<glm::vec3> body_Vnormal;    //vertex normal, for the computation of the silhouette
    std::vector<glm::vec3> ellipse_Vnormal;
    std::vector<glm::vec3> griper1_Vnormal;
    std::vector<glm::vec3> griper2_Vnormal;

    std::vector<cv::Point3d> body_Vpts;     ////to save time from converting in real time
    std::vector<cv::Point3d> ellipse_Vpts;
    std::vector<cv::Point3d> griper1_Vpts;
    std::vector<cv::Point3d> griper2_Vpts;

    std::vector<cv::Point3d> body_Npts;    //vertex normal, for the computation of the silhouette
    std::vector<cv::Point3d> ellipse_Npts;
    std::vector<cv::Point3d> griper1_Npts;
    std::vector<cv::Point3d> griper2_Npts;

    cv::Mat body_Vmat;
    cv::Mat ellipse_Vmat;
    cv::Mat gripper1_Vmat;
    cv::Mat gripper2_Vmat;

    cv::Mat body_Nmat;
    cv::Mat ellipse_Nmat;
    cv::Mat gripper1_Nmat;
    cv::Mat gripper2_Nmat;

    /**
     * The Faces information, including the face and the corresponding neighbor face of cylinder, oval gripper 1 and 2
     */
    std::vector<std::vector<int> > body_faces;
    std::vector<std::vector<int> > ellipse_faces;
    std::vector<std::vector<int> > griper1_faces;
    std::vector<std::vector<int> > griper2_faces;

    std::vector<std::vector<int> > body_neighbors;
    std::vector<std::vector<int> > ellipse_neighbors;
    std::vector<std::vector<int> > griper1_neighbors;
    std::vector<std::vector<int> > griper2_neighbors;

    cv::Mat bodyFace_normal;
    cv::Mat ellipseFace_normal;
    cv::Mat gripper1Face_normal;
    cv::Mat gripper2Face_normal;

    cv::Mat bodyFace_centroid;
    cv::Mat ellipseFace_centroid;
    cv::Mat gripper1Face_centroid;
    cv::Mat gripper2Face_centroid;

    /**
     * This is the part for UKF tracking, in order to get part of the oval vertice and normals
     */
    std::vector<glm::vec3> oval_normal_vertices;
    std::vector<glm::vec3> oval_normal_Vnormal;
    std::vector<cv::Point3d> oval_normal_Vpts;
    std::vector<cv::Point3d> oval_normal_Npts;
    cv::Mat oval_normal_Vmat;
    cv::Mat oval_normal_Nmat;

    std::vector<std::vector<int> > oval_normal_faces;
    std::vector<std::vector<int> > oval_normal_neighbors;
    cv::Mat oval_normalFace_normal;
    cv::Mat oval_normalFace_centroid;

    /**
     * The offsets for modify model
     */
    double offset_body;
    double offset_ellipse; // all in meters
    double offset_gripper; //

    /**
     * Constructor
     */
    ToolModel();

    /**
     * @brief Random number generators
     */
    double randomNumber(double stdev, double mean);
    double randomNum(double min, double max);

    /**
     * @brief loading the vertices and normals of the tool model and use the faces to represent the tool, offline.
     * @param path : absolute path
     * @param out_vertices : output vertices
     * @param vertex_normal : output vertex normals
     * @param out_faces : output faces
     * @param neighbor_faces : output neighbor faces
     */
    void
    load_model_vertices(const char *path, std::vector<glm::vec3> &out_vertices, std::vector<glm::vec3> &vertex_normal,
                        std::vector<std::vector<int> > &out_faces, std::vector<std::vector<int> > &neighbor_faces);

    /**
     * @brief Adjusting the model geometry put four body parts back to their own frames
     * @param input_vertices
     * @param input_Vnormal
     * @param input_Vpts
     * @param input_Npts
     * @param offset
     * @param input_Vmat
     * @param input_Nmat
     */
    void modify_model_(std::vector<glm::vec3> &input_vertices, std::vector<glm::vec3> &input_Vnormal,
                       std::vector<cv::Point3d> &input_Vpts, std::vector<cv::Point3d> &input_Npts, double &offset,
                       cv::Mat &input_Vmat, cv::Mat &input_Nmat);//there are offsets when loading the model convert

    /**
     * @brief Generating random particles when given a seed pose
     * @param seeds : input seed pose
     * @param theta_cylinder : the 5th joint angle
     * @param theta_oval : the 6th joint angle
     * @param theta_open : the last joint angle
     * @param step : the sdev for the random generator
     * @return a particle represents a tool pose
     */
    toolModel setRandomConfig(const toolModel &seeds, const double &theta_cylinder, const double &theta_oval, const double &theta_open, double &step);
    /**
     * @brief Perturbation function during the tracking, updating the particles using gaussian noises
     * @param max_pose
     * @param step
     * @return
     */
    ToolModel::toolModel gaussianSampling(const toolModel &max_pose, double &step);

    /**
     * @brief computing the pose for the tool model based on the CYLINDER pose, return the pose within inputModel
     * @param inputmodel
     * @param theta_ellipse
     * @param theta_grip_1
     * @param theta_grip_2
     */
    void computeEllipsePose(toolModel &inputmodel, const double &theta_ellipse, const double &theta_grip_1,
                          const double &theta_grip_2);
    /**
     * @brief computing the tool pose using the seed pose, satisfied some constraints, which is different from the computeEllipsePose() function
     * @param seed_pose
     * @param inputModel
     * @param theta_tool
     * @param theta_grip_1
     * @param theta_grip_2
     */
    void computeRandomPose(const toolModel &seed_pose, toolModel &inputModel, const double &theta_tool, const double &theta_grip_1,
                                      const double &theta_grip_2);
    /**
     * @brief The rendering function  of four body parts, for PF
     * @param image
     * @param tool
     * @param CamMat
     * @param P
     */
    void renderTool(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P,
                        cv::OutputArray = cv::noArray());

    /**
     * @brief The rendering function for UKF, need vertex normals to compute measurement model
     * @param image
     * @param tool
     * @param CamMat
     * @param P
     * @param tool_points
     * @param tool_normals
     */
    void renderToolUKF(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P,
                       cv::Mat &tool_points, cv::Mat &tool_normals, cv::OutputArray = cv::noArray());

    /**
     * @brief Reprojecting a point to the image using the projection matrix
     * @param point
     * @param P
     * @return
     */
    cv::Point2d reproject(const cv::Mat &point, const cv::Mat &P);

    /**
     * @brief Computing the matching score using opencv function: templatemathcing.
     * @param toolImage
     * @param segmentedImage
     * @return
     */
    float calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage);
    /**
     * @brief Computing the matching score using Chamfer matching algorithm.
     * @param toolImage
     * @param segmentedImage
     * @return
     */
    float calculateChamferScore(cv::Mat &toolImage, const cv::Mat &segmentedImage);

    /**
     * @brief Silhouette extraction function, using the prepared vertex normals and vertices
     * @param input_faces
     * @param neighbor_faces
     * @param input_Vmat
     * @param input_Nmat
     * @param CamMat
     * @param image
     * @param rvec
     * @param tvec
     * @param P
     * @param jac
     */
    void Compute_Silhouette(const std::vector<std::vector<int> > &input_faces,
                            const std::vector<std::vector<int> > &neighbor_faces,
                            const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                            cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                            const cv::Mat &P, cv::OutputArray jac);

    /**
     * @brief Silhouette extraction function for UKF, need extra vertices_vector, stores the sampled vertices
     * @param input_faces
     * @param neighbor_faces
     * @param input_Vmat
     * @param input_Nmat
     * @param CamMat
     * @param image
     * @param rvec
     * @param tvec
     * @param P
     * @param vertices_vector
     * @param jac
     */
    void Compute_Silhouette_UKF(const std::vector<std::vector<int> > &input_faces,
                                           const std::vector<std::vector<int> > &neighbor_faces,
                                           const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                                           cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                                           const cv::Mat &P, std::vector<std::vector<double> > &vertices_vector,
                                           cv::OutputArray jac);

    /**
     * @brief Computing cross product for cv::Point3d
     * @param vec1
     * @param vec2
     * @return
     */
    cv::Point3d crossProduct(cv::Point3d &vec1, cv::Point3d &vec2);

    /**
     * @brief Computing the dot product of two cv::Point3d points
     * @param vec1
     * @param vec2
     * @return
     */
    double dotProduct(cv::Point3d &vec1, cv::Point3d &vec2);

    /**
     * @brief a vector normalization function
     * @param vec1
     * @return
     */
    cv::Point3d Normalize(cv::Point3d &vec1);

    /**
     * @brief Converting the 3D points in inch to meters, using this in the loading function
     * @param input_vertices
     */
    void ConvertInchtoMeters(std::vector<cv::Point3d> &input_vertices);

    /**
     * @brief Converting the opengl points to opencv points
     * @param input_vertices
     * @param out_vertices
     */
    void Convert_glTocv_pts(std::vector<glm::vec3> &input_vertices, std::vector<cv::Point3d> &out_vertices);

    /**
     * @brief Compute a so(3) skew symetric matrix using the w vector
     * @param w
     * @return
     */
    cv::Mat computeSkew(cv::Mat &w);

    cv::Point3d convert_MattoPts(cv::Mat &input_Mat);

    /**
     * @brief Compute inverse of a SE(3) matrix
     * @param inputMat
     * @param outputMat
     */
    void computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat);
    /**
     * @brief Getting the face normal from the vertex normals
     * @param input_v1
     * @param input_v2
     * @param input_v3
     * @param input_n1
     * @param input_n2
     * @param input_n3
     * @return
     */
    cv::Point3d FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                               cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3);

    /**
     * @brief Computing the shared vertices in orderto find the neighbor face later in the load_model_vertices function
     * @param vec1
     * @param vec2
     * @param match_vec
     * @return
     */
    int Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2, std::vector<int> &match_vec);

    /**
     * @brief Finding the face centroid and the face normal
     * @param input_faces
     * @param input_vertices
     * @param input_Vnormal
     * @param face_normals
     * @param face_centroids
     */
    void getFaceInfo(const std::vector<std::vector<int> > &input_faces, const std::vector<cv::Point3d> &input_vertices,
                     const std::vector<cv::Point3d> &input_Vnormal, cv::Mat &face_normals, cv::Mat &face_centroids);

    /**
     * @brief Transforming the input matrix under the camera frame
     * @param cam_mat
     * @param input_mat
     * @return
     */
    cv::Mat camTransformMats(cv::Mat &cam_mat, cv::Mat &input_mat);

    /**
     * @brief Gathering the normals of the corresponding sampled points for computing the measurement model of the UKF tracking
     * @param part1_normals
     * @param part2_normals
     * @param part3_normals
     * @param tool_points
     * @param tool_normals
     */
    void gatherNormals(std::vector< std::vector<double> > &part1_normals, std::vector< std::vector<double> > &part2_normals, std::vector< std::vector<double> > &part3_normals, cv::Mat &tool_points, cv::Mat &tool_normals);

};

#endif