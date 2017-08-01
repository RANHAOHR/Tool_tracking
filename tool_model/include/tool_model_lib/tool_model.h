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
 * @param tool_model_pkg gives the path finding all the tool body parts inside the workspace
 */
std::string tool_model_pkg;

public:
    struct toolModel {
        cv::Matx<double, 3, 1> tvec_cyl;    //cylinder translation vector <x,y,z> represents joints 1-4
        cv::Matx<double, 3, 1> rvec_cyl;    //cylinder rotation vector <r,p,y> represents joints 1-4

        cv::Matx<double, 3, 1> tvec_elp;    //ellipse translation vector
        cv::Matx<double, 3, 1> rvec_elp;    //ellipse rotation vector

        cv::Matx<double, 3, 1> tvec_grip1;    //gripper1 translation vector
        cv::Matx<double, 3, 1> rvec_grip1;    //gripper1 rotation vector

        cv::Matx<double, 3, 1> tvec_grip2;    //gripper2 translation vector
        cv::Matx<double, 3, 1> rvec_grip2;    //gripper2 rotation vector

        //instantiate struct
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
    };

    //vector of all glm::vec3 representation of vertices/normals
    //each glm::vec3 stores the <x,y,z> for the vertex/normal
    std::vector<glm::vec3> body_vertices;
    std::vector<glm::vec3> ellipse_vertices;
    std::vector<glm::vec3> griper1_vertices;
    std::vector<glm::vec3> griper2_vertices;
    std::vector<glm::vec3> body_Vnormal;
    std::vector<glm::vec3> ellipse_Vnormal;
    std::vector<glm::vec3> griper1_Vnormal;
    std::vector<glm::vec3> griper2_Vnormal;

    //vector of all cv::Point3d (not homogeneous) representation of vertices/normals
    //each Point3d stores the <x,y,z> for the vertex/normal
    std::vector<cv::Point3d> body_Vpts;
    std::vector<cv::Point3d> ellipse_Vpts;
    std::vector<cv::Point3d> griper1_Vpts;
    std::vector<cv::Point3d> griper2_Vpts;
    std::vector<cv::Point3d> body_Npts;
    std::vector<cv::Point3d> ellipse_Npts;
    std::vector<cv::Point3d> griper1_Npts;
    std::vector<cv::Point3d> griper2_Npts;

    //cv::Mat homogeneous representation of vertices/normals
    //each vertex contains <x,y,z,1> each normal contains <x,y,z,0>
    cv::Mat body_Vmat;
    cv::Mat ellipse_Vmat;
    cv::Mat gripper1_Vmat;
    cv::Mat gripper2_Vmat;
    cv::Mat body_Nmat;
    cv::Mat ellipse_Nmat;
    cv::Mat gripper1_Nmat;
    cv::Mat gripper2_Nmat;

    //vector of all faces
    //each face stores the indices, not values,for <v1,v2,v3,n1,n2,n3>
    std::vector<std::vector<int> > body_faces;
    std::vector<std::vector<int> > ellipse_faces;
    std::vector<std::vector<int> > griper1_faces;
    std::vector<std::vector<int> > griper2_faces;

    //vector of all neighbors of each face
    //Outer vector indices refers to each face
    //inner vector stores each face which is a neighbor to the face given by outer vector
    //each neighbor contains indices <f, v1,n1,v2,n2>
    //f is index of neighbor face, and v/n are vertices/normals that define the face
    std::vector<std::vector<int> > body_neighbors;
    std::vector<std::vector<int> > ellipse_neighbors;
    std::vector<std::vector<int> > griper1_neighbors;
    std::vector<std::vector<int> > griper2_neighbors;

    //fcv::Mat homogeneous representation of face normals
    //each face normal contains <x,y,z,0> of face
    cv::Mat bodyFace_normal;
    cv::Mat ellipseFace_normal;
    cv::Mat gripper1Face_normal;
    cv::Mat gripper2Face_normal;

    //cv::Mat homogeneous representation of face centroids
    //each centroid contains normalized <x,y,z,1>
    cv::Mat bodyFace_centroid;
    cv::Mat ellipseFace_centroid;
    cv::Mat gripper1Face_centroid;
    cv::Mat gripper2Face_centroid;

    //Get same info for oval to be used for UKF
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

    //Offsets from the tool origin (10cm up the cylinder) to the tool part's origin in meters
    double offset_body;
    double offset_ellipse;
    double offset_gripper;

    /**
     * @brief Constructor
     * Set model params according to their geometry
     * Load/compute vertices/normals/faces/neighbor_faces for cyl, elp, gripper1, gripper2
     */
    ToolModel();

    /**
     * @brief return a value sampled from a gaussian distribution
     * Used for gaussian perturbations of state
     * @param stdev
     * @param mean
     * @return
     */
    double randomNumber(double stdev, double mean);

    /**
     * @brief return a random double between two values
     * Used in resampling step
     * @param min
     * @param max
     * @return random double
     */
    double randomNum(double min, double max);

    /**
     * @brief Load OBJ file and store its vertices, textures, normals, faces, neighbors
     * Then convert data from gl to cv::Point3d and cv::mat and compute offsets from tool origin to part origin
     * Used to initialize tool geometries
     * @param path file path to OBJ file for tool
     * @param out_vertices  vertices of tool
     * @param vertex_normal their normals
     * @param out_faces tool faces.  6x1 indices of <v1,v2,v3,n1,n2,n3>
     * @param neighbor_faces 5x1 <face index, v1,n1,v2,n2>
     */
    void load_model_vertices(const char *path, std::vector<glm::vec3> &out_vertices, std::vector<glm::vec3> &vertex_normal,
                        std::vector<std::vector<int> > &out_faces, std::vector<std::vector<int> > &neighbor_faces);

    /**
     * @brief  offset each part relative to tool origin (10cm up cylinder)
     * convert storage from glm to cv::Point3d and cv::Mat
     * Used in constructor to define tool geometry
     * @param input_vertices vertices of tool part, glm::vec3
     * @param input_Vnormal normals, glm::vec3
     * @param input_Vpts vertices of tool part, cv::Point3d
     * @param input_Npts normals, cv::Point3d
     * @param offset offeset from origin (10cm up cylinder) to start of tool
     * @param input_Vmat vertices of tool part, cv::mat
     * @param input_Nmat normals of tool part, cv::mat
     */
    void modify_model_(std::vector<glm::vec3> &input_vertices, std::vector<glm::vec3> &input_Vnormal,
                       std::vector<cv::Point3d> &input_Vpts, std::vector<cv::Point3d> &input_Npts, double &offset,
                       cv::Mat &input_Vmat, cv::Mat &input_Nmat);//there are offsets when loading the model convert

    /**
     * @brief - Generate a random configuration near a given configuration
     * used only on initialization to generate random particles around the FK guess
     * @param seeds toolModel representation of FK guess of joints 1-4
     * @param theta_cylinder FK guess of joint 5
     * @param theta_oval FK guess of joint 6
     * @param theta_open FK guess of joint 7
     * @return toolModel representation of a particle randomly perturbed from inputs
     */
    toolModel setRandomConfig(const toolModel &seeds, const double &theta_cylinder, const double &theta_oval, const double &theta_open, double &pos, double &rot);

    /**
     * @brief randomly perturb a particle
     * Used as particle filter's motion model (after initialization)
     * @param max_pose particle to perturb
     * @param step scaling factor for perturbation (higher perturbations early and less perturbations later)
     * @return
     */
    ToolModel::toolModel gaussianSampling(const toolModel &max_pose, double &pos, double &rot );

    /**
     * @brief Compute toolModel pose given a toolModel representation of joints 1-4 and a double representation of joints 5,6,7
     * Used on initialization to compute toolModel representation of all new particles from setRandomConfig()
     * @param inputmodel output pose stored as a toolModel (at input already contains joints 1-4)
     * @param theta_ellipse joint 5
     * @param theta_grip_1 joint 6
     * @param theta_grip_2 joint 7
     */
    void computeEllipsePose(toolModel &inputmodel, const double &theta_ellipse, const double &theta_grip_1,
                            const double &theta_grip_2);

    /**
     * @brief Given already perturbed joints 1-4 and toolModel representation of 5,6,7 plus their noise, compute a gaussian perturbed toolModel
     * Used by gaussianSampling and used in particle filter's motion model
     * @param seed_pose unperturbed toolModel; used to get joints 5,6,7
     * @param inputModel contains gaussian perturbed joints 1-4 and empty 5,6,7
     * @param theta_tool gaussian delta joint 5
     * @param theta_grip_1 gaussian delta joint 6
     * @param theta_grip_2 gaussian delta joint 7
     * Note in setRandomConfig, joints are the joint values, not joint noise values
     */
    void computeRandomPose(const toolModel &seed_pose, toolModel &inputModel, const double &theta_tool, const double &theta_grip_1,
                                      const double &theta_grip_2);
    /**
     * @brief render a tool given its toolModel pose onto an image
     * do so by finding/rendering the silhoutte of all 4 of its components (cyl, elp, grip1, grip2)
     * Used for visualization purposes and for measurement model
     * @param image image to render on
     * @param tool pose of tool
     * @param CamMat Gcb from calibration
     * @param P  Camera projection matrix 3x4
     */
    void renderTool(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P,
                        cv::OutputArray = cv::noArray());

    void renderToolUKF(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P,
                       cv::Mat &tool_points, cv::Mat &tool_normals, cv::OutputArray = cv::noArray());

    /**
     * @brief reproject a single point under the camera
     * Used to project edge points to draw in Compute_Silhouette
     * @param point to project
     * @param P Camera projection matrix 3x4
     * @return point projected under camera frame
     */
    cv::Point2d reproject(const cv::Mat &point, const cv::Mat &P);

    float calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage);

    /**
     * @brief return the chamfer distance between a segmented image and a particle rendering
     * Chamfer distance is the closest distance between every pixel of one image to another image
     * Used as the measurement model for particle filter
     * @param toolImage the particle image
     * @param segmentedImage the segmented image from camera of the tool pose
     * @return
     */
    float calculateChamferScore(cv::Mat &toolImage, const cv::Mat &segmentedImage);

    /**
     * @brief compute the silhouette of a part
     * First transform the tool part from its offline pose its current pose in camera frame using rvec, tvec, and camMat
     * Next iterate through the faces list and if a face is front facing and has neighbors:
     *      Then iterate through the neighbors and when a neighbor face is back facing, draw the edge between them
     * Used for visualization and for measurement model
     * @param CamMat Gcb from calibration/tf.listener
     * @param image Image to render on
     * @param rvec rotation vector (of cyl/elp/grip1/grip2) from tool_tracking node
     * @param tvec translation vector (of cyl/elp/grip1/grip2) from tool_tracking node
     * @param P Camera projection matrix 3x4
     * @param jac useless output
     * All vertex info loaded offline in load_model_vertices
     */
    void Compute_Silhouette(const std::vector<std::vector<int> > &input_faces,
                            const std::vector<std::vector<int> > &neighbor_faces,
                            const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                            cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                            const cv::Mat &P, cv::OutputArray jac);

    void Compute_Silhouette_UKF(const std::vector<std::vector<int> > &input_faces,
                                           const std::vector<std::vector<int> > &neighbor_faces,
                                           const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                                           cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                                           const cv::Mat &P, std::vector<std::vector<double> > &vertices_vector,
                                           cv::OutputArray jac);

    /**
     * @brief Compute cross product between two vectors
     * @param vec1
     * @param vec2
     * @return their cross product
     */
    cv::Point3d crossProduct(cv::Point3d &vec1, cv::Point3d &vec2);

    /**
     * @brief Compute dot product between two vectors
     * @param vec1
     * @param vec2
     * @return their dot product
     */
    double dotProduct(cv::Point3d &vec1, cv::Point3d &vec2);

    /**
     * @brief normalize an input vector
     * @param vec1
     * @return normalized vector
     */
    cv::Point3d Normalize(cv::Point3d &vec1);

    /**
     * @briefConvert each vertex from inches to meters
     * @param input_vertices
     */
    void ConvertInchtoMeters(std::vector<cv::Point3d> &input_vertices);

    /**
     * @brief convert from glm::vec3 to cv::Point3d
     * @param input_vertices
     * @param out_vertices
     */
    void Convert_glTocv_pts(std::vector<glm::vec3> &input_vertices, std::vector<cv::Point3d> &out_vertices);

    cv::Mat computeSkew(cv::Mat &w);

    /**
     * @brief convert from cv::Mat to cv::Point3d <x,y,z>
     * @param input_Mat
     * @return  not homogenous Point3d representation of input
     */
    cv::Point3d convert_MattoPts(cv::Mat &input_Mat);

    void computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat);

    /**
     * @brief compute the face normal given the three vertices/normals that define it
     * Used to determine if an edge is front/backfacing depending on the dot product of the normal and camera-face vector
     * @param input_v1
     * @param input_v2
     * @param input_v3
     * @param input_n1
     * @param input_n2
     * @param input_n3
     * @return face normal
     */
    cv::Point3d FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                               cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3);

    /**
     * @brief determine how many vertices two faces share
     * if two, they are neighbors
     * @param vec1 face 1
     * @param vec2 face 2
     * @param match_vec vertices and normals in common between vec1, vec2
     * @return number of vertices two faces share
     */
    int Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2, std::vector<int> &match_vec);

    /**
     * Given vectors of faces, vertices, and normals, compute the face normals/centroids
     * Used on initialization
     * @param input_faces
     * @param input_vertices
     * @param input_Vnormal
     * @param face_normals output computed from the vertices and normals that define the face
     * @param face_centroids output computed from the vertices that define the face
     */
    void getFaceInfo(const std::vector<std::vector<int> > &input_faces, const std::vector<cv::Point3d> &input_vertices,
                     const std::vector<cv::Point3d> &input_Vnormal, cv::Mat &face_normals, cv::Mat &face_centroids);

    /**
     * @brief transform input_mat into cam_mats frame
     * @param cam_mat
     * @param input_mat
     * @return
     */
    cv::Mat camTransformMats(cv::Mat &cam_mat, cv::Mat &input_mat);

    void gatherNormals(std::vector< std::vector<double> > &part1_normals, std::vector< std::vector<double> > &part2_normals, std::vector< std::vector<double> > &part3_normals, cv::Mat &tool_points, cv::Mat &tool_normals);

};

#endif