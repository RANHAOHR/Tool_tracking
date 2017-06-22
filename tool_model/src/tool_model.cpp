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
 *   * Neither the name of Case Western Reserve University, nor the names of its
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

#include <ros/ros.h>
#include <boost/random.hpp>

#include <tool_model_lib/tool_model.h>


using cv_projective::reprojectPoint;
using cv_projective::transformPoints;
using namespace std;

boost::mt19937 rng((const uint32_t &) time(0));

ToolModel::ToolModel() {

    ///offset the model's values according to the tool geometry.  values from testing
    offset_body = 0.4560;
    offset_ellipse = offset_body;
    offset_gripper = offset_ellipse+ 0.005;

    //get path to package
    tool_model_pkg = ros::package::getPath("tool_model");

    std::string cylinder = tool_model_pkg + "/tool_parts/cyliner_tense_end_face.obj"; //"/tense_cylinde_2.obj", test_cylinder_3, cyliner_tense_end_face
    std::string ellipse = tool_model_pkg + "/tool_parts/refine_ellipse_3.obj";
    std::string gripper1 = tool_model_pkg + "/tool_parts/gripper2_1.obj";
    std::string gripper2 = tool_model_pkg + "/tool_parts/gripper2_2.obj";

    load_model_vertices(cylinder.c_str(),
                        body_vertices, body_Vnormal, body_faces, body_neighbors);
    load_model_vertices(ellipse.c_str(),
                        ellipse_vertices, ellipse_Vnormal, ellipse_faces, ellipse_neighbors);
    load_model_vertices(gripper1.c_str(), griper1_vertices,
                        griper1_Vnormal, griper1_faces, griper1_neighbors);
    load_model_vertices(gripper2.c_str(), griper2_vertices,
                        griper2_Vnormal, griper2_faces, griper2_neighbors);

    modify_model_(body_vertices, body_Vnormal, body_Vpts, body_Npts, offset_body, body_Vmat, body_Nmat);
    modify_model_(ellipse_vertices, ellipse_Vnormal, ellipse_Vpts, ellipse_Npts, offset_ellipse, ellipse_Vmat,
                  ellipse_Nmat);
    modify_model_(griper1_vertices, griper1_Vnormal, griper1_Vpts, griper1_Npts, offset_gripper, gripper1_Vmat,
                  gripper1_Nmat);
    modify_model_(griper2_vertices, griper2_Vnormal, griper2_Vpts, griper2_Npts, offset_gripper, gripper2_Vmat,
                  gripper2_Nmat);

    getFaceInfo(body_faces, body_Vpts, body_Npts, bodyFace_normal, bodyFace_centroid);
    getFaceInfo(ellipse_faces, ellipse_Vpts, ellipse_Npts, ellipseFace_normal, ellipseFace_centroid);
    getFaceInfo(griper1_faces, griper1_Vpts, griper1_Npts, gripper1Face_normal, gripper1Face_centroid);
    getFaceInfo(griper2_faces, griper2_Vpts, griper2_Npts, gripper2Face_normal, gripper2Face_centroid);

    //get oval normals for UKF
    std::string oval_normal = tool_model_pkg + "/tool_parts/new_less_normal.obj";  //contains only faces with useful normals from refine_ellipse_3.obj file
    load_model_vertices(oval_normal.c_str(),
                        oval_normal_vertices, oval_normal_Vnormal, oval_normal_faces, oval_normal_neighbors);
    modify_model_(oval_normal_vertices, oval_normal_Vnormal, oval_normal_Vpts, oval_normal_Npts, offset_ellipse, oval_normal_Vmat, oval_normal_Nmat);
    getFaceInfo(oval_normal_faces, oval_normal_Vpts, oval_normal_Npts, oval_normalFace_normal, oval_normalFace_centroid);

    srand((unsigned) time(NULL)); //for the random number generator, use only once
};

double ToolModel::randomNumber(double stdev, double mean) {

    boost::normal_distribution<> nd(mean, stdev);
    boost::variate_generator<boost::mt19937 &, boost::normal_distribution<> > var_nor(rng, nd);
    double d = var_nor();

    return d;

};


double ToolModel::randomNum(double min, double max){
    /// srand((unsigned) time( NULL));  //do this in main or constructor
    int N = 999;

    double randN = rand() % (N + 1) / (double) (N + 1);  // a rand number frm 0 to 1
    double res = randN * (max - min) + min;

    return res;
};

void ToolModel::ConvertInchtoMeters(std::vector<cv::Point3d> &input_vertices) {

    int size = (int) input_vertices.size();
    for (int i = 0; i < size; ++i) {
        input_vertices[i].x = input_vertices[i].x * 0.0254;
        input_vertices[i].y = input_vertices[i].y * 0.0254;
        input_vertices[i].z = input_vertices[i].z * 0.0254;
    }
};

void ToolModel::load_model_vertices(const char *path, std::vector<glm::vec3> &out_vertices,
                                    std::vector<glm::vec3> &vertex_normal,
                                    std::vector<std::vector<int> > &out_faces,
                                    std::vector<std::vector<int> > &neighbor_faces) {

    std::vector<unsigned int> vertexIndices, uvIndices, normalIndices;
    std::vector<glm::vec2> temp_uvs;

    std::vector<int> temp_face;
    temp_face.resize(6);  //need three vertex and corresponding normals

    //Open OBJ file
    FILE *file = fopen(path, "r");
    if (file == NULL) {
        printf("Impossible to open the file ! Are you in the right path ?\n");
        return;
    }

    //Read OBJ file for its vertices, textures, normals, and faces
    while (1) {

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.
        // else : parse lineHeader

        //if vertex, get the <x,y,z> coords
        if (strcmp(lineHeader, "v") == 0) {
            glm::vec3 vertex;
            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z);
            //temp_vertices.push_back(vertex);
            out_vertices.push_back(vertex);

        //if vertex texture, get <x,y> dir of texture
        } else if (strcmp(lineHeader, "vt") == 0) {
            glm::vec2 uv;
            fscanf(file, "%f %f\n", &uv.x, &uv.y);
            // cout<<"uv"<<uv.x<<endl;
            temp_uvs.push_back(uv);

        // if vertex normal, get <x,y,z> coords
        } else if (strcmp(lineHeader, "vn") == 0) {
            glm::vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z);
            //temp_normals.push_back(normal);
            vertex_normal.push_back(normal);

        //if face, get its 3 groups of <vertex, normal> and push back the face and the vert/texture/normals separately
        } else if (strcmp(lineHeader, "f") == 0) {

            unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
            int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0],
                                 &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2],
                                 &normalIndex[2]);
            if (matches != 9) {
                ROS_ERROR("File can't be read by our simple parser : ( Try exporting with other options\n");
            }
            vertexIndices.push_back(vertexIndex[0]);
            vertexIndices.push_back(vertexIndex[1]);
            vertexIndices.push_back(vertexIndex[2]);
            uvIndices.push_back(uvIndex[0]);
            uvIndices.push_back(uvIndex[1]);
            uvIndices.push_back(uvIndex[2]);
            normalIndices.push_back(normalIndex[0]);
            normalIndices.push_back(normalIndex[1]);
            normalIndices.push_back(normalIndex[2]);

            temp_face[0] = vertexIndex[0] - 1;
            temp_face[1] = vertexIndex[1] - 1;
            temp_face[2] = vertexIndex[2] - 1;
            temp_face[3] = normalIndex[0] - 1;
            temp_face[4] = normalIndex[1] - 1;
            temp_face[5] = normalIndex[2] - 1;
            out_faces.push_back(temp_face);
        }
    }

    //Find neighbor faces from out_faces
    neighbor_faces.resize(out_faces.size());
    std::vector<int> temp_vec;

    for (int i = 0; i < neighbor_faces.size(); ++i) {
        for (int j = 0; j < out_faces.size(); ++j) {
            if (j != i) {  //no repeats
                int match = Compare_vertex(out_faces[i], out_faces[j], temp_vec);
                if (match == 2) // face i and face j share an edge
                {
                    //ROS_INFO_STREAM("SIZE FO THE TEMP VEC: " << temp_vec.size() );
                    neighbor_faces[i].push_back(j); // mark the neighbor face index
                    neighbor_faces[i].push_back(temp_vec[0]);  //first vertex
                    neighbor_faces[i].push_back(temp_vec[1]);  //corresponding normal
                    neighbor_faces[i].push_back(temp_vec[2]);  //second vertex
                    neighbor_faces[i].push_back(temp_vec[3]);  //corresponding normal
                    //ROS_INFO_STREAM("neighbor_faces[i]" << neighbor_faces[i].size());
                }
                temp_vec.clear();
            }
        }
    }

    printf("loaded file %s successfully.\n", path);
    /*debug for faces*/
    // for (int i = 0; i < 10; ++i)
    // {
    //     ROS_INFO_STREAM("face: " << i );
    //     ROS_INFO_STREAM("vertex: " << out_faces[i][0] << " " << out_faces[i][1] << " " << out_faces[i][2] << " normal: " << out_faces[i][3] << " " << out_faces[i][4] << " " << out_faces[i][5] );
    // }

    // for (int i = 0; i < 10; ++i)
    // {
    //     ROS_INFO_STREAM("vertex: " << i );
    //     ROS_INFO_STREAM("x: " << out_vertices[i].x << " y: " << out_vertices[i].y << " z: " << out_vertices[i].z );
    // }

    // /*screen unneccessary faces*/
    // for (int i = 0; i < all_faces.size(); ++i)
    // {
    //     neighbor_num = (neighbor_faces[i].size())/3;
    //     if ( neighbor_num > 0){

    //         out_faces.push_back(all_faces[i]);
    //     }
    // }
};

void ToolModel::Convert_glTocv_pts(std::vector<glm::vec3> &input_vertices, std::vector<cv::Point3d> &out_vertices) {
    unsigned long vsize = input_vertices.size();
    out_vertices.resize(vsize);
    for (int i = 0; i < vsize; ++i) {
        out_vertices[i].x = input_vertices[i].x;
        out_vertices[i].y = input_vertices[i].y;
        out_vertices[i].z = input_vertices[i].z;
    }
};


cv::Mat ToolModel::camTransformMats(cv::Mat &cam_mat, cv::Mat &input_mat) {
    cv::Mat output_mat = cam_mat * input_mat; //transform the obj to camera frames, g_CT
    return output_mat;
};

void ToolModel::Compute_Silhouette(const std::vector<std::vector<int> > &input_faces,
                                   const std::vector<std::vector<int> > &neighbor_faces,
                                   const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                                   cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                                   const cv::Mat &P, cv::OutputArray jac) {

//    cv::Mat adjoint_mat = (cv::Mat_<double>(4,4) << -1,0,0,0,
//    0,0,1,0,
//    0,1,0,0,
//    0,0,0,1); //tip

//    cv::Mat adjoint_mat = (cv::Mat_<double>(4,4) << 0,0,1,0,
//            1,0,0,0,
//            0,1,0,0,
//            0,0,0,1);
//
//
//    cv::Mat temp_input_Vmat = adjoint_mat * input_Vmat;
//    cv::Mat temp_input_Nmat = adjoint_mat * input_Nmat;

    //Transform vertices/normals from offline pose to pose in camera frame
    cv::Mat new_Vertices = transformPoints(input_Vmat, rvec, tvec); //transform points from offline pose to real pose corresponding to rot/trans vecs
    new_Vertices = camTransformMats(CamMat, new_Vertices); //transform points to camera frame
    cv::Mat new_Normals = transformPoints(input_Nmat, rvec, tvec);  //transform normals from offline pose to real pose corresponding to rot/trans vecs
    new_Normals = camTransformMats(CamMat, new_Normals); //transform normals to camera frame

    unsigned long neighbor_num = 0;
    cv::Mat temp(4, 1, CV_64FC1);
    cv::Mat ept_1(4, 1, CV_64FC1); // edge point 1
    cv::Mat ept_2(4, 1, CV_64FC1); // edge point 2

    for (int i = 0; i < input_faces.size(); ++i) {
        neighbor_num = (neighbor_faces[i].size()) / 5;  //total number of neighbors to the given face

        //If i'th face has neighbors, analyze it
        if (neighbor_num > 0) {
            //store indeices of the vertices/normals which define face i
            int v1 = input_faces[i][0];
            int v2 = input_faces[i][1];
            int v3 = input_faces[i][2];
            int n1 = input_faces[i][3];
            int n2 = input_faces[i][4];
            int n3 = input_faces[i][5];

            //convert vertices and normals to cv::Point3d using the temp matrix
            new_Vertices.col(v1).copyTo(temp.col(0));
            cv::Point3d pt1 = convert_MattoPts(temp);
            new_Vertices.col(v2).copyTo(temp.col(0));
            cv::Point3d pt2 = convert_MattoPts(temp);
            new_Vertices.col(v3).copyTo(temp.col(0));
            cv::Point3d pt3 = convert_MattoPts(temp);
            new_Normals.col(n1).copyTo(temp.col(0));
            cv::Point3d vn1 = convert_MattoPts(temp);
            new_Normals.col(n2).copyTo(temp.col(0));
            cv::Point3d vn2 = convert_MattoPts(temp);
            new_Normals.col(n3).copyTo(temp.col(0));
            cv::Point3d vn3 = convert_MattoPts(temp);

            //Get the face's normal vector and vector from camera to face
            cv::Point3d fnormal = FindFaceNormal(pt1, pt2, pt3, vn1, vn2, vn3);
            cv::Point3d face_point_i = pt1 + pt2 + pt3; // find center of face WRT camera frame
            face_point_i.x = face_point_i.x / 3;
            face_point_i.y = face_point_i.y / 3;
            face_point_i.z = face_point_i.z / 3;

            double isfront_i = dotProduct(fnormal, face_point_i); //if >0, it is a front face
            if(isfront_i < 0.000){ //only draw edge if its a front face
                for (int neighbor_count = 0; neighbor_count <
                                             neighbor_num; ++neighbor_count) {  //iterate through all neighbor faces
                    //get current neighbors vertices/normals
                    int j = 5 * neighbor_count; //neighbor count= number of neighbors, but each neighbor has 5 fields, so it couldnt be used as an index
                    int v1_ = input_faces[neighbor_faces[i][j]][0];
                    int v2_ = input_faces[neighbor_faces[i][j]][1];
                    int v3_ = input_faces[neighbor_faces[i][j]][2];
                    int n1_ = input_faces[neighbor_faces[i][j]][3];
                    int n2_ = input_faces[neighbor_faces[i][j]][4];
                    int n3_ = input_faces[neighbor_faces[i][j]][5];

                    //convert to cv::Point3d
                    new_Vertices.col(v1_).copyTo(temp.col(0));
                    cv::Point3d pt1_ = convert_MattoPts(temp);
                    new_Vertices.col(v2_).copyTo(temp.col(0));
                    cv::Point3d pt2_ = convert_MattoPts(temp);
                    new_Vertices.col(v3_).copyTo(temp.col(0));
                    cv::Point3d pt3_ = convert_MattoPts(temp);
                    new_Normals.col(n1_).copyTo(temp.col(0));
                    cv::Point3d vn1_ = convert_MattoPts(temp);
                    new_Normals.col(n2_).copyTo(temp.col(0));
                    cv::Point3d vn2_ = convert_MattoPts(temp);
                    new_Normals.col(n3_).copyTo(temp.col(0));
                    cv::Point3d vn3_ = convert_MattoPts(temp);

                    //Get the face's normal vector and vector from camera to face
                    cv::Point3d fnormal_n = FindFaceNormal(pt1_, pt2_, pt3_, vn1_, vn2_, vn3_);
                    cv::Point3d face_point_j = pt1_ + pt2_ + pt3_;
                    face_point_j.x = face_point_j.x / 3;
                    face_point_j.y = face_point_j.y / 3;
                    face_point_j.z = face_point_j.z / 3;

                    double isfront_j = dotProduct(fnormal_n, face_point_j); //determine if neighbor face is front or back

                    if (isfront_i * isfront_j < 0.0) // if one is front and the other is back,
                    { //project the edge and draw it

                        //project edge under camera frame
                        new_Vertices.col(neighbor_faces[i][j + 1]).copyTo(ept_1);
                        new_Vertices.col(neighbor_faces[i][j + 3]).copyTo(ept_2);
                        cv::Point2d prjpt_1 = reproject(ept_1, P);
                        cv::Point2d prjpt_2 = reproject(ept_2, P);

                        if(((prjpt_1.x <= 0 && prjpt_2.x <= 0) || (prjpt_1.x >= 640 && prjpt_2.x >= 640)) || ((prjpt_1.y <= 0 && prjpt_2.y <= 0) || (prjpt_1.y >= 480 && prjpt_2.y >= 480))){
                            //this is not suppose to be in the image
                        }else { //draw edge line
                            cv::line(image, prjpt_1, prjpt_2, cv::Scalar(255, 255, 255), 1, 8, 0);
                        }
                    }
                }
            }
        }
    }
};

void ToolModel::Compute_Silhouette_UKF(const std::vector<std::vector<int> > &input_faces,
                                   const std::vector<std::vector<int> > &neighbor_faces,
                                   const cv::Mat &input_Vmat, const cv::Mat &input_Nmat,
                                   cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec,
                                   const cv::Mat &P, std::vector<std::vector<double> > &vertices_vector, cv::OutputArray jac){

    cv::Mat new_Vertices = transformPoints(input_Vmat, rvec, tvec);

    new_Vertices = camTransformMats(CamMat, new_Vertices); //transform every point under camera frame

    cv::Mat new_Normals = transformPoints(input_Nmat, rvec, tvec);
    new_Normals = camTransformMats(CamMat, new_Normals); //transform every surface normal under camera frame

    unsigned long neighbor_num = 0;
    cv::Mat temp(4, 1, CV_64FC1);

    cv::Mat ept_1(4, 1, CV_64FC1);
    cv::Mat ept_2(4, 1, CV_64FC1);

    for (int i = 0; i < input_faces.size(); ++i) {
        neighbor_num = (neighbor_faces[i].size()) / 5;  //each neighbor has two vertices, used to be 3 when normals

        if (neighbor_num > 0) {
            int v1 = input_faces[i][0];
            int v2 = input_faces[i][1];
            int v3 = input_faces[i][2];
            int n1 = input_faces[i][3];
            int n2 = input_faces[i][4];
            int n3 = input_faces[i][5];

            new_Vertices.col(v1).copyTo(temp.col(0));
            cv::Point3d pt1 = convert_MattoPts(temp);
            new_Vertices.col(v2).copyTo(temp.col(0));
            cv::Point3d pt2 = convert_MattoPts(temp);
            new_Vertices.col(v3).copyTo(temp.col(0));
            cv::Point3d pt3 = convert_MattoPts(temp);

            new_Normals.col(n1).copyTo(temp.col(0));
            cv::Point3d vn1 = convert_MattoPts(temp);
            new_Normals.col(n2).copyTo(temp.col(0));
            cv::Point3d vn2 = convert_MattoPts(temp);
            new_Normals.col(n3).copyTo(temp.col(0));
            cv::Point3d vn3 = convert_MattoPts(temp);

            cv::Point3d fnormal = FindFaceNormal(pt1, pt2, pt3, vn1, vn2, vn3); //knowing the direction and normalized
            cv::Point3d face_point_i = pt1 + pt2 + pt3;
            face_point_i.x = face_point_i.x / 3;
            face_point_i.y = face_point_i.y / 3;
            face_point_i.z = face_point_i.z / 3;

            double isfront_i = dotProduct(fnormal, face_point_i);
            if(isfront_i < 0.000){ //first need to find the front facing face
                for (int neighbor_count = 0; neighbor_count <
                                             neighbor_num; ++neighbor_count) {  //notice: cannot use J here, since the last j will not be counted
                    int j = 5 * neighbor_count;
                    int v1_ = input_faces[neighbor_faces[i][j]][0];
                    int v2_ = input_faces[neighbor_faces[i][j]][1];
                    int v3_ = input_faces[neighbor_faces[i][j]][2];

                    int n1_ = input_faces[neighbor_faces[i][j]][3];
                    int n2_ = input_faces[neighbor_faces[i][j]][4];
                    int n3_ = input_faces[neighbor_faces[i][j]][5];


                    new_Vertices.col(v1_).copyTo(temp.col(0));
                    cv::Point3d pt1_ = convert_MattoPts(temp);
                    new_Vertices.col(v2_).copyTo(temp.col(0));
                    cv::Point3d pt2_ = convert_MattoPts(temp);
                    new_Vertices.col(v3_).copyTo(temp.col(0));
                    cv::Point3d pt3_ = convert_MattoPts(temp);

                    new_Normals.col(n1_).copyTo(temp.col(0));
                    cv::Point3d vn1_ = convert_MattoPts(temp);
                    new_Normals.col(n2_).copyTo(temp.col(0));
                    cv::Point3d vn2_ = convert_MattoPts(temp);
                    new_Normals.col(n3_).copyTo(temp.col(0));
                    cv::Point3d vn3_ = convert_MattoPts(temp);

                    cv::Point3d fnormal_n = FindFaceNormal(pt1_, pt2_, pt3_, vn1_, vn2_, vn3_);

                    cv::Point3d face_point_j = pt1_ + pt2_ + pt3_;
                    face_point_j.x = face_point_j.x / 3;
                    face_point_j.y = face_point_j.y / 3;
                    face_point_j.z = face_point_j.z / 3;

                    double isfront_j = dotProduct(fnormal_n, face_point_j);

                    if (isfront_i * isfront_j < 0.0) // one is front, another is back
                    {   /*finish finding, drawing the image*/
                        new_Vertices.col(neighbor_faces[i][j + 1]).copyTo(ept_1);  //under camera frames
                        new_Vertices.col(neighbor_faces[i][j + 3]).copyTo(ept_2);

                        cv::Point2d prjpt_1 = reproject(ept_1, P);
                        cv::Point2d prjpt_2 = reproject(ept_2, P);

                        if(((prjpt_1.x <= 0 && prjpt_2.x <= 0) || (prjpt_1.x >= 640 && prjpt_2.x >= 640)) || ((prjpt_1.y <= 0 && prjpt_2.y <= 0) || (prjpt_1.y >= 480 && prjpt_2.y >= 480))){
                            //this is not suppose to be in the image
                        }else{

                            cv::line(image, prjpt_1, prjpt_2, cv::Scalar(255, 255, 255), 1, 8, 0);
                            /**** get new vertex ****/
                            cv::Point2d mid_vertex = prjpt_1 + prjpt_2;
                            mid_vertex.x = mid_vertex.x / 2;
                            mid_vertex.y = mid_vertex.y / 2;

                            double delta_y = prjpt_2.y - prjpt_1.y;
                            double delta_x = prjpt_2.x - prjpt_1.x;

                            double k  = delta_y / delta_x;
                            cv::Mat temp_normal(1,2,CV_64FC1);
                            temp_normal.at<double>(0,0) = -1.0 * k;   //n_x
                            temp_normal.at<double>(0,1) = 1.0 ;     //n_y

                            cv::Mat Vnormal_1 = new_Normals.col(neighbor_faces[i][j + 2]).clone();
                            cv::Mat Vnormal_2 = new_Normals.col(neighbor_faces[i][j + 4]).clone();

                            cv::Mat mid_normal(1,2,CV_64FC1);
                            mid_normal.at<double>(0,0) = 0.5 * (Vnormal_1.at<double>(0,0) + Vnormal_2.at<double>(0,0));
                            mid_normal.at<double>(0,1) = 0.5 * (Vnormal_1.at<double>(1,0) + Vnormal_2.at<double>(1,0));

                            double dot_normal = mid_normal.dot(temp_normal);
                            if(dot_normal < 0.0){
                                temp_normal = -1.0 * temp_normal;   //flip?
                            }

                            /**get measurement points for UKF**/
                            std::vector<double> vertex_vector;
                            vertex_vector.resize(4); // vertices, normals

                            if(mid_vertex.x >= 10 && mid_vertex.x <=640 && mid_vertex.y >= 0 && mid_vertex.y <= 480){
                                vertex_vector[0] = mid_vertex.x;
                                vertex_vector[1] = mid_vertex.y;
                                vertex_vector[2] = temp_normal.at<double>(0,0);
                                vertex_vector[3] = temp_normal.at<double>(0,1);
                                vertices_vector.push_back(vertex_vector);
                            }

                        }

                    }
                }
            }
        }
    }
};

cv::Point3d ToolModel::convert_MattoPts(cv::Mat &input_Mat) {
    cv::Point3d output_point;
    output_point.x = input_Mat.at<double>(0, 0);
    output_point.y = input_Mat.at<double>(1, 0);
    output_point.z = input_Mat.at<double>(2, 0);
    return output_point;
};

void ToolModel::getFaceInfo(const std::vector<std::vector<int> > &input_faces,
                            const std::vector<cv::Point3d> &input_vertices,
                            const std::vector<cv::Point3d> &input_Vnormal, cv::Mat &face_normals,
                            cv::Mat &face_centroids) {

    int face_size = input_faces.size();
    face_normals = cv::Mat(4, face_size, CV_64FC1);
    face_centroids = cv::Mat(4, face_size, CV_64FC1);

    for (int i = 0; i < face_size; ++i) {
        //Get indices of v1,v2,v3,n1,n2,n3 WRT vertices/normals lists
        int v1 = input_faces[i][0];
        int v2 = input_faces[i][1];
        int v3 = input_faces[i][2];
        int n1 = input_faces[i][3];
        int n2 = input_faces[i][4];
        int n3 = input_faces[i][5];

        //get the values for v1,v2,v3,n1,n2,n3 from vertices/normals lists
        cv::Point3d pt1 = input_vertices[v1];
        cv::Point3d pt2 = input_vertices[v2];
        cv::Point3d pt3 = input_vertices[v3];
        cv::Point3d normal1 = input_Vnormal[n1];
        cv::Point3d normal2 = input_Vnormal[n2];
        cv::Point3d normal3 = input_Vnormal[n3];

        //Compute face normal given v1,v2,v3,n1,n2,n3
        cv::Point3d fnormal = FindFaceNormal(pt1, pt2, pt3, normal1, normal2, normal3); //knowing the direction and normalized\
        //Compute face centroid given face normal
        face_normals.at<double>(0, i) = fnormal.x;
        face_normals.at<double>(1, i) = fnormal.y;
        face_normals.at<double>(2, i) = fnormal.z;
        face_normals.at<double>(3, i) = 0;
        cv::Point3d face_point = pt1 + pt2 + pt3;
        face_point.x = face_point.x / 3.000000;
        face_point.y = face_point.y / 3.000000;
        face_point.z = face_point.z / 3.000000;
        face_point = Normalize(face_point);
        face_centroids.at<double>(0, i) = face_point.x;
        face_centroids.at<double>(1, i) = face_point.y;
        face_centroids.at<double>(2, i) = face_point.z;
        face_centroids.at<double>(3, i) = 1;
    }
};

cv::Point3d ToolModel::crossProduct(cv::Point3d &vec1, cv::Point3d &vec2) {
    cv::Point3d res_vec;
    res_vec.x = vec1.y * vec2.z - vec1.z * vec2.y;
    res_vec.y = vec1.z * vec2.x - vec1.x * vec2.z;
    res_vec.z = vec1.x * vec2.y - vec1.y * vec2.x;
    return res_vec;
};

double ToolModel::dotProduct(cv::Point3d &vec1, cv::Point3d &vec2) {
    double dot_res;
    dot_res = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;
    return dot_res;
};

cv::Point3d ToolModel::Normalize(cv::Point3d &vec1) {
    cv::Point3d norm_res;

    double norm = vec1.x * vec1.x + vec1.y * vec1.y + vec1.z * vec1.z;
    if (norm == 0.000000) {
        //ROS_ERROR("Trying to normalize a zero vector!");
    } else {
        norm = pow(norm, 0.5);

        norm_res.x = vec1.x / norm;
        norm_res.y = vec1.y / norm;
        norm_res.z = vec1.z / norm;

        return norm_res;
    }

};

cv::Point3d ToolModel::FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                                      cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3) {
    cv::Point3d temp_v12; //v1 rel to v2
    cv::Point3d temp_v13; //v1 rel to v3

    temp_v12.x = input_v1.x - input_v2.x;
    temp_v12.y = input_v1.y - input_v2.y;
    temp_v12.z = input_v1.z - input_v2.z;

    temp_v13.x = input_v1.x - input_v3.x;
    temp_v13.y = input_v1.y - input_v3.y;
    temp_v13.z = input_v1.z - input_v3.z;

    cv::Point3d res = crossProduct(temp_v12, temp_v13); //output cross product of v12 v13

    //determine sign of output
    double outward_normal_1 = dotProduct(res, input_n1);
    double outward_normal_2 = dotProduct(res, input_n2);
    double outward_normal_3 = dotProduct(res, input_n3);
    if ((outward_normal_1 < 0) || (outward_normal_2 < 0) || (outward_normal_3 < 0)) {
        res = -res;
    }
    return res;
};

int ToolModel::Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2, std::vector<int> &match_vec) {
    int match_count = 0;
    if (vec1.size() != vec2.size())  ///face vectors
    {
        printf("Two vectors are not in the same size \n");
    }
    else {
        for (int j = 0; j < 3; ++j) {
            if (vec1[0] == vec2[j]) {
                match_count += 1;
                match_vec.push_back(vec1[0]);   //vertex
                match_vec.push_back(vec1[3]);  // corresponding vertex normal
            }
        }

        for (int j = 0; j < 3; ++j) {
            if (vec1[1] == vec2[j]) {
                match_count += 1;
                match_vec.push_back(vec1[1]);
                match_vec.push_back(vec1[4]);  // corresponding vertex normal
            }
        }

        for (int j = 0; j < 3; ++j) {
            if (vec1[2] == vec2[j]) {
                match_count += 1;
                match_vec.push_back(vec1[2]);
                match_vec.push_back(vec1[5]);  // corresponding vertex normal
            }
        }
    }

    return match_count;
};

void ToolModel::modify_model_(std::vector<glm::vec3> &input_vertices, std::vector<glm::vec3> &input_Vnormal,
                              std::vector<cv::Point3d> &input_Vpts, std::vector<cv::Point3d> &input_Npts,
                              double &offset, cv::Mat &input_Vmat, cv::Mat &input_Nmat) {

    //convert vertices from glm to cv::Point3d
    Convert_glTocv_pts(input_vertices, input_Vpts);
    ConvertInchtoMeters(input_Vpts);

    //incorporate offset from tool origin to part origin
    int size = input_Vpts.size();
    for (int i = 0; i < size; ++i) {
        input_Vpts[i].y = input_Vpts[i].y - offset;
    }

    //convert normals from gl to cv::Point3d
    Convert_glTocv_pts(input_Vnormal, input_Npts); //not using homogeneous for v weight now
    ConvertInchtoMeters(input_Npts);

    //convert vertices from cv::Point3d to cv::mat
    input_Vmat = cv::Mat(4, size, CV_64FC1);
    for (int i = 0; i < size; ++i) {
        input_Vmat.at<double>(0, i) = input_Vpts[i].x;
        input_Vmat.at<double>(1, i) = input_Vpts[i].y;
        input_Vmat.at<double>(2, i) = input_Vpts[i].z;
        input_Vmat.at<double>(3, i) = 1;
    }

    //convert normals from cv::Point3d to cv::mat
    int Nsize = input_Npts.size();
    input_Nmat = cv::Mat(4, Nsize, CV_64FC1);
    for (int i = 0; i < Nsize; ++i) {
        input_Nmat.at<double>(0, i) = input_Npts[i].x;
        input_Nmat.at<double>(1, i) = input_Npts[i].y;
        input_Nmat.at<double>(2, i) = input_Npts[i].z;
        input_Nmat.at<double>(3, i) = 0;
    }
};

ToolModel::toolModel ToolModel::setRandomConfig(const toolModel &seeds, const double &theta_cylinder, const double &theta_oval, const double &theta_open, double &step){

    toolModel newTool = seeds;   //Output toolModel

    //Perturb tvec_cyl/rvec_cyl by random noise
    double dev = randomNumber(step, 0); //gaussian noise
    newTool.tvec_cyl(0) = seeds.tvec_cyl(0) + dev;
    dev = randomNumber(step, 0);
    newTool.tvec_cyl(1) = seeds.tvec_cyl(1) + dev;
    dev = randomNumber(step, 0);
    newTool.tvec_cyl(2) = seeds.tvec_cyl(2)+ dev;
    dev = randomNumber(step, 0);
    newTool.rvec_cyl(0) = seeds.rvec_cyl(0)+ dev;
    dev = randomNumber(step, 0);
    newTool.rvec_cyl(1) = seeds.rvec_cyl(1)+ dev;
    dev = randomNumber(step, 0);
    newTool.rvec_cyl(2) = seeds.rvec_cyl(2)+ dev;

    //Perturb joints 5,6,7 by random noise
    //Positive = CW if angles are pretty goog, then give small noises
    double theta_1 = theta_cylinder + randomNumber(0.0001, 0);   // tool rotation
    double theta_grip_1 = theta_oval + randomNumber(0.0001, 0); // oval rotation
    double theta_grip_2 = theta_open + randomNumber(0.0001, 0);

    //Generate a toolModel representation of joints 1-7
    computeEllipsePose(newTool, theta_1, theta_grip_1, theta_grip_2);

    return newTool;
};

ToolModel::toolModel ToolModel::gaussianSampling(const toolModel &max_pose, double &step){

    toolModel gaussianTool;  //new sample

    double dev_pos = randomNumber(step, 0);
    gaussianTool.tvec_cyl(0) = max_pose.tvec_cyl(0)+ dev_pos;

    dev_pos = randomNumber(step, 0);
    gaussianTool.tvec_cyl(1) = max_pose.tvec_cyl(1)+ dev_pos;

    dev_pos = randomNumber(step, 0);
    gaussianTool.tvec_cyl(2) = max_pose.tvec_cyl(2)+ dev_pos;

    double dev_ori = randomNumber(step, 0);
    gaussianTool.rvec_cyl(0) = max_pose.rvec_cyl(0)+ dev_ori;

    dev_ori = randomNumber(step, 0);
    gaussianTool.rvec_cyl(1) = max_pose.rvec_cyl(1)+ dev_ori;

    dev_ori = randomNumber(step, 0);
    gaussianTool.rvec_cyl(2) = max_pose.rvec_cyl(2)+ dev_ori;

    //perturb joint angles by random noise, positive = CW
    double theta_ = randomNumber(step, 0);    //-90,90
    double theta_grip_1 = randomNumber(step, 0);
    double theta_grip_2 = randomNumber(step, 0);

    computeRandomPose(max_pose, gaussianTool, theta_, theta_grip_1, theta_grip_2);

    return gaussianTool;

};

void ToolModel::computeRandomPose(const toolModel &seed_pose, toolModel &inputModel, const double &theta_tool, const double &theta_grip_1,
                                  const double &theta_grip_2) {

    ///take cylinder part as the origin
    cv::Mat I = cv::Mat::eye(3, 3, CV_64FC1);

    ////////////////////////PART 1: computations for ellipse //////////////////////
    //Part 1.a: translation component
    //Compute ellipse origin <x,y,z,1>
    cv::Mat q_ellipse_ = cv::Mat(4, 1, CV_64FC1);//values filled in from trial and error in gazebo
    q_ellipse_.at<double>(0, 0) = 0;
    q_ellipse_.at<double>(1, 0) = 0.0036; // //offset_ellipse - offset_body; pointing to the gripper tip
    q_ellipse_.at<double>(2, 0) = 0;
    q_ellipse_.at<double>(3, 0) = 1;

    //transform the ellipse coord according to cylinder pose
    cv::Mat q_temp(4, 1, CV_64FC1);
    q_temp = transformPoints(q_ellipse_, cv::Mat(inputModel.rvec_cyl),
                             cv::Mat(inputModel.tvec_cyl));

    //Update output field tvec_elp
    inputModel.tvec_elp(0) = q_temp.at<double>(0, 0);
    inputModel.tvec_elp(1) = q_temp.at<double>(1, 0);
    inputModel.tvec_elp(2) = q_temp.at<double>(2, 0);

    /*********** computations for oval kinematics using best particle pose **********/
    //Part 1.b: rotation component
    cv::Mat rot_ellipse(3,3,CV_64FC1);
    cv::Rodrigues(seed_pose.rvec_elp, rot_ellipse);

    double cos_theta = cos(theta_tool);
    double sin_theta = sin(theta_tool);

    cv::Mat g_ellipse = (cv::Mat_<double>(3,3) << cos_theta, -sin_theta, 0,
            sin_theta, cos_theta, 0,
            0,0, 1);

    cv::Mat rot_new = g_ellipse * rot_ellipse;
    cv::Mat temp_vec(3,1,CV_64FC1);
    cv::Rodrigues(rot_new, inputModel.rvec_elp);


    /////////////////////////////PART 2: computations for gripper /////////////////////////
    //Part 2.a: gripper 1
    cv::Mat test_gripper(3, 1, CV_64FC1);
    test_gripper.at<double>(0, 0) = 0;
    test_gripper.at<double>(1, 0) = offset_gripper - offset_ellipse;
    test_gripper.at<double>(2, 0) = 0;

    cv::Mat rot_elp(3, 3, CV_64FC1);
    cv::Rodrigues(inputModel.rvec_elp, rot_elp);  // get rotation mat of the ellipse

    cv::Mat q_rot(3, 1, CV_64FC1);
    q_rot = rot_elp * test_gripper;

    inputModel.tvec_grip1(0) = q_rot.at<double>(0, 0) + inputModel.tvec_elp(0);
    inputModel.tvec_grip1(1) = q_rot.at<double>(1, 0) + inputModel.tvec_elp(1);
    inputModel.tvec_grip1(2) = q_rot.at<double>(2, 0) + inputModel.tvec_elp(2);

    //update gripper 2 translation component
    inputModel.tvec_grip2(0) = inputModel.tvec_grip1(0);
    inputModel.tvec_grip2(1) = inputModel.tvec_grip1(1);
    inputModel.tvec_grip2(2) = inputModel.tvec_grip1(2);

    //Update rotation component
    double grip_1_delta = theta_grip_1 - theta_grip_2;
    double grip_2_delta = theta_grip_1 + theta_grip_2;

    cos_theta = cos(grip_1_delta);
    sin_theta = sin(-grip_1_delta);

    cv::Mat gripper_1_ = (cv::Mat_<double>(3,3) << 1, 0, 0,
            0,cos_theta,-sin_theta,
            0, sin_theta, cos_theta);

    cv::Mat rot_grip_1(3,3,CV_64FC1);
    cv::Rodrigues(seed_pose.rvec_grip1, rot_grip_1);
    rot_grip_1 = gripper_1_ * rot_grip_1;
    cv::Rodrigues(rot_grip_1, inputModel.rvec_grip1 );

    // PART 2.b: gripper 2//
    //Update rotation component
    cos_theta = cos(grip_2_delta);
    sin_theta = sin(-grip_2_delta);

    cv::Mat gripper_2_ = (cv::Mat_<double>(3,3) << 1, 0, 0,
            0,cos_theta,-sin_theta,
            0, sin_theta, cos_theta);

    cv::Mat rot_grip_2(3,3,CV_64FC1);
    cv::Rodrigues(seed_pose.rvec_grip2, rot_grip_2);
    rot_grip_2 = gripper_2_ * rot_grip_2;
    cv::Rodrigues(rot_grip_2, inputModel.rvec_grip2 );
};

void ToolModel::computeEllipsePose(toolModel &inputModel, const double &theta_ellipse, const double &theta_grip_1,
                                   const double &theta_grip_2) {

    //take cylinder as origin
    cv::Mat I = cv::Mat::eye(3, 3, CV_64FC1);

    ////////////////////////PART 1: computations for ellipse //////////////////////
    //Part 1.a: translation component
    //Compute ellipse origin <x,y,z,1>
    cv::Mat q_ellipse_(4, 1, CV_64FC1); //values filled in from trial and error in gazebo
    q_ellipse_.at<double>(0, 0) = 0;
    q_ellipse_.at<double>(1, 0) = 0.0036;//(offset_ellipse - offset_body);
    q_ellipse_.at<double>(2, 0) = 0;
    q_ellipse_.at<double>(3, 0) = 1;

    //transform the ellipse coord according to cylinder pose
    cv::Mat q_temp(4, 1, CV_64FC1);
    q_temp = transformPoints(q_ellipse_, cv::Mat(inputModel.rvec_cyl),
                             cv::Mat(inputModel.tvec_cyl));

    //Update output field tvec_elp
    inputModel.tvec_elp(0) = q_temp.at<double>(0, 0);
    inputModel.tvec_elp(1) = q_temp.at<double>(1, 0);
    inputModel.tvec_elp(2) = q_temp.at<double>(2, 0);

    //Part 1.b: Rotation component
    cv::Mat rot_ellipse(3,3,CV_64FC1);
    cv::Rodrigues(inputModel.rvec_cyl, rot_ellipse);

    double cos_theta = cos(theta_ellipse);
    double sin_theta = sin(theta_ellipse);

    cv::Mat g_ellipse = (cv::Mat_<double>(3,3) << cos_theta, -sin_theta, 0,
            sin_theta, cos_theta, 0,
            0,0, 1);

    cv::Mat rot_new =  rot_ellipse * g_ellipse;
    cv::Mat temp_vec(3,1,CV_64FC1);
    cv::Rodrigues(rot_new, inputModel.rvec_elp);


    /////////////////////////////PART 2: computations for gripper /////////////////////////
    //Part 2.a: gripper 1
    cv::Mat test_gripper(3, 1, CV_64FC1);
    test_gripper.at<double>(0, 0) = 0;
    test_gripper.at<double>(1, 0) = 0.006;//close to offset_gripper - offset_ellipse;
    test_gripper.at<double>(2, 0) = 0;

    cv::Mat rot_elp(3, 3, CV_64FC1);
    cv::Rodrigues(inputModel.rvec_elp, rot_elp);  // get rotation mat of the ellipse

    cv::Mat q_rot(3, 1, CV_64FC1);
    q_rot = rot_elp * test_gripper;

    inputModel.tvec_grip1(0) = q_rot.at<double>(0, 0) + inputModel.tvec_elp(0);
    inputModel.tvec_grip1(1) = q_rot.at<double>(1, 0) + inputModel.tvec_elp(1);
    inputModel.tvec_grip1(2) = q_rot.at<double>(2, 0) + inputModel.tvec_elp(2);

    //Update rotation component
    double grip_1_delta = theta_grip_1 - theta_grip_2;
    double grip_2_delta = theta_grip_1 + theta_grip_2;

    cos_theta = cos(grip_1_delta);
    sin_theta = sin(-grip_1_delta);

    cv::Mat gripper_1_ = (cv::Mat_<double>(3,3) << 1, 0, 0,
            0,cos_theta,-sin_theta,
            0, sin_theta, cos_theta);

    cv::Mat rot_grip_1 = rot_elp * gripper_1_ ;
    cv::Rodrigues(rot_grip_1, inputModel.rvec_grip1);

    // PART 2.b: gripper 2//
    //Update translation component
    inputModel.tvec_grip2(0) = inputModel.tvec_grip1(0);
    inputModel.tvec_grip2(1) = inputModel.tvec_grip1(1);
    inputModel.tvec_grip2(2) = inputModel.tvec_grip1(2);

    cos_theta = cos(grip_2_delta);
    sin_theta = sin(-grip_2_delta);

    cv::Mat gripper_2_ = (cv::Mat_<double>(3,3) << 1, 0, 0,
            0,cos_theta,-sin_theta,
            0, sin_theta, cos_theta);

    //Update rotation component
    cv::Mat rot_grip_2 = rot_elp * gripper_2_;
    cv::Rodrigues(rot_grip_2, inputModel.rvec_grip2);
};

cv::Mat ToolModel::computeSkew(cv::Mat &w) {
    cv::Mat skew(3, 3, CV_64FC1);
    skew.at<double>(0, 0) = 0;
    skew.at<double>(1, 0) = w.at<double>(2, 0);
    skew.at<double>(2, 0) = -w.at<double>(1, 0);
    skew.at<double>(0, 1) = -w.at<double>(2, 0);
    skew.at<double>(1, 1) = 0;
    skew.at<double>(2, 1) = w.at<double>(0, 0);
    skew.at<double>(0, 2) = w.at<double>(1, 0);
    skew.at<double>(1, 2) = -w.at<double>(0, 0);
    skew.at<double>(2, 2) = 0;

    return skew;

};

void ToolModel::computeInvSE(const cv::Mat &inputMat, cv::Mat &outputMat){

    outputMat = cv::Mat::eye(4,4,CV_64F);

    cv::Mat R = inputMat.colRange(0,3).rowRange(0,3);
    cv::Mat p = inputMat.colRange(3,4).rowRange(0,3);

    /*debug: opencv......*/
    cv::Mat R_rat = R.clone();
    cv::Mat p_tra = p.clone();

    R_rat = R_rat.t();  // rotation of inverse
    p_tra = -1 * R_rat * p_tra; // translation of inverse

    R_rat.copyTo(outputMat.colRange(0,3).rowRange(0,3));
    p_tra.copyTo(outputMat.colRange(3,4).rowRange(0,3));

}

void ToolModel::renderTool(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P, cv::OutputArray jac) {

    Compute_Silhouette(body_faces, body_neighbors, body_Vmat, body_Nmat, CamMat, image, cv::Mat(tool.rvec_cyl),
                       cv::Mat(tool.tvec_cyl), P, jac);

    Compute_Silhouette(ellipse_faces, ellipse_neighbors, ellipse_Vmat, ellipse_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_elp), cv::Mat(tool.tvec_elp), P, jac);

    Compute_Silhouette(griper1_faces, griper1_neighbors, gripper1_Vmat, gripper1_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_grip1), cv::Mat(tool.tvec_grip1), P, jac);

    Compute_Silhouette(griper2_faces, griper2_neighbors, gripper2_Vmat, gripper2_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_grip2), cv::Mat(tool.tvec_grip2), P, jac);

};

/*** difference: give tool_normals ***/
void ToolModel::renderToolUKF(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P,
                         cv::Mat &tool_points, cv::Mat &tool_normals, cv::OutputArray jac) {

    std::vector< std::vector<double> > tool_vertices_normals;
    Compute_Silhouette_UKF(body_faces, body_neighbors, body_Vmat, body_Nmat, CamMat, image, cv::Mat(tool.rvec_cyl),
                       cv::Mat(tool.tvec_cyl), P, tool_vertices_normals, jac);

    std::vector< std::vector<double> > tool_oval_normals;
    Compute_Silhouette_UKF(oval_normal_faces, oval_normal_neighbors, oval_normal_Vmat, oval_normal_Nmat, CamMat, image,
                       cv::Mat(tool.rvec_elp), cv::Mat(tool.tvec_elp), P, tool_oval_normals, jac);

    std::vector< std::vector<double> > tool_gripper_normals;
    Compute_Silhouette_UKF(griper1_faces, griper1_neighbors, gripper1_Vmat, gripper1_Nmat, CamMat, image,
                           cv::Mat(tool.rvec_grip1), cv::Mat(tool.tvec_grip1), P, tool_gripper_normals, jac);

    Compute_Silhouette_UKF(griper2_faces, griper2_neighbors, gripper2_Vmat, gripper2_Nmat, CamMat, image,
                           cv::Mat(tool.rvec_grip2), cv::Mat(tool.tvec_grip2), P, tool_gripper_normals, jac);

    int point_size = tool_oval_normals.size();
    for (int i = 0; i < point_size; ++i) {
        cv::Mat temp_normal(1,2,CV_64FC1);
        temp_normal.at<double>(0,0) = tool_oval_normals[i][2];
        temp_normal.at<double>(0,1) = tool_oval_normals[i][3];

        //flip the normal
        temp_normal = -1 * temp_normal;
        tool_oval_normals[i][2] = temp_normal.at<double>(0,0);
        tool_oval_normals[i][3] = temp_normal.at<double>(0,1);

    }

//    for (int i = 0; i <tool_vertices_normals.size() ; ++i) {
//        ROS_INFO("tool_vertices_normals i: %d, %f, %f,%f,%f ", i, tool_vertices_normals[i][0], tool_vertices_normals[i][1],tool_vertices_normals[i][2],tool_vertices_normals[i][3]);
//    }
//    reorganizeVertices(tool_vertices_normals, tool_points, tool_normals);
    gatherNormals(tool_vertices_normals, tool_oval_normals, tool_gripper_normals, tool_points, tool_normals);

};

void ToolModel::reorganizeVertices(std::vector< std::vector<double> > &tool_vertices_normals, cv::Mat &tool_points, cv::Mat &tool_normals){

    std::sort(tool_vertices_normals.begin(), tool_vertices_normals.end());
    tool_vertices_normals.erase(std::unique(tool_vertices_normals.begin(), tool_vertices_normals.end()), tool_vertices_normals.end());

    int point_dim = tool_vertices_normals.size();

    int actual_dim = point_dim;   //need one more for other orientation
    tool_points = cv::Mat::zeros(actual_dim, 2,CV_64FC1);
    tool_normals = cv::Mat::zeros(actual_dim, 2,CV_64FC1);

    for (int j = 0; j < point_dim; ++j) {
        tool_points.at<double>(j,0) = tool_vertices_normals[j][0];
        tool_points.at<double>(j,1) = tool_vertices_normals[j][1];

        tool_normals.at<double>(j,0) = tool_vertices_normals[j][2];
        tool_normals.at<double>(j,1) = tool_vertices_normals[j][3];
    }
    /******** using less side normals: current best using stereo ********/
//    std::vector< std::vector<double> > temp_vec_normals;
//    ///need adjust the first few normals
//
//    for (int m = 0; m <point_dim - 9; ++m) {
//        temp_vec_normals.push_back(tool_vertices_normals[m]);
//    }
//
//    cv::Mat cylinder_norm(1,2,CV_64FC1);
//    cylinder_norm.at<double>(0,0) = temp_vec_normals[0][2];
//    cylinder_norm.at<double>(0,1) = temp_vec_normals[0][3];
//    cv::normalize(cylinder_norm, cylinder_norm);
//    //ROS_INFO_STREAM("cylinder_norm " << cylinder_norm);
//    for (int l = point_dim - 9; l < point_dim; ++l) {
//        cv::Mat temp(1,2,CV_64FC1);
//        temp.at<double>(0,0) = tool_vertices_normals[l][2];
//        temp.at<double>(0,1) = tool_vertices_normals[l][3];
//        cv::normalize(temp, temp);
//        double bar = cylinder_norm.dot(temp);
//        //ROS_INFO_STREAM("bar IS " << bar);
//        if(bar < 0.3 && bar > -0.1){
//            temp_vec_normals.push_back(tool_vertices_normals[l]);
//        }
//    }
//    int actual_dim = temp_vec_normals.size();   //need one more for other orientation
//    tool_points = cv::Mat::zeros(actual_dim, 2,CV_64FC1);
//    tool_normals = cv::Mat::zeros(actual_dim, 2,CV_64FC1);
//
//    for (int j = 0; j < actual_dim; ++j) {
//        tool_points.at<double>(j,0) = temp_vec_normals[j][0];
//        tool_points.at<double>(j,1) = temp_vec_normals[j][1];
//
//        tool_normals.at<double>(j,0) = temp_vec_normals[j][2];
//        tool_normals.at<double>(j,1) = temp_vec_normals[j][3];
//
//    }
    /***** normalize *****/
    for (int i = 0; i < actual_dim; ++i) {
        cv::Mat temp(1,2,CV_64FC1);
        cv::normalize(tool_normals.row(i), temp);
        temp.copyTo(tool_normals.row(i));
    }
};

void ToolModel::gatherNormals(std::vector< std::vector<double> > &part1_normals, std::vector< std::vector<double> > &part2_normals, std::vector< std::vector<double> > &part3_normals, cv::Mat &tool_points, cv::Mat &tool_normals){

    std::sort(part1_normals.begin(), part1_normals.end());
    part1_normals.erase(std::unique(part1_normals.begin(), part1_normals.end()), part1_normals.end());

    std::sort(part2_normals.begin(), part2_normals.end());
    part2_normals.erase(std::unique(part2_normals.begin(), part2_normals.end()), part2_normals.end());

    std::sort(part3_normals.begin(), part3_normals.end());
    part3_normals.erase(std::unique(part3_normals.begin(), part3_normals.end()), part3_normals.end());

    int point_dim = part1_normals.size();

    std::vector< std::vector<double> > temp_vec_normals;
    ///need adjust the first few normals
    for (int m = 0; m <point_dim - 9; ++m) {
        temp_vec_normals.push_back(part1_normals[m]);
    }

    cv::Mat cylinder_norm(1,2,CV_64FC1);
    cylinder_norm.at<double>(0,0) = temp_vec_normals[0][2];
    cylinder_norm.at<double>(0,1) = temp_vec_normals[0][3];
    cv::normalize(cylinder_norm, cylinder_norm);

    //ROS_INFO_STREAM("cylinder_norm " << cylinder_norm);
    for (int l = point_dim - 9; l < point_dim; ++l) {
        cv::Mat temp(1,2,CV_64FC1);
        temp.at<double>(0,0) = part1_normals[l][2];
        temp.at<double>(0,1) = part1_normals[l][3];
        cv::normalize(temp, temp);
        double bar = cylinder_norm.dot(temp);
        if(bar < 0.3 && bar > -0.1){
            temp_vec_normals.push_back(part1_normals[l]);
        }
    }

    /****** oval part normals *****/
    for (int i = 0; i < 2; ++i) { // here we really don't need too much normals
        temp_vec_normals.push_back(part2_normals[i]);
    }

    /****** oval part normals *****/
    temp_vec_normals.push_back(part3_normals[4]);
    temp_vec_normals.push_back(part3_normals[7]);


    int actual_dim = temp_vec_normals.size();   //need one more for other orientation
    tool_points = cv::Mat::zeros(actual_dim, 2,CV_64FC1);
    tool_normals = cv::Mat::zeros(actual_dim, 2,CV_64FC1);

    for (int j = 0; j < actual_dim; ++j) {
        tool_points.at<double>(j,0) = temp_vec_normals[j][0];
        tool_points.at<double>(j,1) = temp_vec_normals[j][1];
        tool_normals.at<double>(j,0) = temp_vec_normals[j][2];
        tool_normals.at<double>(j,1) = temp_vec_normals[j][3];
    }

    /***** normalize *****/
    for (int i = 0; i < actual_dim; ++i) {
        cv::Mat temp(1,2,CV_64FC1);
        cv::normalize(tool_normals.row(i), temp);
        temp.copyTo(tool_normals.row(i));
    }
};

float ToolModel::calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage) {

    float matchingScore;

    /*** When ROI is an empty rec, the position of tool is simply just not match, return 0 matching score ***/
    cv::Mat ROI_toolImage = toolImage.clone(); //(ROI); //crop tool image
    cv::Mat segImageGrey = segmentedImage.clone(); //(ROI); //crop segmented image, notice the size of the segmented image

    segImageGrey.convertTo(segImageGrey, CV_32FC1);
    cv::Mat segImgBlur;
    cv::GaussianBlur(segImageGrey,segImgBlur, cv::Size(9,9),4,4);
    segImgBlur /= 255; //scale the blurred image

    cv::Mat toolImageGrey; //grey scale of toolImage since tool image has 3 channels
    cv::Mat toolImFloat; //Float data type of grey scale tool image

    cv::cvtColor(ROI_toolImage, toolImageGrey, CV_BGR2GRAY); //convert it to grey scale

    toolImageGrey.convertTo(toolImFloat, CV_32FC1); // convert grey scale to float
    cv::Mat result(1, 1, CV_32FC1);

    cv::matchTemplate(segImgBlur, toolImFloat, result, CV_TM_CCORR_NORMED); //seg, toolImg
    matchingScore = static_cast<float> (result.at<float>(0));

    return matchingScore;
}

float ToolModel::calculateChamferScore(cv::Mat &toolImage, const cv::Mat &segmentedImage) {

    float output = 0;

    ///////////////////PART 1: particle rendering processing////////////////////////////////
    //get toolImage's binary grayscale representation (all 1's refer to pixels representing the particle)
    cv::Mat ROI_toolImage = toolImage.clone(); //CV_8UC3
    cv::Mat toolImageGrey(ROI_toolImage.size(), CV_8UC1); //grey scale of toolImage since tool image has 3 channels
    cv::cvtColor(ROI_toolImage, toolImageGrey, CV_BGR2GRAY); //render bw version of ROI_toolImage on toolImageGrey

    cv::Mat toolImFloat(ROI_toolImage.size(), CV_32FC1); //Float data type of toolImageGrey
    toolImageGrey.convertTo(toolImFloat, CV_32FC1);

    cv::Mat BinaryImg(toolImFloat.size(), toolImFloat.type()); //binary representation of toolImFloat
    BinaryImg = toolImFloat * (1.0/255);


    ///////////////////PART 2: segmenting image processing////////////////////////////////
    //convert segmented image to 8UC1 and invert it.  All 0 values in segImgGrey are parts of the tool
    cv::Mat segImgGrey = segmentedImage.clone();
    segImgGrey.convertTo(segImgGrey, CV_8UC1);
    for (int i = 0; i < segImgGrey.rows; i++) {
        for (int j = 0; j < segImgGrey.cols; j++) {
            segImgGrey.at<uchar>(i,j) = 255 - segImgGrey.at<uchar>(i,j);
        }
    }

    //compute normalized closest distance of every pixel to a segmented tool part (represented by 0's in segImgGrey)
    cv::Mat distance_img; //distance to closest part of tool
    cv::Mat normDIST; //normalized distance_img
    cv::distanceTransform(segImgGrey, distance_img, CV_DIST_L2, 3);
    cv::normalize(distance_img, normDIST, 0.00, 1.00, cv::NORM_MINMAX);

//    cv::imshow("segImgGrey img", segImgGrey);
//    cv::imshow("distance_img img", distance_img);
//    cv::imshow("Normalized img", normDIST);
//    cv::waitKey();

    //////////////////PART 3: Compute Chamfer Distance//////////////////////////
    //When we multiply BinaryImg by normDist, we get the distance of every pixel representing the particle to the closest pixel representing the segmented image
    cv::Mat resultImg;
    cv::multiply(normDIST, BinaryImg, resultImg);

    //We now sum all these distances
    for (int k = 0; k < resultImg.rows; ++k) {
        for (int i = 0; i < resultImg.cols; ++i) {
            double mul = resultImg.at<float>(k,i);
            if(mul > 0.0)
                output += mul;
        }
    }
    //ROS_INFO_STREAM("OUTPUT: " << output);

    //Scale the output
    output = exp(-1 * output/80);
    return output;
};

cv::Point2d ToolModel::reproject(const cv::Mat &point, const cv::Mat &P) {
    cv::Mat results(3, 1, CV_64FC1);
    cv::Point2d output;

    //convert point to homogenous matrix representation to allow for multiplication
    cv::Mat ptMat(4, 1, CV_64FC1);
    ptMat.at<double>(0, 0) = point.at<double>(0, 0);
    ptMat.at<double>(1, 0) = point.at<double>(1, 0);
    ptMat.at<double>(2, 0) = point.at<double>(2, 0);
    ptMat.at<double>(3, 0) = 1.0;

    //project point under camera frame using multiplication
    results = P * ptMat;
    output.x = results.at<double>(0, 0) / results.at<double>(2, 0);
    output.y = results.at<double>(1, 0) / results.at<double>(2, 0);

    return output;
};