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
using cv_projective::transformPoints;
boost::mt19937 rng((const uint32_t &) time(0));
using namespace std;

//constructor
ToolModel::ToolModel(cv::Mat& CamMat){

    offset_body = 0.3429;  //meters
    offset_ellipse = 0.45352716;
    offset_gripper = 0.46118; //0.46118 - 0.4522

    /****initialize the rotation and traslation points*****/
    q_ellipse = cv::Mat(4,1,CV_64FC1);
    q_ellipse.at<double>(0,0) = 0;
    q_ellipse.at<double>(1,0) = offset_ellipse - offset_body;  //0.1106m
    q_ellipse.at<double>(2,0) = 0;
    q_ellipse.at<double>(3,0) = 1;

    q_gripper = cv::Mat(3,1,CV_64FC1);
    q_gripper.at<double>(0,0) = 0;
    q_gripper.at<double>(1,0) = offset_gripper - 0.4522;  //
    q_gripper.at<double>(2,0) = 0;



    //CamMat = cv::Mat(4, 4, CV_64FC1);  //obtain this



    /****initialize the vertices fo different part of tools****/
	load_model_vertices("/home/deeplearning/ros_ws/src/tooltrack/tool_parts/refine_cylinder_2.obj", body_vertices, body_Vnormal, body_faces, body_neighbors );
    // load_model_vertices("/home/deeplearning/ros_ws/src/tooltrack/tool_parts/refine_ellipse_2.obj", ellipse_vertices, ellipse_Vnormal, ellipse_faces, ellipse_neighbors );
    // load_model_vertices("/home/deeplearning/ros_ws/src/tooltrack/tool_parts/gripper_1.obj", griper1_vertices, griper1_Vnormal, griper1_faces, griper1_neighbors );
    // load_model_vertices("/home/deeplearning/ros_ws/src/tooltrack/tool_parts/gripper_2.obj", griper2_vertices, griper2_Vnormal, griper2_faces, griper2_neighbors );

    ROS_INFO("after loading");
    modify_model_(body_vertices, body_Vnormal, body_Vpts, body_Npts, offset_body, body_Vmat);
    // modify_model_(ellipse_vertices, ellipse_Vnormal, ellipse_Vpts, ellipse_Npts, offset_ellipse );
    // modify_model_(griper1_vertices, griper1_Vnormal, griper1_Vpts, griper1_Npts, offset_gripper );
    // modify_model_(griper2_vertices, griper2_Vnormal, griper2_Vpts, griper2_Npts, offset_gripper );

    bodyFace_info = collectFacesTranform(body_faces, body_Vpts, body_Npts);
    bodyFace_normal = getFaceNormals(body_faces, body_Vpts, body_Npts);
    bodyFace_centroid = getFaceCentroid(body_faces, body_Vpts);

    /****convert to cv points for second silhouette computation****/
    // camTransformPoints(CamMat, body_Vpts, CamBodyPts);
    // camTransformPoints(CamMat, ellipse_Vpts, CamEllipPts);

    // camTransformVecs(CamMat, body_Npts, CamBodyNorms);
    // camTransformVecs(CamMat, ellipse_Npts, CamEllipNorms);

//    cv::Point3d maxx(0.0) , minn(0.0);
//    for (auto abc: CamBodyPts) {
//        maxx.x = max(maxx.x, abc.x);
//        maxx.y = max(maxx.y, abc.y);
//        maxx.z = max(maxx.z, abc.z);
//        minn.x = min(minn.x, abc.x);
//        minn.y = min(minn.y, abc.y);
//        minn.z = min(minn.z, abc.z);
//    }
//    cout << "max x:" << maxx.x << " max y:" << maxx.y << " max z:" << maxx.z << endl;
//    cout << "min x:" << minn.x << " min y:" << minn.y << " min z:" << minn.z << endl;

    //ROS_INFO_STREAM("THE ELLIPSE FACES: " << ellipse_faces.size());

};

double ToolModel::randomNumber(double stdev, double mean){
	boost::normal_distribution<> nd(mean, stdev);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
	double d = var_nor();

	return d;

}
void ToolModel::ConvertInchtoMeters(std::vector< cv::Point3d > &input_vertices){
    int size = (int) input_vertices.size();
    for (int i = 0; i < size; ++i)
    {
        input_vertices[i].x = input_vertices[i].x * 0.0254;
        input_vertices[i].y = input_vertices[i].y * 0.0254;
        input_vertices[i].z = input_vertices[i].z * 0.0254;
    }
}

cv::Point3d ToolModel::ConvertCelitoMeters(cv::Point3d &input_pt){

        cv::Point3d out_pt;
        out_pt.x = input_pt.x * 0.01;
        out_pt.y = input_pt.y * 0.01;
        out_pt.z = input_pt.z * 0.01;
        return out_pt;
    
};

//set zero configuratio for tool points;
void ToolModel::load_model_vertices(const char * path, std::vector< glm::vec3 > &out_vertices, std::vector< glm::vec3 > &vertex_normal, 
    std::vector< std::vector<int> > &out_faces,  std::vector< std::vector<int> > &neighbor_faces){

    ROS_INFO("Loading OBJ file %s...\n", path);

    std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
    //std::vector< unsigned int > vertexIndices;
    std::vector< glm::vec3 > temp_vertices;
    std::vector< glm::vec2 > temp_uvs;
    std::vector< glm::vec3 > temp_normals;

    std::vector< int > temp_face;
    temp_face.resize(6);  //need three vertex and corresponding normals

    FILE * file = fopen(path, "r");
    if( file == NULL ){
        printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
    }


    while( 1 ){

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader
        
        if ( strcmp( lineHeader, "v" ) == 0 ){
            glm::vec3 vertex;
            
            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z );
            // cout<<"vertex x"<< vertex.x<<endl;
            // cout<<"vertex y"<<vertex.y<<endl;
            // cout<<"vertex z"<<vertex.z<<endl;
            //temp_vertices.push_back(vertex);
            out_vertices.push_back(vertex);
        }

        else if ( strcmp( lineHeader, "vt" ) == 0 ){
        glm::vec2 uv;
        fscanf(file, "%f %f\n", &uv.x, &uv.y );
        // cout<<"uv"<<uv.x<<endl;
        temp_uvs.push_back(uv);
        } 
        else if ( strcmp( lineHeader, "vn" ) == 0 ){
            glm::vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z );
            // cout<<"normal x"<< normal.x<<endl;
            // cout<<"normal y"<< normal.y<<endl;
            // cout<<"normal z"<< normal.z<<endl;
            //temp_normals.push_back(normal);
            vertex_normal.push_back(normal);

        }
        else if ( strcmp( lineHeader, "f" ) == 0 ){
    
    unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
    int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
    if (matches != 9){
        ROS_ERROR("File can't be read by our simple parser : ( Try exporting with other options\n");
    }
    
    vertexIndices.push_back(vertexIndex[0]);
    vertexIndices.push_back(vertexIndex[1]);
    vertexIndices.push_back(vertexIndex[2]);
    uvIndices    .push_back(uvIndex[0]);
    uvIndices    .push_back(uvIndex[1]);
    uvIndices    .push_back(uvIndex[2]);
    normalIndices.push_back(normalIndex[0]);
    normalIndices.push_back(normalIndex[1]);
    normalIndices.push_back(normalIndex[2]);

    temp_face[0] = vertexIndex[0]-1;
    temp_face[1] = vertexIndex[1]-1;
    temp_face[2] = vertexIndex[2]-1;
    temp_face[3] = normalIndex[0]-1;
    temp_face[4] = normalIndex[1]-1;
    temp_face[5] = normalIndex[2]-1;

    out_faces.push_back(temp_face);

               
        }
    }

    // cout<<"size"<< out_vertices.size()<<endl;
    // cout<<"NORMAL size "<< vertex_normal.size()<<endl;

    /***find neighbor faces***/
    neighbor_faces.resize(out_faces.size());
    std::vector< int > temp_vec;

    for (int i = 0; i < neighbor_faces.size(); ++i)
    {
        /*********find the neighbor faces************/
        for (int j = i; j < neighbor_faces.size(); ++j)
        {
            if ( j != i)
            {
                int match = Compare_vertex(out_faces[i], out_faces[j], temp_vec); 
                
                if ( match == 2 ) //so face i and face j share an edge
                {
                    //ROS_INFO_STREAM("SIZE FO THE TEMP VEC: " << temp_vec.size() );
                    neighbor_faces[i].push_back(j); // mark the neighbor face index
                    neighbor_faces[i].push_back(temp_vec[0]);
                    neighbor_faces[i].push_back(temp_vec[1]);

                }

                temp_vec.clear();
            }

        }
       //cout<< "neighbor number: "<< (neigh_faces[i].size())/3 << endl;  //so now the neighbor contains one 
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

/*output a glm to a cv 3d point*/
void ToolModel::Convert_glTocv_pts(std::vector< glm::vec3 > &input_vertices, std::vector< cv::Point3d > &out_vertices){
    int vsize = input_vertices.size();
    out_vertices.resize(vsize);
        for (int i = 0; i < vsize; ++i)
    {
        out_vertices[i].x = input_vertices[i].x;
        out_vertices[i].y = input_vertices[i].y;
        out_vertices[i].z = input_vertices[i].z;
    }
};

cv::Point3d ToolModel::Convert_glTocv_pt(glm::vec3 &input_vertex){
    cv::Point3d out_vertex;
    out_vertex.x = input_vertex.x;
    out_vertex.y = input_vertex.y;
    out_vertex.z = input_vertex.z;
    return out_vertex;

};

/* find the camera view point, should it be (0,0,0), input faces stores the indices of the vertices and normals, 
which are not related to the pose of the tool object*/
cv::Mat ToolModel::collectFacesTranform(const std::vector< std::vector<int> > &input_faces, const std::vector< cv::Point3d > &input_vertices, 
                                    const std::vector< cv::Point3d > &input_Vnormal){
    int f_size = 6 * input_faces.size();
    cv::Mat outputFaces(4, f_size, CV_64FC1 );


    for (int i = 0; i < input_faces.size(); ++i)
    {
        int count = 6 * i;
        int v1 = input_faces[i][0];
        int v2 = input_faces[i][1];
        int v3 = input_faces[i][2];
        int n1 = input_faces[i][3];
        int n2 = input_faces[i][4];
        int n3 = input_faces[i][5];

        outputFaces.at<double>(0, count) = input_vertices[v1].x;
        outputFaces.at<double>(1, count) = input_vertices[v1].y;
        outputFaces.at<double>(2, count) = input_vertices[v1].z;
        outputFaces.at<double>(3, count) = 1;

        outputFaces.at<double>(0, count+1) = input_vertices[v2].x;
        outputFaces.at<double>(1, count+1) = input_vertices[v2].y;
        outputFaces.at<double>(2, count+1) = input_vertices[v2].z;
        outputFaces.at<double>(3, count+1) = 1;

        outputFaces.at<double>(0, count+2) = input_vertices[v3].x;
        outputFaces.at<double>(1, count+2) = input_vertices[v3].y;
        outputFaces.at<double>(2, count+2) = input_vertices[v3].z;
        outputFaces.at<double>(3, count+2) = 1;

        outputFaces.at<double>(0, count+3) = input_Vnormal[n1].x;
        outputFaces.at<double>(1, count+3) = input_Vnormal[n1].y;
        outputFaces.at<double>(2, count+3) = input_Vnormal[n1].z;
        outputFaces.at<double>(3, count+3) = 0;

        outputFaces.at<double>(0, count+4) = input_Vnormal[n2].x;
        outputFaces.at<double>(1, count+4) = input_Vnormal[n2].y;
        outputFaces.at<double>(2, count+4) = input_Vnormal[n2].z;
        outputFaces.at<double>(3, count+4) = 0;

        outputFaces.at<double>(0, count+5) = input_Vnormal[n3].x;
        outputFaces.at<double>(1, count+5) = input_Vnormal[n3].y;
        outputFaces.at<double>(2, count+5) = input_Vnormal[n3].z;
        outputFaces.at<double>(3, count+5) = 0;

    }

    return outputFaces;

};

 
cv::Mat ToolModel::transformMats(const cv::Mat &face_info, cv::Mat &CamMat, const cv::Mat &rvec, const cv::Mat &tvec){
    
    cv::Mat temp_mat = transformPoints(face_info,rvec,tvec);

    cv::Mat transformedFaces =  camTransformMats(CamMat, temp_mat );


};


cv::Point3d ToolModel::transformation_pts(const cv::Point3d &point, const cv::Mat& rvec, const cv::Mat &tvec){
    cv::Mat prjPoints(4,1,CV_64FC1);

    cv::Point3d output;

    cv::Mat ptMat(4,1,CV_64FC1);
    ptMat.at<double>(0,0) = point.x;
    ptMat.at<double>(1,0) = point.y;
    ptMat.at<double>(2,0) = point.z;
    ptMat.at<double>(3,0) = 1.0;

    //Mat transDeriv;

        if(rvec.total()==3 && tvec.total()==3)
        {
            // if(jac.needed())
            // {
            //     prjPoints = transformPoints(ptMat,rvec,tvec,transDeriv);
            // }
            //else
            //{
                prjPoints = transformPoints(ptMat,rvec,tvec);
            //}
        }
        else
        {
            ROS_INFO("Nothing in point transformed");
            prjPoints = ptMat;
        }

        output.x = prjPoints.at<double>(0,0);
        output.y = prjPoints.at<double>(1,0);
        output.z = prjPoints.at<double>(2,0);

        return output;

};

cv::Point3d ToolModel::transformation_nrms(const cv::Point3d &vec, const cv::Mat& rvec, const cv::Mat &tvec){
    cv::Mat prjPoints(4,1,CV_64FC1);

    cv::Point3d output_v;

    cv::Mat ptMat(4,1,CV_64FC1);
    ptMat.at<double>(0,0) = vec.x;
    ptMat.at<double>(1,0) = vec.y;
    ptMat.at<double>(2,0) = vec.z;
    ptMat.at<double>(3,0) = 0.0;

    //Mat transDeriv;

        if(rvec.total()==3 && tvec.total()==3)
        {
            // if(jac.needed())
            // {
            //     prjPoints = transformPoints(ptMat,rvec,tvec,transDeriv);
            // }
            //else
            //{
                prjPoints = transformPoints(ptMat,rvec,tvec);
            //}
        }
        else
        {
            ROS_INFO("Nothing in vec transformed");
            prjPoints = ptMat;
        }

        output_v.x = prjPoints.at<double>(0,0);
        output_v.y = prjPoints.at<double>(1,0);
        output_v.z = prjPoints.at<double>(2,0);

        return output_v;    

};

/*camera transformations*/
cv::Mat ToolModel::camTransformMats(cv::Mat &cam_mat, cv::Mat &input_mat ){
    /*cam mat should be a 4x4*/
    
    cv::Mat Inv = cam_mat.inv();

    cv::Mat output_mat = Inv*input_mat; //transform the obj to camera frames

    return output_mat;
};

cv::Point3d ToolModel::camTransformPoint(cv::Mat &cam_mat, cv::Point3d &input_vertex){

    /*cam mat should be a 4x4*/
    cv::Mat temp(4,1,CV_64FC1);
    cv::Mat Inv = cam_mat.inv();

    cv::Point3d output_vertex;

    temp.at<double>(0,0) = input_vertex.x;
    temp.at<double>(1,0) = input_vertex.y;
    temp.at<double>(2,0) = input_vertex.z;
    temp.at<double>(3,0) = 1;  //point

    temp = Inv*temp; //transform the obj to camera frames


    output_vertex.x = temp.at<double>(0,0);
    output_vertex.y = temp.at<double>(1,0);
    output_vertex.z = temp.at<double>(2,0);

    return output_vertex;

};
cv::Point3d ToolModel::camTransformVec(cv::Mat &cam_mat, cv::Point3d &input_vec){
    /*cam mat should be a 4x4*/
    cv::Mat temp(4,1,CV_64FC1);
    cv::Mat Inv = cam_mat.inv();

    cv::Point3d output_vec;

    temp.at<double>(0,0) = input_vec.x;
    temp.at<double>(1,0) = input_vec.y;
    temp.at<double>(2,0) = input_vec.z;
    temp.at<double>(3,0) = 0;  //vec

    temp = Inv*temp; //transform the obj to camera frames

    output_vec.x = temp.at<double>(0,0);
    output_vec.y = temp.at<double>(1,0);
    output_vec.z = temp.at<double>(2,0);

    return output_vec;
};

void ToolModel::camTransformPoints(cv::Mat &cam_mat, std::vector< cv::Point3d > &input_vertices, std::vector< cv::Point3d > &output_vertices){

    /*cam mat should be a 4x4*/
    cv::Mat temp(4,1,CV_64FC1);
    output_vertices.resize(input_vertices.size());

    temp.at<double>(3,0) = 1; //point

    cv::Mat Inv = cam_mat.inv();

   for (int i = 0; i < output_vertices.size(); ++i)
    {
        //transform to homogenous
        temp.at<double>(0,0) = input_vertices[i].x;
        temp.at<double>(1,0) = input_vertices[i].y;
        temp.at<double>(2,0) = input_vertices[i].z;

        temp = Inv*temp; //transform the obj to camera frames

        output_vertices[i].x = temp.at<double>(0,0);
        output_vertices[i].y = temp.at<double>(1,0);
        output_vertices[i].z = temp.at<double>(2,0);

    } 

};

void ToolModel::camTransformVecs(cv::Mat &cam_mat, std::vector< cv::Point3d > &input_normals, std::vector< cv::Point3d > &output_normals){

    /*cam mat should be a 4x4*/
    cv::Mat temp(4,1,CV_64FC1);
    output_normals.resize(input_normals.size());

    temp.at<double>(3,0) = 0;  //vector
    cv::Mat Inv = cam_mat.inv();
    
   for (int i = 0; i < output_normals.size(); ++i)
    {
        //transform to homogenous
        temp.at<double>(0,0) = input_normals[i].x;
        temp.at<double>(1,0) = input_normals[i].y;
        temp.at<double>(2,0) = input_normals[i].z;

        temp = Inv*temp; //transform the obj to camera frames

        output_normals[i].x = temp.at<double>(0,0);
        output_normals[i].y = temp.at<double>(1,0);
        output_normals[i].z = temp.at<double>(2,0);

        //output_normals[i] = Normalize(output_normals[i]);
    } 

};
/****camera transformations****/

/***********using BODY frame for transformation********************/
void ToolModel::Compute_Silhouette(const std::vector< std::vector<int> > &input_faces, const std::vector< std::vector<int> > &neighbor_faces, 
                                   const std::vector< cv::Point3d > &body_vertices, const std::vector< cv::Point3d > &body_Vnormal, cv::Mat &CamMat,
                                   cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec, const cv::Mat &P, cv::OutputArray jac, cv::Point2d &XY_max, cv::Point2d &XY_min){
    // cv::Mat jacMat;
    // if(jac.needed())
    // {
    //     //jac.create(2*(segmentCount+1),6,CV_64FC1);
    //     jac.create(2*(total_pts_size),6,CV_64FC1);
    //     jacMat = jac.getMat();
    // }

    // cv::Mat temp_jac;

    int neighbor_num = 0;

/********************approach one: using body frame for transformation**************************/
    for (int i = 0; i < input_faces.size(); ++i)
    {
        //when there are neighbors, it is necessary to compute the edge

        neighbor_num = (neighbor_faces[i].size())/3;  //each neighbor has two vertices

        if ( neighbor_num > 0)  
        {
            //ROS_INFO("INSIDE LOOP %d", i);
            /*****first get the surface normal******/

        int v1 = input_faces[i][0];
        int v2 = input_faces[i][1];
        int v3 = input_faces[i][2];
        int n1 = input_faces[i][3];
        int n2 = input_faces[i][4];
        int n3 = input_faces[i][5];

        /*testing*/
        // cv::Point3d pt1 = body_vertices[v1];
        // cv::Point3d pt2 = body_vertices[v2];
        // cv::Point3d pt3 = body_vertices[v3];

        // cv::Point3d vn1 = body_Vnormal[n1];
        // cv::Point3d vn2 = body_Vnormal[n2];
        // cv::Point3d vn3 = body_Vnormal[n3];
        /*debug*/
        // ROS_INFO_STREAM("face: " << i );
        // ROS_INFO_STREAM("pt1.x: " << pt1.x << " pt1.y: " << pt1.y << " pt1.z: " << pt1.z );
        // ROS_INFO_STREAM("pt2.x: " << pt2.x << " pt2.y: " << pt2.y << " pt2.z: " << pt2.z );
        // ROS_INFO_STREAM("pt3.x: " << pt3.x << " pt3.y: " << pt3.y << " pt3.z: " << pt3.z );

        // ROS_INFO_STREAM("vn1.x: " << vn1.x << " vn1.y: " << vn1.y << " vn1.z: " << vn1.z );
        // ROS_INFO_STREAM("vn2.x: " << vn2.x << " vn2.y: " << vn2.y << " vn2.z: " << vn2.z );
        // ROS_INFO_STREAM("vn3.x: " << vn3.x << " vn3.y: " << vn3.y << " vn3.z: " << vn3.z ); 
        // ROS_INFO("NEXT FACE");
        /******transformation for neightbor faces*******/
        cv::Point3d pt1 = transformation_pts(body_vertices[v1], rvec, tvec);
        cv::Point3d pt2 = transformation_pts(body_vertices[v2], rvec, tvec);
        cv::Point3d pt3 = transformation_pts(body_vertices[v3], rvec, tvec);

        cv::Point3d vn1 = transformation_nrms(body_Vnormal[n1], rvec, tvec);
        cv::Point3d vn2 = transformation_nrms(body_Vnormal[n2], rvec, tvec);
        cv::Point3d vn3 = transformation_nrms(body_Vnormal[n3], rvec, tvec);


        cv::Point3d Cpt1 = camTransformPoint(CamMat, pt1);
        cv::Point3d Cpt2 = camTransformPoint(CamMat, pt2);
        cv::Point3d Cpt3 = camTransformPoint(CamMat, pt3);

        cv::Point3d Cvn1 = camTransformVec(CamMat, vn1);
        cv::Point3d Cvn2 = camTransformVec(CamMat, vn1);
        cv::Point3d Cvn3 = camTransformVec(CamMat, vn1);
  

        cv::Point3d fnormal = FindFaceNormal(Cpt1, Cpt2, Cpt3, Cvn1, Cvn2, Cvn3); //knowing the direction and normalized
        //ROS_INFO_STREAM("fnormal: " << fnormal);

        // if (fnormal.x == 0.0 && fnormal.y == 0.0 && fnormal.z == 0.0 )
        // {
        //     ROS_ERROR("ONE SURFACE HAS TWO ALIGNED VECTORS???");
        // }
        cv::Point3d face_point_i = Cpt1 + Cpt2 + Cpt3;
        face_point_i.x = face_point_i.x/3;
        face_point_i.y = face_point_i.y/3;
        face_point_i.z = face_point_i.z/3;

        //ROS_INFO_STREAM("face_point_i: " << face_point_i);

        face_point_i = Normalize(face_point_i);
        //ROS_INFO_STREAM("cam_vec_i.x: " << cam_vec_i.x << "cam_vec_i.y: " << cam_vec_i.y << "cam_vec_i.z: " << cam_vec_i.z  );
        double isfront_i = dotProduct(fnormal, face_point_i);
        //ROS_INFO_STREAM("isfront_i: " << isfront_i);

        for (int neighbor_count = 0; neighbor_count < neighbor_num; ++neighbor_count){  //notice: cannot use J here, since the last j will not be counted

                int j = 3*neighbor_count;

                int v1_ = input_faces[neighbor_faces[i][j]][0];
                int v2_ = input_faces[neighbor_faces[i][j]][1];
                int v3_ = input_faces[neighbor_faces[i][j]][2];
                int n1_ = input_faces[neighbor_faces[i][j]][3];
                int n2_ = input_faces[neighbor_faces[i][j]][4];
                int n3_ = input_faces[neighbor_faces[i][j]][5];

                /*testing*/
                // cv::Point3d pt1_ = body_vertices[v1_];
                // cv::Point3d pt2_ = body_vertices[v2_];
                // cv::Point3d pt3_ = body_vertices[v3_];

                // cv::Point3d vn1_ = body_Vnormal[n1_];
                // cv::Point3d vn2_ = body_Vnormal[n2_];
                // cv::Point3d vn3_ = body_Vnormal[n3_];
                /******transformation for neightbor faces*******/
                cv::Point3d pt1_ = transformation_pts(body_vertices[v1_], rvec, tvec);
                cv::Point3d pt2_ = transformation_pts(body_vertices[v2_], rvec, tvec);
                cv::Point3d pt3_ = transformation_pts(body_vertices[v3_], rvec, tvec);

                cv::Point3d vn1_ = transformation_nrms(body_Vnormal[n1_], rvec, tvec);
                cv::Point3d vn2_ = transformation_nrms(body_Vnormal[n2_], rvec, tvec);
                cv::Point3d vn3_ = transformation_nrms(body_Vnormal[n3_], rvec, tvec);

                cv::Point3d Cpt1_ = camTransformPoint(CamMat, pt1_);
                cv::Point3d Cpt2_ = camTransformPoint(CamMat, pt2_);
                cv::Point3d Cpt3_ = camTransformPoint(CamMat, pt3_);

                cv::Point3d Cvn1_ = camTransformVec(CamMat, vn1_);
                cv::Point3d Cvn2_ = camTransformVec(CamMat, vn2_);
                cv::Point3d Cvn3_ = camTransformVec(CamMat, vn3_);    

                cv::Point3d fnormal_n = FindFaceNormal(Cpt1_, Cpt2_, Cpt3_, Cvn1_, Cvn2_, Cvn3_);
                //ROS_INFO_STREAM("fnormal_n: " << fnormal_n);


                // if (fnormal_n.x == 0.0 && fnormal_n.y == 0.0 && fnormal_n.z == 0.0 )
                // {
                //     ROS_ERROR("neighbor SURFACE HAS TWO ALIGNED VECTORS???");
                // }
                //ROS_INFO_STREAM("fnormal_n.x: " << fnormal_n.x << "fnormal_n.y: " << fnormal_n.y << "fnormal_n.z: " << fnormal_n.z  );

                cv::Point3d face_point_j = Cpt1_ + Cpt2_ + Cpt3_;
                face_point_j.x = face_point_j.x/3;
                face_point_j.y = face_point_j.y/3;
                face_point_j.z = face_point_j.z/3;
                face_point_j = Normalize(face_point_j);

                double isfront_j = dotProduct(fnormal_n, face_point_j);

                //ROS_INFO_STREAM("isfront_j: " << isfront_j);

                if (isfront_i * isfront_j <= 0.0) // one is front, another is back
                {
                    //ROS_INFO("IS AN EGDE");


                    /*finish finding*/
                    cv::Point3d temp = body_vertices[neighbor_faces[i][j+1]];
                    cv::Point3d ept_1 = transformation_pts(temp, rvec, tvec);  //transform before the the camera transformation
                    ept_1 = camTransformPoint(CamMat, ept_1 );
                    

                    temp = body_vertices[neighbor_faces[i][j+2]];
                    cv::Point3d ept_2 = transformation_pts(temp, rvec, tvec);
                    ept_2 = camTransformPoint(CamMat, ept_2 ); //so the two points that on the edge

                    //ROS_INFO_STREAM("ept_1: " << ept_1);
                    //ROS_INFO_STREAM("ept_2: " << ept_2);


                    cv::Point2d prjpt_1 = reproject(ept_1, P);
                    cv::Point2d prjpt_2 = reproject(ept_2, P);

                    //ROS_INFO_STREAM("prjpt_1: " << prjpt_1);
                    //ROS_INFO_STREAM("prjpt_2: " << prjpt_2);

                    // if(jac.needed())
                    // {
                    //     //make the jacobian ((2x) x 6)
                    //     temp_jac.colRange(3,9).copyTo(jacMat.rowRange(i*2,i*2+2));
                    // }

                    cv::line(image, prjpt_1, prjpt_2, cv::Scalar(1,1,1), 1, 8, 0);  /*InputOutputArray img, InputArrayOfArrays pts,const Scalar &color, 
                                                                                     int lineType=LINE_8, int shift=0, Point offset=Point())*/
                    //ROS_INFO("DRAW THE LINE!!!!!!!!");
                    if(prjpt_1.x > XY_max.x) XY_max.x = prjpt_1.x;
                    if(prjpt_1.y > XY_max.y) XY_max.y = prjpt_1.y;
                    if(prjpt_1.x < XY_min.x) XY_min.x = prjpt_1.x;
                    if(prjpt_1.y < XY_min.y) XY_min.y = prjpt_1.y;

                    if(prjpt_2.x > XY_max.x) XY_max.x = prjpt_2.x;
                    if(prjpt_2.y > XY_max.y) XY_max.y = prjpt_2.y;
                    if(prjpt_2.x < XY_min.x) XY_min.x = prjpt_2.x;
                    if(prjpt_2.y < XY_min.y) XY_min.y = prjpt_2.y;

                }

            }

        }


    } 

};

/***************using camera frame for transformation*******************/
void ToolModel::Compute_Silhouette( const std::vector< std::vector<int> > &input_faces, const std::vector< std::vector<int> > &neighbor_faces, 
                                 const cv::Mat &input_Vmat, cv::Mat &face_normal, cv::Mat &face_centroid, cv::Mat &CamMat, cv::Mat &image, const cv::Mat &rvec, const cv::Mat &tvec, 
                                 const cv::Mat &P, cv::OutputArray jac, cv::Point2d &XY_max, cv::Point2d &XY_min)
{

    // cv::Mat jacMat;
    // if(jac.needed())
    // {
    //     //jac.create(2*(segmentCount+1),6,CV_64FC1);
    //     jac.create(2*(total_pts_size),6,CV_64FC1);
    //     jacMat = jac.getMat();
    // }

    // cv::Mat temp_jac;

    cv::Mat newVertices = transformMats(input_Vmat, CamMat, rvec, tvec);  //transformed in cam and new pose
    cv::Mat newFaceNormal = transformMats(face_normal, CamMat, rvec, tvec);
    cv::Mat newFaceCentroid = transformMats(face_centroid, CamMat, rvec, tvec);

    int neighbor_num = 0;

    cv::Mat temp(4,1, CV_64FC1);

/********************approach one: using body frame for transformation**************************/
    for (int i = 0; i < input_faces.size(); ++i)
    {
        //when there are neighbors, it is necessary to compute the edge

        neighbor_num = (neighbor_faces[i].size())/3;  //each neighbor has two vertices

        if ( neighbor_num > 0)  
        {
            //ROS_INFO("INSIDE LOOP %d", i);
            /*****first get the surface normal******/

        //ROS_INFO_STREAM("cam_vec_i.x: " << cam_vec_i.x << "cam_vec_i.y: " << cam_vec_i.y << "cam_vec_i.z: " << cam_vec_i.z  );
        cv::Mat _normal_ = newFaceNormal.col(i);
        cv::Mat _centroid_ = newFaceCentroid.col(i);

        double isfront_i = dotProductMat(_normal_, _centroid_);
        //ROS_INFO_STREAM("isfront_i: " << isfront_i);

        for (int neighbor_count = 0; neighbor_count < neighbor_num; ++neighbor_count){  //notice: cannot use J here, since the last j will not be counted

                int j = 3*neighbor_count;

                // int v2_ = input_faces[neighbor_faces[i][j]][1];
                cv::Mat n_normal_ = newFaceNormal.col(neighbor_faces[i][j]);
                cv::Mat n_centroid_ = newFaceCentroid.col(neighbor_faces[i][j]);

                double isfront_j = dotProductMat(n_normal_, n_centroid_);

                //ROS_INFO_STREAM("isfront_j: " << isfront_j);

                if (isfront_i * isfront_j <= 0.0) // one is front, another is back
                {
                    //ROS_INFO("IS AN EGDE");


                    /*finish finding*/

                    temp = newVertices.col(neighbor_faces[i][j+1]);
                    cv::Point3d ept_1 = convert_MattoPts(temp);  //transform before the the camera transformation

                    
                    temp = newVertices.col(neighbor_faces[i][j+2]);
                    cv::Point3d ept_2 = convert_MattoPts(temp);

                    //ROS_INFO_STREAM("ept_1: " << ept_1);
                    //ROS_INFO_STREAM("ept_2: " << ept_2);


                    cv::Point2d prjpt_1 = reproject(ept_1, P);
                    cv::Point2d prjpt_2 = reproject(ept_2, P);

                    //ROS_INFO_STREAM("prjpt_1: " << prjpt_1);
                    //ROS_INFO_STREAM("prjpt_2: " << prjpt_2);

                    // if(jac.needed())
                    // {
                    //     //make the jacobian ((2x) x 6)
                    //     temp_jac.colRange(3,9).copyTo(jacMat.rowRange(i*2,i*2+2));
                    // }

                    cv::line(image, prjpt_1, prjpt_2, cv::Scalar(1,1,1), 1, 8, 0);  /*InputOutputArray img, InputArrayOfArrays pts,const Scalar &color, 
                                                                                     int lineType=LINE_8, int shift=0, Point offset=Point())*/
                    //ROS_INFO("DRAW THE LINE!!!!!!!!");
                    if(prjpt_1.x > XY_max.x) XY_max.x = prjpt_1.x;
                    if(prjpt_1.y > XY_max.y) XY_max.y = prjpt_1.y;
                    if(prjpt_1.x < XY_min.x) XY_min.x = prjpt_1.x;
                    if(prjpt_1.y < XY_min.y) XY_min.y = prjpt_1.y;

                    if(prjpt_2.x > XY_max.x) XY_max.x = prjpt_2.x;
                    if(prjpt_2.y > XY_max.y) XY_max.y = prjpt_2.y;
                    if(prjpt_2.x < XY_min.x) XY_min.x = prjpt_2.x;
                    if(prjpt_2.y < XY_min.y) XY_min.y = prjpt_2.y;

                }

            }

        }


    } 
    /**************Second approach done*******************/

};

cv::Point3d ToolModel::convert_MattoPts(cv::Mat &input_Mat){ //should be a 4 by 1 mat
    cv::Point3d output_point;
    output_point.x = input_Mat.at<double>(0,0);
    output_point.y = input_Mat.at<double>(1,0);
    output_point.z = input_Mat.at<double>(2,0);

    return output_point;

};
cv::Mat ToolModel::getFaceNormals(const std::vector< std::vector<int> > &input_faces, const std::vector< cv::Point3d > &input_vertices, const std::vector< cv::Point3d > &input_Vnormal){

    int face_size = input_faces.size();
    cv::Mat face_normals(4,face_size,CV_64FC1 );

    for (int i = 0; i < face_size; ++i)
    {
        int v1 = input_faces[i][0];
        int v2 = input_faces[i][1];
        int v3 = input_faces[i][2];
        int n1 = input_faces[i][3];
        int n2 = input_faces[i][4];
        int n3 = input_faces[i][5];

        cv::Point3d pt1 = input_vertices[v1];
        cv::Point3d pt2 = input_vertices[v2];
        cv::Point3d pt3 = input_vertices[v3];

        cv::Point3d normal1 = input_Vnormal[n1];
        cv::Point3d normal2 = input_Vnormal[n2];
        cv::Point3d normal3 = input_Vnormal[n3];

        cv::Point3d fnormal = FindFaceNormal(pt1, pt2, pt3, normal1, normal2, normal3); //knowing the direction and normalized

        face_normals.at<double>(0,i) = fnormal.x;
        face_normals.at<double>(1,i) = fnormal.y;
        face_normals.at<double>(2,i) = fnormal.z;
        face_normals.at<double>(3,i) = 0;

    }

    return face_normals;
};

cv::Mat ToolModel::getFaceCentroid(const std::vector< std::vector<int> > &input_faces, const std::vector< cv::Point3d > &input_vertices){
    int face_size = input_faces.size();
    cv::Mat face_centroids(4, face_size, CV_64FC1 );

    for (int i = 0; i < face_size; ++i)
    {
        int v1 = input_faces[i][0];
        int v2 = input_faces[i][1];
        int v3 = input_faces[i][2];
        int n1 = input_faces[i][3];
        int n2 = input_faces[i][4];
        int n3 = input_faces[i][5];

        cv::Point3d pt1 = input_vertices[v1];
        cv::Point3d pt2 = input_vertices[v2];
        cv::Point3d pt3 = input_vertices[v3];

        cv::Point3d face_point = pt1 + pt2 + pt3;
        face_point.x = face_point.x/3;
        face_point.y = face_point.y/3;
        face_point.z = face_point.z/3;
        face_point = Normalize(face_point);

        face_centroids.at<double>(0,i) = face_point.x;
        face_centroids.at<double>(1,i) = face_point.y;
        face_centroids.at<double>(2,i) = face_point.z;
        face_centroids.at<double>(3,i) = 1;

    }

    return face_centroids;

};
cv::Point3d ToolModel::crossProduct(cv::Point3d &vec1, cv::Point3d &vec2){    //3d vector

    cv::Point3d res_vec;
    res_vec.x = vec1.y * vec2.z - vec1.z * vec2.y;
    res_vec.y = vec1.z * vec2.x - vec1.x * vec2.z;
    res_vec.z = vec1.x * vec2.y - vec1.y * vec2.x;

    return res_vec;
};

double ToolModel::dotProduct(cv::Point3d &vec1, cv::Point3d &vec2){
    double dot_res;
    dot_res = vec1.x*vec2.x + vec1.y*vec2.y + vec1.z*vec2.z;

    return dot_res;
};

double ToolModel::dotProductMat(cv::Mat &vec1, cv::Mat &vec2){  //should be in 4 by 1

    double dot_product;
    dot_product = vec1.at<double>(0,0) * vec2.at<double>(0,0) + vec1.at<double>(1,0) * vec2.at<double>(1,0) + vec1.at<double>(2,0) * vec2.at<double>(2,0);

    return dot_product;

};
cv::Point3d ToolModel::Normalize(cv::Point3d &vec1){
    cv::Point3d norm_res;
    
    double norm = vec1.x*vec1.x + vec1.y*vec1.y + vec1.z*vec1.z;
    if (norm == 0.00000)
     {
        //ROS_ERROR("Trying to normalize a zero vector!");
     }
     else{
        norm = pow(norm, 0.5);

        norm_res.x = vec1.x/norm;
        norm_res.y = vec1.y/norm;
        norm_res.z = vec1.z/norm;

        return norm_res;
     } 

};

cv::Point3d ToolModel::FindFaceNormal(cv::Point3d &input_v1, cv::Point3d &input_v2, cv::Point3d &input_v3,
                                     cv::Point3d &input_n1, cv::Point3d &input_n2, cv::Point3d &input_n3)
{
    cv::Point3d temp_v1;
    cv::Point3d temp_v2;

    temp_v1.x = input_v1.x - input_v2.x;    //let temp v1 be v1-v2
    temp_v1.y = input_v1.y - input_v2.y;
    temp_v1.z = input_v1.z - input_v2.z;

    temp_v2.x = input_v1.x - input_v3.x;    //let temp v1 be v1-v3
    temp_v2.y = input_v1.y - input_v3.y;
    temp_v2.z = input_v1.z - input_v3.z;

    cv::Point3d res = crossProduct(temp_v1, temp_v2);

    //cv::Point3d temp_Vnorm = input_n1 + input_n2 + input_n3;
    // if (dot_normal < 0 )  ///so if dot product is nagtive, flip the normal
    // {
    //     res = -res;
    // }

    double outward_normal_1 = dotProduct(res, input_n1);
    double outward_normal_2 = dotProduct(res, input_n2);
    double outward_normal_3 = dotProduct(res, input_n3);
    if ((outward_normal_1 < 0) || (outward_normal_2 < 0) || (outward_normal_2 < 0) )
    {
        res = -res;
    }

    res = Normalize(res);

    return res;  // knowing the direction

};


int ToolModel::Compare_vertex(std::vector<int> &vec1, std::vector<int> &vec2, std::vector<int> &match_vec ){
    int match_count = 0;
    //std::vector<int> match_vec;   //match_vec should contain both the matching count and the matched vertices

    if (vec1.size() != vec2.size())  ///face vectors
    {
         printf("Two vectors are not in the same size \n");
    }
    else{
            for (int j = 0; j < 3; ++j)
            {
                if (vec1[0] == vec2[j])
                {
                    match_count +=1;
                    match_vec.push_back(vec1[0]);

                }

            }

            for (int j = 0; j < 3; ++j)
            {
                if (vec1[1] == vec2[j])
                {
                    match_count +=1;
                    match_vec.push_back(vec1[1]);

                }

            }

            for (int j = 0; j < 3; ++j)
            {
                if (vec1[2] == vec2[j])
                {
                    match_count +=1;
                    match_vec.push_back(vec1[2]);

                }

            } 
    }

    return match_count;
};

/*******This function is to do transformations to the raw data from the loader, to offset each part*******/
void ToolModel::modify_model_(std::vector< glm::vec3 > &input_vertices, std::vector< glm::vec3 > &input_Vnormal, 
                              std::vector< cv::Point3d > &input_Vpts, std::vector< cv::Point3d > &input_Npts, double &offset, cv::Mat &input_Vmat){

    Convert_glTocv_pts(input_vertices, input_Vpts);
    ConvertInchtoMeters(input_Vpts);

    int size = input_Vpts.size();
    for (int i = 0; i < size; ++i)
    {
        input_Vpts[i].y = input_Vpts[i].y - offset;
    }
    Convert_glTocv_pts(input_Vnormal, input_Npts); //not using homogenous for v reight now
    ConvertInchtoMeters(input_Npts);   

    input_Vmat = cv::Mat(4, size,CV_64FC1);

    for (int i = 0; i < size; ++i)
    {
        input_Vmat.at<double>(0,i) = input_Vpts[i].x;
        input_Vmat.at<double>(1,i) = input_Vpts[i].y;
        input_Vmat.at<double>(2,i) = input_Vpts[i].z;
        input_Vmat.at<double>(3,i) = 1;

    }
};

/*This function is to generate a tool model, contains the following info:
traslation, raotion, new z axis, new x axis*/
ToolModel::toolModel ToolModel::setRandomConfig(const toolModel &initial, double stdev, double mean)
{


	toolModel newTool = initial;  //BODY part is done here

    /******testing section here********/
    newTool.theta_ellipse = 0.0;
    newTool.theta_grip_1 = 0.0;
    newTool.theta_grip_2 = 0.0;

    cv::Mat I = cv::Mat::eye(3,3,CV_64FC1);
    /******testing section here********/

	//create normally distributed random number with the given stdev and mean

	//TODO: pertub all translation components
/*	newTool.tvec_cyl(0) += randomNumber(stdev,mean); 
	newTool.tvec_cyl(1) += randomNumber(stdev,mean);
	newTool.tvec_cyl(2) += randomNumber(stdev,mean);
	
	double angle = randomNumber(stdev,mean);
	newTool.rvec_cyl(0) += (angle/10.0)*1000.0; //rotation on x axis +/-5 degrees

	angle = randomNumber(stdev,mean);
	newTool.rvec_cyl(0) += (angle/10.0)*1000.0; ////rotation on x axis +/-5 degrees

	angle = randomNumber(stdev,mean);
	newTool.rvec_cyl(2) += (angle/10.0)*1000.0; //rotation on z axis +/-5 degrees*/


    /**************smaple the angles of the joints**************/
	//-90,90//
/*	angle = randomNumber(stdev,mean);
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
		newTool.theta_grip_1 = newTool.theta_grip_2 - randomNumber(stdev,mean);*/


   /***********computations for ellipse kinematics**********/

    cv::Mat q_temp(4,1,CV_64FC1);

    q_temp = transformPoints(q_ellipse,cv::Mat(initial.rvec_cyl),cv::Mat(initial.tvec_cyl)); 

    newTool.tvec_elp(0) = q_temp.at<double>(0,0);
    newTool.tvec_elp(1) = q_temp.at<double>(1,0);
    newTool.tvec_elp(2) = q_temp.at<double>(2,0);       

    newTool.rvec_elp(0) = newTool.rvec_cyl(0); //roll angle should be the same.
    newTool.rvec_elp(1) = newTool.rvec_cyl(1); //pitch angle should be the same.
    newTool.rvec_elp(2) = newTool.rvec_cyl(2) + newTool.theta_ellipse; //yaw angle is plus the theta_ellipse

   /***********computations for gripper kinematics**********/

    cv::Mat test_gripper(3,1,CV_64FC1);
    test_gripper.at<double>(0,0) = 0;
    test_gripper.at<double>(1,0) = offset_gripper - 0.4522;  //
    test_gripper.at<double>(2,0) = 0;
    // q_temp = transformPoints(q_gripper,cv::Mat(initial.rvec_elp),cv::Mat(initial.tvec_cyl));
    cv::Mat w_x(4,1,CV_64FC1);
    cv::Mat w_y(4,1,CV_64FC1);
    cv::Mat w_z(4,1,CV_64FC1);

    w_x.at<double>(0) = 1;
    w_x.at<double>(1) = 0;
    w_x.at<double>(2) = 0;

    w_y.at<double>(0) = 0;
    w_y.at<double>(1) = 1;
    w_y.at<double>(2) = 0;

    w_z.at<double>(0) = 0;
    w_z.at<double>(1) = 0;
    w_z.at<double>(2) = 1;

    cv::Mat skew_x = computeSkew(w_x);
    cv::Mat skew_y = computeSkew(w_y);
    cv::Mat skew_z = computeSkew(w_z);

    cv::Mat roll_mat = I + sin(newTool.rvec_elp(0)) * skew_x + (1-cos(newTool.rvec_elp(0))) * skew_x * skew_x;
    cv::Mat pitch_mat = I + sin(newTool.rvec_elp(1)) * skew_y + (1-cos(newTool.rvec_elp(1))) * skew_y * skew_y;
    cv::Mat yaw_mat = I + sin(newTool.rvec_elp(2)) * skew_z + (1-cos(newTool.rvec_elp(2))) * skew_z * skew_z;
    cv::Mat rotation_mat = yaw_mat*pitch_mat*roll_mat;

    cv::Mat q_rotation = rotation_mat * q_gripper;

    newTool.tvec_grip1(0) = q_rotation.at<double>(0,0) + newTool.tvec_elp(0);
    newTool.tvec_grip1(1) = q_rotation.at<double>(1,0) + newTool.tvec_elp(1);
    newTool.tvec_grip1(2) = q_rotation.at<double>(2,0) + newTool.tvec_elp(2);

    newTool.rvec_grip1(0) = newTool.rvec_elp(0) + newTool.theta_grip_1;  //roll angle is plus the theta_gripper
    newTool.rvec_grip1(1) = newTool.rvec_elp(1);
    newTool.rvec_grip1(2) = newTool.rvec_elp(2);    

    /*gripper 2*/

    newTool.tvec_grip2(0) = newTool.tvec_grip1(0);
    newTool.tvec_grip2(1) = newTool.tvec_grip1(1);
    newTool.tvec_grip2(2) = newTool.tvec_grip1(2);

    newTool.rvec_grip2(0) = newTool.rvec_elp(0) + newTool.theta_grip_2;  //roll angle is plus the theta_gripper
    newTool.rvec_grip2(1) = newTool.rvec_elp(1);
    newTool.rvec_grip2(2) = newTool.rvec_elp(2);  

 
	return newTool;
};

cv::Mat ToolModel::computeSkew(cv::Mat &w){
    cv::Mat skew(3,3, CV_64FC1);
    skew.at<double>(0,0) = 0;
    skew.at<double>(1,0) = w.at<double>(2,0);
    skew.at<double>(2,0) = -w.at<double>(1,0);
    skew.at<double>(0,1) = -w.at<double>(2,0);
    skew.at<double>(1,1) = 0;
    skew.at<double>(2,1) = w.at<double>(0,0);
    skew.at<double>(0,2) = w.at<double>(1,0);
    skew.at<double>(1,2) = -w.at<double>(0,0);
    skew.at<double>(2,2) = 0;

    return skew;

};


/****render a rectangle contains the tool model*****/
cv::Rect ToolModel::renderTool(cv::Mat &image, const toolModel &tool, cv::Mat &CamMat, const cv::Mat &P, cv::OutputArray jac ){

    cv::Rect ROI; // rectanle that contains tool model
    cv::Rect cropped; //cropped image to speed up the process

    /****************project points ste up***************************/

    int padding =10; //add 10pixels of padding for cropping
    cv::Point2d XY_max(-10000,-10000); //minimum of X and Y
    cv::Point2d XY_min(10000,10000); //maximum of X and Y


/***********for cylinder********/

//try to project at one time

// reprojectPoints(body_ver_pts, body_pts_, P, tool.rvec_cyl, tool.tvec_cyl);
// for (int i = 0; i < cyl_size; ++i)
// {
//         tool_pts_[i] = body_pts_[i];

//         //find max/min X and Y for cropping purpose
//         if(tool_pts_[i].x > XY_max.x) XY_max.x = tool_pts_[i].x;
//         if(tool_pts_[i].y > XY_max.y) XY_max.y = tool_pts_[i].y;
//         if(tool_pts_[i].x < XY_min.x) XY_min.x = tool_pts_[i].x;
//         if(tool_pts_[i].y < XY_min.y) XY_min.y = tool_pts_[i].y;
    
// }

Compute_Silhouette(body_faces, body_neighbors, body_Vmat, bodyFace_normal, bodyFace_centroid, CamMat, image, cv::Mat(tool.rvec_cyl), cv::Mat(tool.tvec_cyl), P, jac, XY_max, XY_min);
//Compute_Silhouette(body_faces, body_neighbors, body_Vpts, body_Npts, CamMat, image, cv::Mat(tool.rvec_cyl), cv::Mat(tool.tvec_cyl), P, jac, XY_max, XY_min);

// /*********for ellipse***********/
// Compute_Silhouette(ellipse_faces, ellipse_neighbors, ellipse_Vpts, ellipse_Npts, CamMat, image, cv::Mat(tool.rvec_elp), cv::Mat(tool.tvec_elp), P, jac, XY_max, XY_min);
// // /************for gripper 1************/
// Compute_Silhouette(griper1_faces, griper1_neighbors, griper1_Vpts, griper1_Npts, CamMat, image, cv::Mat(tool.rvec_grip1), cv::Mat(tool.tvec_grip1), P, jac, XY_max, XY_min);

// // /*************for gripper 2**************/
// Compute_Silhouette(griper2_faces, griper2_neighbors, griper2_Vpts, griper2_Npts, CamMat, image, cv::Mat(tool.rvec_grip2), cv::Mat(tool.tvec_grip2), P, jac, XY_max, XY_min);


    /***done projecting***/
    //connect points with lines to draw the rendered needle 
    // the rendering is for white on a floating point image.
    // for(int i=0; i<(segmentCount-1); i++)
    // {
    //     //draw the line
    //     cv::line(image, p_pts[i], p_pts[i+1], cv::Scalar(1,1,1), size, 8, 0);
    // }
    //draw circle to the base
    //cv::circle(image, p_pts[0], 5, cv::Scalar(1,0,0), size, 8, 0);



    //shape the rectangle that captures the rendered needle
    ROI.width = abs(static_cast<int>(XY_max.x-XY_min.x))+2*padding;
    ROI.height = abs(static_cast<int>(XY_max.y-XY_min.y))+2*padding;
    ROI.x = XY_min.x-padding;
    ROI.y = XY_min.y-padding;

    return ROI;

};

// double ToolModel::calculateMatchingScore(cv::Mat &toolImage, const cv::Mat &segmentedImage, cv::Rect &ROI, bool displayPause)
// {
            

//     double matchingScore(0.0);
    
//     cv::Mat ROI_toolImage = toolImage(ROI); //crop tool image
//     cv::Mat ROI_segmentedImage = segmentedImage(ROI); //crop segmented image


//     cv::Mat product; //elementwise product of images
//     cv::Mat toolImageGrey; //grey scale of toolImage since tool image has 3 channels
//     cv::Mat toolImFloat; //Float data type of grey scale tool image
//     cv::Mat toolImFloatBlured; //Float data type of grey scale blurred toolImage

//     cv::cvtColor(ROI_toolImage,toolImageGrey,CV_BGR2GRAY); //convert it to grey scale



//     toolImageGrey.convertTo(toolImFloat, CV_32FC1); // convert grey scale to float



//     //blur imtoolfloat
//     cv::GaussianBlur(toolImFloat,toolImFloatBlured, cv::Size(9,9),2,2);

//     imshow("blurred image", toolImFloatBlured);


//     toolImFloatBlured /= 255; //scale the blurred image



//     cv::Mat result(1,1,CV_32FC1);
//     cv::matchTemplate(ROI_segmentedImage,toolImFloatBlured,result,CV_TM_CCORR);
//     matchingScore = static_cast<double> (result.at< float >(0));

//     return matchingScore;
// }

/*************reproject a single point without rvec and tvec, and no jac, FOR THE BODY COORD TRNSFORMATION*******************/
cv::Point2d ToolModel::reproject(const cv::Point3d &point, const cv::Mat &P)
{


    cv::Mat prjPoints(4,1,CV_64FC1);
    cv::Mat results(3,1,CV_64FC1);
    cv::Point2d output;

    cv::Mat ptMat(4,1,CV_64FC1);
    ptMat.at<double>(0,0) = point.x;
    ptMat.at<double>(1,0) = point.y;
    ptMat.at<double>(2,0) = point.z;
    ptMat.at<double>(3,0) = 1.0;

    prjPoints = ptMat;

    results = P*prjPoints;
    output.x = results.at<double>(0,0)/results.at<double>(2,0);
    output.y = results.at<double>(1,0)/results.at<double>(2,0);

    return output;
};

