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
using namespace std;

//constructor
ToolModel::ToolModel(){

    /****a identity matrix****/
	for(int i(0); i<3; i++){
		for(int j(0);j<3;j++){
			I[i][j] = 0.0;
		}
	}
	I[0][0] = 1.0;
	I[1][1] = 1.0;
	I[2][2] = 1.0;

    /****initialize the rotation and traslation points*****/
    q_1[0] = 0;
    q_1[1] = 0.04837;  //m,  48.37mm
    q_1[2] = 0;

    q_2[0] = 0;
    q_2[1] = 0.05784;  //measure from the model
    q_2[2] = 0;


    /****initialize the vertices fo different part of tools****/
	load_model_vertices("cylinder_part.obj", body_vertices );
	load_model_vertices("ellipse_contour.obj", ellipse_vertices );
	load_model_vertices("griper_1.obj", griper1_vertices );
	load_model_vertices("griper_2.obj", griper2_vertices );

};

double ToolModel::randomNumber(double stdev, double mean){
	boost::normal_distribution<> nd(mean, stdev);
	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor(rng, nd);
	double d = var_nor();

	return d;

}

//set zero configuratio for tool points;
void ToolModel::load_model_vertices(const char * path, std::vector< glm::vec3 > &out_vertices ){

    printf("Loading file %s...\n", path);

    FILE * file = fopen(path, "r");
    if( file == NULL ){
        printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
        return;
    }

    std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
    //std::vector< unsigned int > vertexIndices;
	std::vector< glm::vec3 > temp_vertices;
	std::vector< glm::vec2 > temp_uvs;
	std::vector< glm::vec3 > temp_normals;

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
            temp_vertices.push_back(vertex);
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
            temp_normals.push_back(normal);
        }
        else if ( strcmp( lineHeader, "f" ) == 0 ){
        std::string vertex1, vertex2, vertex3;
        unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
        int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
        if (matches != 9){
            printf("File can't be read by our simple parser : ( Try exporting with other options\n");
            return;
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
            
        }
    }

    // For each vertex of each triangle
    for( unsigned int i=0; i<vertexIndices.size(); i++ ){

        // Get the indices of its attributes
        unsigned int vertexIndex = vertexIndices[i];
        
        // Get the attributes thanks to the index
        glm::vec3 vertex = temp_vertices[ vertexIndex-1 ];
        
        // Put the attributes in buffers
        out_vertices.push_back(vertex);
    
    }

    cout<<"size"<< out_vertices.size()<<endl;
    printf("loaded file %s successfully.\n", path);

    //return true;
};

/*This function is to generate a tool model, contains the following info:
traslation, raotion, new z axis, new x axis*/
ToolModel::toolModel ToolModel::setRandomConfig(const toolModel &initial, double stdev, double mean)
{
	// copy initial toolModel
	toolModel newTool = initial;

	//create normally distributed random number with the given stdev and mean

	//TODO: pertub all translation components
	newTool.tvec_cyl(0) += randomNumber(stdev,mean); 
	newTool.tvec_cyl(1) += randomNumber(stdev,mean);
	newTool.tvec_cyl(2) += randomNumber(stdev,mean);
	
	double angle = randomNumber(stdev,mean);
	newTool.rvec_cyl(0) += (angle/10.0)*1000.0; //rotation on x axis +/-5 degrees

	angle = randomNumber(stdev,mean);
	newTool.rvec_cyl(0) += (angle/10.0)*1000.0; ////rotation on x axis +/-5 degrees

	angle = randomNumber(stdev,mean);
	newTool.rvec_cyl(2) += (angle/10.0)*1000.0; //rotation on z axis +/-5 degrees


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


   /***********need computations for forward kinematics**********/

    roll_mat[0][0] = 1;
    roll_mat[0][1] = 0;
    roll_mat[0][2] = 0;
    roll_mat[1][0] = 0;
    roll_mat[1][1] = cos(newTool.rvec_cyl(0));
    roll_mat[1][2] = -sin(newTool.rvec_cyl(0));
    roll_mat[2][0] = 0;
    roll_mat[2][1] = sin(newTool.rvec_cyl(0));
    roll_mat[2][2] = cos(newTool.rvec_cyl(0));

    pitch_mat[0][0] = cos(newTool.rvec_cyl(1));
    pitch_mat[0][1] = 0;
    pitch_mat[0][2] = sin(newTool.rvec_cyl(1));
    pitch_mat[1][0] = 0;
    pitch_mat[1][1] = 1;
    pitch_mat[1][2] = 0;
    pitch_mat[2][0] = sin(newTool.rvec_cyl(1));
    pitch_mat[2][1] = 0;
    pitch_mat[2][2] = cos(newTool.rvec_cyl(1));

    yaw_mat[0][0] = cos(newTool.rvec_cyl(2));
    yaw_mat[0][1] = -sin(newTool.rvec_cyl(2));
    yaw_mat[0][2] = 0;
    yaw_mat[1][0] = sin(newTool.rvec_cyl(2));
    yaw_mat[1][1] = cos(newTool.rvec_cyl(2));
    yaw_mat[1][2] = 0;
    yaw_mat[2][0] = 0;
    yaw_mat[2][1] = 0;
    yaw_mat[2][2] = 1;


    rotation_mat = yaw_mat * pitch_mat * roll_mat;
    
    //w for the ellipse is z of rotation mat

    cv::Matx<double,3,1> w_z;   //for computational purpose
    //cv::Matx<double,3,1> w_x;
    
    w_z(0) = rotation_mat[0][2];
    w_z(1) = rotation_mat[1][2];
    w_z(2) = rotation_mat[2][2];

    //w for the two girpper part is x of rotation mat
    // newTool.w_x(0) = rotation_mat[0][0];
    // newTool.w_x(1) = rotation_mat[1][0];
    // newTool.w_x(2) = rotation_mat[2][0];


    /*get ellipse orininal point */
    glm::vec3 q_temp;
    double temp_point;

    glm::vec3 q_ellipse;
    glm::vec3 q_gripper;

    q_temp = rotation_mat * q_1;
    temp_point = newTool.tvec_cyl(0);
    q_ellipse[0] = q_temp[0] + temp_point;

    temp_point = newTool.tvec_cyl(1);
    q_ellipse[1] = q_temp[1] + temp_point;

    temp_point = newTool.tvec_cyl(2);
    q_ellipse[2] = q_temp[2] + temp_point;


    glm::mat3 ellip_Rotation;
    glm::mat3 wz_mat;
    glm::mat3 wz_mat_1;
    glm::mat3 wz_mat_2;

    wz_mat[0][0] = 0;
    wz_mat[0][1] = -w_z(2);
    wz_mat[0][2] = w_z(1);
    wz_mat[1][0] = w_z(2);
    wz_mat[1][1] = 0;
    wz_mat[1][2] = -w_z(0);
    wz_mat[2][0] = -w_z(1);
    wz_mat[2][1] = w_z(0);
    wz_mat[2][2] = 0;

    for(int i(0); i<3; i++){
        for(int j(0);j<3;j++){
            wz_mat_1[i][j] = wz_mat[i][j] * sin(newTool.theta_ellipse);
        }
    }

    wz_mat_2 =  wz_mat * wz_mat;
    for(int i(0); i<3; i++){
        for(int j(0);j<3;j++){
            wz_mat_2[i][j] = wz_mat_2[i][j] * (1-cos(newTool.theta_ellipse));
        }
    }
    /*ellip_rotation = I * wz_mat*sin(theta) * wz_mat^2 * (1-cos(theta))*/
    ellip_Rotation = I + wz_mat_1 + wz_mat_2;

    q_temp = rotation_mat * ellip_Rotation * q_2;

    temp_point = newTool.tvec_cyl(0);
    q_gripper[0] = q_temp[0] + temp_point;

    temp_point = newTool.tvec_cyl(1);
    q_gripper[1] = q_temp[1] + temp_point;

    temp_point = newTool.tvec_cyl(2);
    q_gripper[2] = q_temp[2] + temp_point;

    /*now compute the t and r for ellipse and grippers*/

    //translation vectors
    newTool.tvec_elp(0) = q_ellipse[0];
    newTool.tvec_elp(1) = q_ellipse[1];
    newTool.tvec_elp(2) = q_ellipse[2];

    newTool.tvec_grip1(0) = q_gripper[0];
    newTool.tvec_grip1(1) = q_gripper[1];
    newTool.tvec_grip1(2) = q_gripper[2];

    newTool.tvec_grip2(0) = q_gripper[0];
    newTool.tvec_grip2(1) = q_gripper[1];
    newTool.tvec_grip2(2) = q_gripper[2];

    //rotation vetors, Euler angle
    newTool.rvec_elp(0) = newTool.rvec_cyl(0); //roll angle should be the same.
    newTool.rvec_elp(1) = newTool.rvec_cyl(1); //pitch angle should be the same.
    newTool.rvec_elp(2) = newTool.rvec_cyl(2) + newTool.theta_ellipse; //yaw angle is plus the theta_ellipse

    newTool.rvec_grip1(0) = newTool.rvec_cyl(0) + newTool.theta_grip_1;  //roll angle is plus the theta_gripper
    newTool.rvec_grip1(1) = newTool.rvec_cyl(1);
    newTool.rvec_grip1(2) = newTool.rvec_cyl(2);

    newTool.rvec_grip2(0) = newTool.rvec_cyl(0) + newTool.theta_grip_2;  //roll angle is plus the theta_gripper
    newTool.rvec_grip2(1) = newTool.rvec_cyl(1);
    newTool.rvec_grip2(2) = newTool.rvec_cyl(2);    



	return newTool;
}

/****render a rectangle contains the tool model*****/
cv::Rect ToolModel::renderTool(cv::Mat &image, const toolModel &tool, const cv::Mat &P, int size,cv::OutputArray toolPts, cv::OutputArray jac){

    cv::Rect ROI; // rectanle that contains tool model
    cv::Rect cropped; //cropped image to speed up the process

    //points stored in the glm::vec3 

    /**********temp vector to store points in new image***********/
    // std::vector<cv::Point2d> p_pts;
    // p_pts.resize(segmentCount+1);
    // cv::Mat jacMat;
    // if(jac.needed())
    // {
    //     jac.create(2*(segmentCount+1),6,CV_64FC1);
    //     jacMat = jac.getMat();
    // }

    // cv::Mat temp_jac;

    std::vector< glm::vec2 > body_pts_img;
    std::vector< glm::vec2 > elp_pts_img;
    std::vector< glm::vec2 > grip1_pts_img;
    std::vector< glm::vec2 > grip2_pts_img;

    body_pts_img.resize(body_vertices.size());
    elp_pts_img.resize(ellipse_vertices.size());
    grip1_pts_img.resize(griper1_vertices.size());
    grip2_pts_img.resize(griper2_vertices.size());

    //also need four of jacs
    // cv::Mat jacMat;
    // if(jac.needed())
    // {
    //     jac.create(2*(segmentCount+1),6,CV_64FC1);
    //     jacMat = jac.getMat();
    // }

    // cv::Mat temp_jac;


    /****************project points***************************/

    int padding(10); //add 10pixels of padding for cropping
    cv::Point2d XY_max(-10000,-10000); //minimum of X and Y
    cv::Point2d XY_min(10000,10000); //maximum of X and Y

    for(int i=0; i<(segmentCount+1); i++){
        p_pts[i] = reprojectPoint(pts[i], P, cv::Mat(needle.rvec), cv::Mat(needle.tvec), temp_jac);
        if(jac.needed())
        {
            //make the jacobian ((2x) x 6)
            temp_jac.colRange(3,9).copyTo(jacMat.rowRange(i*2,i*2+2));
        }

        //find max/min X and Y for cropping purpose
        if(p_pts[i].x > XY_max.x) XY_max.x = p_pts[i].x;
        if(p_pts[i].y > XY_max.y) XY_max.y = p_pts[i].y;
        if(p_pts[i].x < XY_min.x) XY_min.x = p_pts[i].x;
        if(p_pts[i].y < XY_min.y) XY_min.y = p_pts[i].y;
    }

    //connect points with lines to draw the rendered needle 
    // the rendering is for white on a floating point image.
    for(int i=0; i<(segmentCount-1); i++)
    {
        //draw the line
        cv::line(image, p_pts[i], p_pts[i+1], cv::Scalar(1,1,1), size, 8, 0);
    }
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