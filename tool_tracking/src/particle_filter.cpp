 #include <ros/ros.h>
 #include <tool_tracking/particle_filter.h>  //everything inside here
 #include <opencv2/calib3d/calib3d.hpp>


 using namespace std;

 ParticleFilter::ParticleFilter(ros::NodeHandle* nodehandle):
     nh_(*nodehandle), numParticles(2), toolSize(2), perturbStd(0.001), newToolModel(Cam)
{

    /****initial position guess
	everything here is in meters*****/
	initial.tvec_cyl(0) = 0.0;
	initial.tvec_cyl(1) = 0.0;
	initial.tvec_cyl(2) = 0.0;
	initial.rvec_cyl(0) = 0.0;
	initial.rvec_cyl(1) = 0.0;   //better make everything zero
	initial.rvec_cyl(2) = 0.0;

    /****need to subscrib this***/
    Cam = cv::Mat(4,4,CV_64FC1);
    Cam.at<double>(0,0) = 1;
    Cam.at<double>(1,0) = 0;
    Cam.at<double>(2,0) = 0;
    Cam.at<double>(3,0) = 0;    

    Cam.at<double>(0,1) = 0;
    Cam.at<double>(1,1) = -1;
    Cam.at<double>(2,1) = 0;
    Cam.at<double>(3,1) = 0;

    Cam.at<double>(0,2) = 0;
    Cam.at<double>(1,2) = 0;
    Cam.at<double>(2,2) = -1;
    Cam.at<double>(3,2) = 0;

    Cam.at<double>(0,3) = 0.0;   //should be in meters
    Cam.at<double>(1,3) = 0.1;
    Cam.at<double>(2,3) = 0.2;  // cannot have z = 0 for reprojection, camera_z must be always point to object
    Cam.at<double>(3,3) = 1;

	//initialize particles by randomly assigning around the initial guess
	initializeParticles();
	// ROS_INFO("---- Initialization is done---");

	// initialization, just basic black image ??? how to get the size of the image
	toolImage_left = cv::Mat::zeros(480, 640, CV_8UC3);
	toolImage_right = cv::Mat::zeros(480, 640, CV_8UC3);

    //timer = nh_.createTimer(ros::Duration(0.01), &ParticleFilter::timerCallback, this);
};

ParticleFilter::~ParticleFilter()
{

};

 //To show results and debug TODO:
/* void ParticleFilter::timerCallback(const ros::TimerEvent&)
 {
     std::vector<cv::Mat> *trackingimages;
     trackingimages = &trackingImages;  //copy here

     cv::imshow( "rendered image: LEFT", trackingimages[0]);
     cv::imshow( "rendered image: RIGHT", trackingimages[1]);
     cv::waitKey(10);

 };*/

void ParticleFilter::initializeParticles()
{
	ROS_INFO("---- Initialize particle is called---");
	particles.resize(numParticles); //initialize particle array
	matchingScores.resize(numParticles); //initialize matching score array
	particleWeights.resize(numParticles); //initialize particle weight array
	//generate random needle 
	for(int i(0); i<numParticles; i++)
	{
		particles[i] = newToolModel.setRandomConfig(initial, 1, 0);
	}

};

 std::vector<cv::Mat> ParticleFilter::trackingTool(const cv::Mat &bodyVel, const cv::Mat &segmented_left, const cv::Mat &segmented_right,const cv::Mat &P_left, const cv::Mat &P_right){

     cv::Mat segmentedImage_left = segmented_left.clone();
     cv::Mat segmentedImage_right = segmented_right.clone();

     std::vector<cv::Mat> trackingImages;
     trackingImages.resize(2);

     double maxScore = -1.0; //track the maximum scored particle
     int maxScoreIdx = -1; //maximum scored particle index
     double totalScore = 0.0; //total score
     double left(0.0); //matching score for the left image
     double right(0.0); //matching score for the right image

     /***do the sampling and get the matching score***/
     for (int i = 0; i <numParticles; ++i) {
         toolImage_left.setTo(0); //reset image for every start of an new loop
         ROI_left = newToolModel.renderTool(toolImage_left, particles[i], Cam, P_left); //first get the rendered image using 3d model of the tool
         left = newToolModel.calculateMatchingScore(toolImage_left, segmented_left, ROI_left);  //get the matching score

         toolImage_right.setTo(0); //resetimage
         ROI_right = newToolModel.renderTool(toolImage_right, particles[i], Cam, P_right);
         right = newToolModel.calculateMatchingScore(toolImage_right, segmented_right, ROI_right);

         matchingScores[i] = sqrt(pow(left,2) + pow(right,2));

         if(matchingScores[i]>maxScore)
         {
             maxScore = matchingScores[i];
             maxScoreIdx = i;
         }
         totalScore += matchingScores[i];
     }

     /***you may wanna do this in a different stream, TODO: ***/
     ROS_INFO_STREAM(maxScore);  //debug

     newToolModel.renderTool(segmentedImage_left, particles[maxScoreIdx], Cam, P_left);  //render in segmented image, no need to get the ROI
     newToolModel.renderTool(segmentedImage_right, particles[maxScoreIdx], Cam, P_right);

     trackingImages[0] = segmentedImage_left;
     trackingImages[1] = segmentedImage_right;

     /***calculate weights using matching score and do the resampling***/
     for (int j = 0; j <numParticles; ++j) { // normalize the weights
         particleWeights[j] = (matchingScores[j]/totalScore);
     }

     std::vector<ToolModel::toolModel>  oldParticles = particles;
     resampleLowVariance(oldParticles, particleWeights, particles); //each time will clear the particles and resample them

     double dT = 0.1; //sampling rate

     /***update particles, based on the given body vel and updating rate***/
     updateParticles(bodyVel, dT);


     //if there is no movement, we need to perturb it
     if(bodyVel.at<double>(0,0) == 0.0 && bodyVel.at<double>(1,0) == 0.0 && bodyVel.at<double>(5,0) == 0.0)
     {
         //TODO: look for matching score, if it is good assign smaller perturbation standard deviation
         if(maxScore>0.91)
         {
             perturbStd = 0.0;
         }

         for(int i(0); i<particles.size(); i++)
         {
             particles[i] = newToolModel.setRandomConfig(particles[i],perturbStd,0.0);
         }
     }

     return trackingImages;
 };

 void ParticleFilter::updateParticles(const cv::Mat &bodyVel, double &updateRate){ //get particles and update them based on given spatial velocity

     cv::Mat Rot = cv::Mat::zeros(3,3,CV_64F);
     cv::Mat p = cv::Mat::zeros(3,1,CV_64F);
     cv::Mat particleFrame=cv::Mat::eye(4,4,CV_64F);

     cv::Mat spatialVel = cv::Mat::zeros(6,1,CV_64F);
     cv::Mat updatedParticleFrame=cv::Mat::eye(4,4,CV_64F);

     cv::Mat I = cv::Mat::eye(3,3,CV_64F);

     for (int k = 0; k < particles.size(); ++k) {

         cv::Rodrigues(particles[k].rvec_cyl, Rot); //get rotation mat from cylinder
         p.at<double>(0,0)=particles[k].tvec_cyl(0); //get translation vec from cylinder
         p.at<double>(1,0)=particles[k].tvec_cyl(1);
         p.at<double>(2,0)=particles[k].tvec_cyl(2);

         Rot.copyTo(particleFrame.colRange(0,3).rowRange(0,3));
         p.copyTo(particleFrame.colRange(3,4).rowRange(0,3));

         //calculate spatial velocity
         spatialVel = adjoint(particleFrame) * bodyVel;
         cv::Mat v = spatialVel.colRange(0,1).rowRange(0,3); //translation velocity
         cv::Mat w = spatialVel.colRange(0,1).rowRange(3,6); //rotational velocity

         cv::Mat move = cv::Mat::eye(4,4,CV_64F);

         if(w.at<double>(0,0) == 0.0 && w.at<double>(1,0) == 0.0 && w.at<double>(2,0) == 0.0){  //pure translation
             cv::Mat vdt = v * updateRate;
             vdt.copyTo(move.colRange(3,4).rowRange(0,3));
         } else{
             cv::Mat vtil = v*updateRate;
             cv::Mat wtil = w*updateRate;

             double M = cv::norm(wtil);
             cv::Mat v_bar = vtil/M;  //h = w*v//||w||
             cv::Mat w_bar = wtil/M;

             cv::Mat w_hat = newToolModel.computeSkew(w_bar);
             cv::Mat rotation = I+w_hat*sin(M)+(w_hat*w_hat)*(1-cos(M));
             cv::Mat trans = (I-rotation)*(w_bar.cross(v_bar)+w_bar*w_bar.t()*v_bar*M);

             rotation.copyTo(move.colRange(0,3).rowRange(0,3));
             trans.copyTo(move.colRange(3,4).rowRange(0,3));
         }

         /***update the cylinder pose***/
         updatedParticleFrame = move * particleFrame;

         //convert rotation matrix to Rodrigues
         cv::Mat tempR =cv::Mat::zeros(3,3,CV_64F);
         cv::Mat updateR =cv::Mat::zeros(3,1,CV_64F);
         cv::Mat updateT =cv::Mat::zeros(3,1,CV_64F);

         updateT = updatedParticleFrame.colRange(3,4).rowRange(0,3);
         tempR = updatedParticleFrame.colRange(0,3).rowRange(0,3);
         cv::Rodrigues(tempR, updateR);

         particles[k].tvec_cyl(0) = updateT.at<double>(0,0);
         particles[k].tvec_cyl(1) = updateT.at<double>(1,0);
         particles[k].tvec_cyl(2) = updateT.at<double>(2,0);
         particles[k].rvec_cyl(0) = updateR.at<double>(0,0);
         particles[k].rvec_cyl(1) = updateR.at<double>(1,0);
         particles[k].rvec_cyl(2) = updateR.at<double>(2,0);

         /***according to the cylinder pose, update ellipse and grippers pose***/
         newToolModel.computeModelPose(particles[k], 0.0, 0.0, 0.0); // no need to change relative angles

     }
 };

 cv::Mat ParticleFilter::adjoint(cv::Mat& G)
 {
     cv::Mat adjG = cv::Mat::zeros(6,6,CV_64F);

     cv::Mat Rot = G.colRange(0,3).rowRange(0,3);
     cv::Mat p = G.colRange(3,4).rowRange(0,3);

     cv::Mat p_skew = newToolModel.computeSkew(p);
     //upper right corner
     cv::Mat temp = p_skew*Rot;
     Rot.copyTo(adjG.colRange(0,3).rowRange(0,3));
     Rot.copyTo(adjG.colRange(3,6).rowRange(3,6));
     temp.copyTo(adjG.colRange(3,6).rowRange(0,3));
     return adjG;
 };

 void ParticleFilter::resampleLowVariance(const std::vector<ToolModel::toolModel> &sampleModel, const std::vector<double> &particleWeight,  std::vector<ToolModel::toolModel> &results)
 {
     int M = sampleModel.size(); //total number of particles
     double max = 1.0/M;
     double min(0.0);
     double U(0.0);
     double r(0.0);
     double c(0.0);

     c = particleWeight[0]; //first particle weight
     int idx(0); //index
     results.clear(); //final particles (how to clear)
     r = newToolModel.randomNum(min,max); //random number in range [0,1/M]

     for(int i(1); i<=M; i++)
     {
         U = r+((double)(i-1)*max);
         while(U>c)
         {
             idx +=1;
             c += particleWeight[idx];
         }
         results.push_back(sampleModel[idx]);
     }

 };

 std::vector<ToolModel::toolModel> ParticleFilter::perturb(const std::vector<ToolModel::toolModel> &particles, double stdev, double mean){

     std::vector<ToolModel::toolModel> result = particles; //resulting perturbed particles
     int size = result.size(); //size of the particle
     result.clear(); //final particles

     for(int i(0); i<size; i++)
     {
         result[i] = newToolModel.setRandomConfig(particles[i], stdev, mean);
     }

     return result;
 };
