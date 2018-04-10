
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/String.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cv.h"
#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>
#include <iostream>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <pub_msgs/where_msgs.h>
#include <pub_msgs/three_w_msgs.h>

using std::string;

static char TRACKER_ID[100]="Standby";
static char MESSAGE[100]="";

float SN_KINECT1_OFFSET_X = 0*1000.0;
//float SN_KINECT1_OFFSET_Y = (1.855-0.3)*1000;
float SN_KINECT1_OFFSET_Y = 1.8*1000.0;
float SN_KINECT1_OFFSET_Z = 0.0;

float SN_KINECT1_OFFSET_RPY_R 	= 0.0;
float SN_KINECT1_OFFSET_RPY_P 	= 0.0;
//if Y is Large, Y value decrease when moving to left
float SN_KINECT1_OFFSET_RPY_Y 	= 0.0;

tf::Matrix3x3 m1;
tf::Vector3 v1,v2;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

ros::Publisher  pub;

char Colors[][3] =
{
		{255,0,0},
		{0,255,0},
		{0,0,255},
		{155,0,100},
		{155,100,0},
		{0,100,155},
		{100,0,155},
		{0,155,100},
		{100,155,0},
		{100,155,100}
};
XnUInt32 nColors = 10;


IplImage 			image_rgb;
image_transport::Publisher img_pub;
static const char TOPIC_NAME[] = "/sn_kinect1/rgb/image_color";

char camera_name[32];

char quit;
int  nMode;
bool bFirst;
int User_action[15];

/*
void imageCb_rgb(const sensor_msgs::ImageConstPtr& img_rgb)
{
	//	printf("test\n");
	//	ROS_INFO("Image Callback!!");
	cv_bridge::CvImagePtr cv_ptr;
	try{cv_ptr = cv_bridge::toCvCopy(img_rgb, sensor_msgs::image_encodings::BGR8);}
	catch (cv_bridge::Exception& e){ROS_ERROR("cv_bridge exception: %s", e.what());return;}
	image_rgb = cv_ptr->image;
}
 */



void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) 
{
	ROS_INFO("New User %d", nId);
	sprintf(MESSAGE, "New User %d", nId);
	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
	sprintf(MESSAGE, "Lost User %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
	sprintf(MESSAGE, "Calibration for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
	if (bSuccess) {
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else {
		ROS_INFO("Calibration failed for user %d", nId);
		if (g_bNeedPose)
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		else
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
			
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) {
	ROS_INFO("Pose %s detected for user %d", strPose, nId);
	g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

     
void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id, XnPoint3D & Loc_val) {
	static tf::TransformBroadcaster br;

	XnSkeletonJointPosition joint_position;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
	double x = -joint_position.position.X / 1000.0;
	double y = joint_position.position.Y / 1000.0;
	double z = joint_position.position.Z / 1000.0;

	XnSkeletonJointOrientation joint_orientation;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

	XnFloat* m = joint_orientation.orientation.elements;
	KDL::Rotation rotation(m[0], m[1], m[2],
				m[3], m[4], m[5],
				m[6], m[7], m[8]);
	double qx, qy, qz, qw;
	rotation.GetQuaternion(qx, qy, qz, qw);

	char child_frame_no[128];
	snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(0, 0, 0));

	tf::Quaternion frame_rotation;
	frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
	change_frame.setRotation(frame_rotation);

	transform = change_frame * transform;
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));

	Loc_val.X = x;
	Loc_val.Y = y;
	Loc_val.Z = z;

	//ROS_WARN("%s :\t%.2f,\t%.2f,\t%.2f", child_frame_id.c_str(), x,y,z);
}

// What module judgment (1(ask), 2(sit or stand))
int action_judge(XnPoint3D p_Head, XnPoint3D p_Torso, XnPoint3D p_Sh_Left, XnPoint3D p_El_Left, XnPoint3D p_Ha_Left, XnPoint3D p_Sh_Right, XnPoint3D p_El_Right,XnPoint3D p_Ha_Right)
{
	//printf("%.2f - %.2f= %.2f\n", p_Head.Y, p_Torso.Y, fabs(p_Head.Y-p_Torso.Y));
	if     ((p_Head.Y < p_El_Left.Y)  || (p_Head.Y < p_Ha_Left.Y))	{ return 1; }	// ask (left hand)	
	else if((p_Head.Y < p_El_Right.Y) || (p_Head.Y < p_Ha_Right.Y))	{ return 1; }	// ask (right hand)
	else if((fabs(p_Head.Y-p_Torso.Y)<0.250))			{ return 4; }	// sleep
	else 								{ return 2; }	// sit or stand
}

void publishTransforms(const std::string& frame_id) 
{
	XnUserID users[15];
	XnUInt16 users_count = 15;
	g_UserGenerator.GetUsers(users, users_count);

	pub_msgs::where_msgs g_track;
	sensor_msgs::RegionOfInterest my_roi;
	geometry_msgs::Point32 my_point;
	std_msgs::String my_str;

	float X[15], Y[15], Z[15];
	int User_action[15];
	
	for(int i=0; i<15; i++)
	{
		User_action[i] = 0;
		X[i] = 0;
		Y[i] = 0;
		Z[i] = 0;
	}
	
	XnPoint3D p_Torso;
	p_Torso={0.0, 0.0, 0.0};
	for (int i = 0; i < users_count; ++i)
	{
		XnPoint3D p_Head;
		XnPoint3D p_Sh_Left, p_El_Left, p_Ha_Left;
		XnPoint3D p_Sh_Right, p_El_Right, p_Ha_Right;

		XnUserID user = users[i];
		XnPoint3D p;
		if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
			continue;
		
		//ROS_WARN("\nUSER: %d", users[i]);

		g_UserGenerator.GetCoM(users[i],p);
		
		publishTransform(user, XN_SKEL_HEAD,           frame_id, "head", p_Head);
		//publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
		publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso", p_Torso);

		publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder", p_Sh_Left);
		publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow", p_El_Left);
		publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand", p_Ha_Left);

		publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder", p_Sh_Right);
		publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow", p_El_Right);
		publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand", p_Ha_Right);
/*
		publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
		publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
		publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

		publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
		publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
		publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
*/

		// behavior decision
		User_action[users[i]] = action_judge(p_Head, p_Torso, p_Sh_Left, p_El_Left, p_Ha_Left, p_Sh_Right, p_El_Right, p_Ha_Right);
		//User Action: %d\n",i,User_action[i]);}
		//else{printf("sit or stand\n");}

/*
		btVector3 v1,v2;
//		m1.setEulerZYX(0.14,0.03,0.03);
		m1.setEulerZYX(SN_KINECT2_OFFSET_RPY_R,SN_KINECT2_OFFSET_RPY_P,SN_KINECT2_OFFSET_RPY_Y);

		float tmpX = (float)-p.X;
		float tmpY = (float)p.Y;
		float tmpZ = (float)p.Z;


		v1.setX(tmpX);v1.setY(tmpY);v1.setZ(tmpZ);
		v2	=	m1*v1;

		X[users[i]] = (float)v2.getX() + SN_KINECT2_OFFSET_X;
		Y[users[i]] = (float)v2.getY() + SN_KINECT2_OFFSET_Y;
		Z[users[i]] = (float)v2.getZ() + SN_KINECT2_OFFSET_Z;
*/
		X[users[i]] = (float)-p.X;
		Y[users[i]] = (float)p.Y;
		Z[users[i]] = (float)p.Z;
		

		XnPoint3D proj, real;
		proj.X = X[users[i]];
		proj.Y = Y[users[i]];
		proj.Z = Z[users[i]];

		g_DepthGenerator.ConvertProjectiveToRealWorld(1, &proj, &real);
		X[users[i]] = real.X;
		Y[users[i]] = real.Y;
		Z[users[i]] = real.Z;
//		sprintf(pos_text, "x before: %.2f, y before : %.2f, z before: %.2f", X[users[i]], Y[users[i]], Z[users[i]]);
		//X[users[i]] = (float)-p.X + SN_KINECT2_OFFSET_X;
		//Y[users[i]] = (float)p.Y + SN_KINECT2_OFFSET_Y;
		//Z[users[i]] = (float)p.Z + SN_KINECT2_OFFSET_Z;
		printf("SN_KINECT2  %f %f %f\n",X[users[i]],Y[users[i]],Z[users[i]]);
		printf("SN_KINECT2_skeleton  %f %f %f\n",p_Head.X,p_Head.Y,p_Head.Z);
	}

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;

	g_UserGenerator.GetUserPixels(0,sceneMD);

	unsigned short* depthPixels = (unsigned short*)sceneMD.Data();

	cv::Mat mat(480,640,CV_8UC1,cv::Scalar::all(0));
	int u_rx[15], u_lx[15], u_ry[15], u_ly[15];
	int tmp_u_rx=0, tmp_u_lx=0, tmp_u_ry=0, tmp_u_ly=0;
	int ii =0;

	IplImage* imagePixels;
	imagePixels = cvCreateImage(cvSize(640,480),8,3);

	for(int j=0; j<15; j++)	{
		u_rx[j] = 0;	u_lx[j] = 640;	u_ry[j] = 0;	u_ly[j] = 480;
	}

	for(int y=0;y<480;y++)
	{
		for(int x=0;x<640;x++,ii++)
		{
			imagePixels->imageData[(y*imagePixels->widthStep) + x*3+0] = 0;
			imagePixels->imageData[(y*imagePixels->widthStep) + x*3+1] = 0;
			imagePixels->imageData[(y*imagePixels->widthStep) + x*3+2] = 0;

			if (depthPixels[ii] > 0)
			{
				imagePixels->imageData[(y*imagePixels->widthStep) + x*3+0] = Colors[depthPixels[ii]%15][0];
				imagePixels->imageData[(y*imagePixels->widthStep) + x*3+1] = Colors[depthPixels[ii]%15][1];
				imagePixels->imageData[(y*imagePixels->widthStep) + x*3+2] = Colors[depthPixels[ii]%15][2];

				tmp_u_rx = u_rx[depthPixels[ii]%15];
				tmp_u_lx = u_lx[depthPixels[ii]%15];
				tmp_u_ry = u_ry[depthPixels[ii]%15];
				tmp_u_ly = u_ly[depthPixels[ii]%15];

				tmp_u_rx = std::max(tmp_u_rx,x);
				tmp_u_lx = std::min(tmp_u_lx,x);
				tmp_u_ry = std::max(tmp_u_ry,y);
				tmp_u_ly = std::min(tmp_u_ly,y);

				u_rx[depthPixels[ii]%15] = tmp_u_rx;
				u_lx[depthPixels[ii]%15] = tmp_u_lx;
				u_ry[depthPixels[ii]%15] = tmp_u_ry;
				u_ly[depthPixels[ii]%15] = tmp_u_ly;
			}
		}
	}

	int cnt=0;
	float tmp_x=0, tmp_y=0, tmp_z=0;
	for(int i=0; i<15; i++)
	{
		if((u_rx[i] - u_lx[i]) > 0 && (u_rx[i] - u_lx[i]) <= 640 && (u_ry[i] - u_ly[i]) > 0 && (u_ry[i] - u_ly[i]) <= 480)
		{
			if(u_rx[i]>640)	u_rx[i] = 640;
			if(u_lx[i]<0)	u_lx[i] = 0;
			if(u_ry[i]>480)	u_ry[i] = 480;
			if(u_ly[i]<0)	u_ly[i] = 0;

			my_str.data = camera_name;
			my_roi.x_offset = u_lx[i];
			my_roi.y_offset = u_ly[i];
			my_roi.width = abs(u_rx[i] - u_lx[i]);
			my_roi.height = abs(u_ry[i] - u_ly[i]);
			

			tf::Vector3 v1,v2;
			m1.setEulerZYX(SN_KINECT1_OFFSET_RPY_Y, SN_KINECT1_OFFSET_RPY_P, SN_KINECT1_OFFSET_RPY_R);
			X[i] = ((float)(320 - (float)(u_lx[i]+abs(u_rx[i] - u_lx[i])/2)) / 525 * Z[i]);
			Y[i] = ((float)(240 - u_ly[i]) / 525 * Z[i]);

			v1.setX(X[i]); v1.setY(Y[i]); v1.setZ(Z[i]);
			v2	=	m1*v1;

			//ROS_ERROR("i:%d, %d, %d, %d, %d", i, u_lx[i], u_ly[i], u_rx[i], u_ry[i]);
			//ROS_ERROR("%.2f, %.2f", X[i], Y[i]);

			CvPoint pt1 = {u_lx[i], u_ly[i]};
			CvPoint pt2 = {u_rx[i], u_ry[i]};
			


			my_point.x = -((float)v2.getX() + SN_KINECT1_OFFSET_X)*0.001;
			my_point.y =  ((float)v2.getY() + SN_KINECT1_OFFSET_Y)*0.001-0.5;
			my_point.z =  ((float)Z[i] +      SN_KINECT1_OFFSET_Z)*0.001;
			printf("%i",u_lx[i]);
			printf("my_point: %.2f, %.2f, %.2f\n", my_point.x, my_point.y, my_point.z);
			printf("my_roi: %.2f, %.2f\n", my_roi.x_offset, my_roi.y_offset);
			
			if(my_point.z!=0 and my_roi.width*my_roi.height>30000){
				cvRectangle(imagePixels, pt1, pt2, CV_RGB(255,255,255), 1, 8, 0);
			}
			int gesture_;
			if(my_point.y>1.45){
				if(User_action[i] == 1 ) {
					gesture_=1; printf("Asking!\n");
				} else{
					gesture_=2; printf("Stand!\n"); 
				}
			}
			else {
				if (User_action[i]==4) {
					gesture_=4; printf("Sleep\n");	// it should add the leaning angle here!
				} else {
					gesture_=3; printf("Sit!\n"); 
				}	
			}			

			my_point.x = (float)X[i]/1000.0;
			my_point.y = (float)Y[i]/1000.0;
			my_point.z = (float)Z[i]/1000.0;
			int userID = (int)i; //users[i];

			//printf("ID : %d // User_action : %d\n",i,User_action[i]);
			CvFont font;
			cvInitFont(&font, CV_FONT_VECTOR0, 0.5, 0.5, 0, 2);
			char pos_text[100], roi_text[100];
			sprintf(pos_text, "x : %.2f, y : %.2f, z : %.2f", my_point.x, my_point.y, my_point.z);
			sprintf(roi_text, "id: %d, roi : %d, %d", userID, my_roi.width, my_roi.height);			
			//cvPutText(imagePixels, pos_text, cvPoint(u_lx[i], u_ly[i]-10), &font, CV_RGB(255,255,255));
			//cvPutText(imagePixels, roi_text, cvPoint(u_lx[i], u_ly[i]-30), &font, CV_RGB(255,255,255));
			//if(p_Torso.X !=0 && p_Torso.Y !=0 && p_Torso.Z!=0)			
				//cvPutText(imagePixels, "skeleton", cvPoint(u_lx[i], u_ly[i]+10), &font, CV_RGB(255,127,0));

			if(my_point.z!=0 and my_roi.width*my_roi.height>30000){
				g_track.cam_id.push_back(my_str);
				g_track.roi.push_back(my_roi);
				//g_track.location.push_back(my_point);	
				g_track.location.push_back(my_point);	// it should be a torso data
				g_track.user_id.push_back(gesture_);
				cnt++;
//draw only publish topics
//				cvRectangle(imagePixels, pt1, pt2, CV_RGB(255,255,255), 1, 8, 0);
				cvPutText(imagePixels, pos_text, cvPoint(u_lx[i], u_ly[i]-10), &font, CV_RGB(255,255,255));
				cvPutText(imagePixels, roi_text, cvPoint(u_lx[i], u_ly[i]-30), &font, CV_RGB(255,255,255));
				if(p_Torso.X !=0 && p_Torso.Y !=0 && p_Torso.Z!=0)			
					cvPutText(imagePixels, "skeleton", cvPoint(u_lx[i], u_ly[i]+10), &font, CV_RGB(255,127,0));
			}
			else{
				if (my_point.z==0 and my_roi.width*my_roi.height>30000){
					g_track.cam_id.push_back(my_str);
					g_track.roi.push_back(my_roi);
					//g_track.location.push_back(my_point);	
					g_track.location.push_back(my_point);	// it should be a torso data
					g_track.user_id.push_back(gesture_);
					cnt++;
				}
			}
		}
		else
		{
			continue;
			quit = cv::waitKey(10);
		}
	}
	
	g_track.total = cnt;
	g_track.header.stamp = ros::Time::now();
	g_track.header.frame_id = frame_id;
	pub.publish(g_track);

	CvFont font;
	cvInitFont(&font, CV_FONT_VECTOR0, 0.6, 0.6, 0, 1);
	cvPutText(imagePixels, MESSAGE, cvPoint(20, 20), &font, CV_RGB(255,255,255));
	
	int locX=0, locY=0;
	if     (strcmp(camera_name,"sn_kinect_1")==0) {locX=50, locY=50; }
	else if(strcmp(camera_name,"sn_kinect_2")==0) {locX=50+320; locY=50;}
	else if(strcmp(camera_name,"sn_kinect_3")==0) {locX=50; locY=50+240;}
	else					      {locX=50+320; locY=50+240;}
	quit = cv::waitKey(10);
	if(nMode==1)
	{
		cvShowImage(TRACKER_ID,imagePixels);
		if(!bFirst){
			cvMoveWindow(TRACKER_ID,locX,locY);
			bFirst=true;
		}

	}else if(nMode==2){
		IplImage* imagePixels2;
		imagePixels2 = cvCreateImage(cvSize(320,240),8,3);
		cvResize(imagePixels, imagePixels2, CV_INTER_LINEAR);
		cvShowImage(TRACKER_ID,imagePixels2);
		if(!bFirst)	{
			cvMoveWindow(TRACKER_ID,locX,locY);
			bFirst=true;
		}
		cvReleaseImage(&imagePixels2);
	}
	cvReleaseImage(&imagePixels);

}




#define CHECK_RC(nRetVal, what)                                     \
		if (nRetVal != XN_STATUS_OK)                                    \
		{                                                               \
			ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
			return nRetVal;                                             \
		}

int main(int argc, char **argv) 
{
	bFirst=false;
	nMode = 2;
	if(argc==1){
		ROS_ERROR("Need to input argument like sn_kinect_1");
		sprintf(camera_name, "sn_kinect_1");
	}
	else
		sprintf(camera_name,"%s",argv[1]);

	ROS_ERROR("==========================================");
	ROS_ERROR("TRACKER1: CAM NAME is %s", camera_name);

	//sprintf(TRACKER_ID, "%s_tracker [press 's','x','q']", camera_name);
	sprintf(TRACKER_ID, "%s_tracker__x_r_s", camera_name);

	ros::init(argc, argv, TRACKER_ID);
	ros::NodeHandle nh;

	string configFilename = ros::package::getPath("sp_hl_hd_op") + "/openni_tracker.xml";
	printf("Path: %s\n", configFilename.c_str());
	
	XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
	CHECK_RC(nRetVal, "InitFromXml");

	nRetVal = XN_STATUS_OK;

	//xnLogInitFromXmlFile(csXmlFile);

	nRetVal = g_Context.Init();
	XN_IS_STATUS_OK(nRetVal);

	// SELECTION OF THE DEVICE
	xn::EnumerationErrors errors;
	xn::Device g_Device;

	// find devices
	xn::NodeInfoList list;
	xn::NodeInfoList list_depth;
	nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_DEVICE, NULL, list, &errors);
	XN_IS_STATUS_OK(nRetVal);

	//ROS_ERROR("The following devices were found:");
	int i = 1;
	for (xn::NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it, ++i)
	{
		xn::NodeInfo deviceNodeInfo = *it;

		xn::Device deviceNode;
		deviceNodeInfo.GetInstance(deviceNode);
		XnBool bExists = deviceNode.IsValid();
		if (!bExists)
		{
			g_Context.CreateProductionTree(deviceNodeInfo, deviceNode);
			// this might fail.
		}

		if (deviceNode.IsValid() && deviceNode.IsCapabilitySupported(XN_CAPABILITY_DEVICE_IDENTIFICATION))
		{
			const XnUInt32 nStringBufferSize = 200;
			XnChar strDeviceName[nStringBufferSize];
			XnChar strSerialNumber[nStringBufferSize];

			XnUInt32 nLength = nStringBufferSize;
			deviceNode.GetIdentificationCap().GetDeviceName(strDeviceName, nLength);
			nLength = nStringBufferSize;
			deviceNode.GetIdentificationCap().GetSerialNumber(strSerialNumber, nLength);
			printf("[%d] %s (%s)\n", i, strDeviceName, strSerialNumber);
		}
		else
		{
			printf("[%d] %s\n", i, deviceNodeInfo.GetCreationInfo());
		}

		// release the device if we created it
		if (!bExists && deviceNode.IsValid())
		{
			deviceNode.Release();
		}
	}
	printf("\n");
	printf("Choose device to open : ");


	int chosen = 1;
	//int nRetval = scanf("%d", &chosen);

	//	printf("############### %d\n",atoi(argv[1]));
	// create it
	xn::NodeInfoList::Iterator it2 = list.Begin();
	for (i = 1; i < chosen; ++i)
	{
		it2++;
	}

	xn::NodeInfo deviceNode = *it2;
	nRetVal = g_Context.CreateProductionTree(deviceNode, g_Device);
	//ROS_ERROR("Production tree of the device created.\n");

	// SELECTION OF THE DEPTH GENERATOR
	nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, list_depth, &errors);
	XN_IS_STATUS_OK(nRetVal);

	//ROS_ERROR("The following devices were found:\n");
	int i_depth = 1;
	for (xn::NodeInfoList::Iterator it_depth = list_depth.Begin(); it_depth != list_depth.End(); ++it_depth, ++i_depth)
	{
		xn::NodeInfo depthNodeInfo = *it_depth;

		xn::Device depthNode;
		depthNodeInfo.GetInstance(depthNode);
		XnBool bExists_depth = depthNode.IsValid();
		if (!bExists_depth)
		{
			g_Context.CreateProductionTree(depthNodeInfo, depthNode);
			// this might fail.
		}

		if (depthNode.IsValid() && depthNode.IsCapabilitySupported(XN_CAPABILITY_DEVICE_IDENTIFICATION))
		{
			const XnUInt32 nStringBufferSize = 200;
			XnChar strDeviceName[nStringBufferSize];
			XnChar strSerialNumber[nStringBufferSize];

			XnUInt32 nLength = nStringBufferSize;
			depthNode.GetIdentificationCap().GetDeviceName(strDeviceName, nLength);
			nLength = nStringBufferSize;
			depthNode.GetIdentificationCap().GetSerialNumber(strSerialNumber, nLength);
			printf("[%d] %s (%s)\n", i, strDeviceName, strSerialNumber);
		}
		else
		{
			printf("[%d] %s\n", i, depthNodeInfo.GetCreationInfo());
		}

		// release the device if we created it
		if (!bExists_depth && depthNode.IsValid())
		{
			depthNode.Release();
		}
	}
	//ROS_ERROR("Choose device to open : ");

	int chosen_depth = 1;

	xn::NodeInfoList::Iterator it_depth = list_depth.Begin();
	xn::NodeInfo depthNode = *it_depth;
	nRetVal = g_Context.CreateProductionTree(depthNode, g_DepthGenerator);
	//ROS_ERROR("Production tree of the DepthGenerator created.\n");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	printf("Production tree of the depth generator created.\n");
	XN_IS_STATUS_OK(nRetVal);
	//ROS_ERROR("XN_IS_STATUS_OK(nRetVal).\n");



	CHECK_RC(nRetVal, "Find depth generator");
	//ROS_ERROR("CHECK_RC(nRetVal, Find depth generator);\n");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	printf("User generator found.\n");
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		printf("User generator created.\n");
		sprintf(MESSAGE, "User generator created");
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		printf("Supplied user generator doesn't support skeleton.\n");
		return 1;
	}

	XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
		g_bNeedPose = TRUE;
		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");


	ros::NodeHandle pnh;
	//ros::Subscriber sub = pnh.subscribe("sn_kinect1/rgb/image_color", 10, imageCb_rgb);

	string frame_id("/sn_kinect1_depth_frame");
	pnh.getParam("camera_frame_id", frame_id);
	pub = pnh.advertise<pub_msgs::where_msgs>("/sn_kinect/detector", 2);
	image_transport::ImageTransport it(pnh);
	
	
	ros::Rate r(30);
	std::vector<double> global_name;
	int cnt=0;
	ROS_ERROR("============= sp_tracker1 is ready =================");

	while (ros::ok()&& quit!='q') 
	{
		if(quit =='x')		nMode=0;
		if(quit =='s')		nMode=1;
		if(quit =='r')		nMode=2;

		if(cnt==0 || cnt>30*600)
		{		
			if     (strcmp(camera_name,"sn_kinect_2")==0) pnh.getParam("/psn_unit2", global_name);
			else if(strcmp(camera_name,"sn_kinect_3")==0) pnh.getParam("/psn_unit3", global_name);
			else if(strcmp(camera_name,"sn_kinect_4")==0) pnh.getParam("/psn_unit4", global_name);
			else					      pnh.getParam("/psn_unit1", global_name);

			ROS_ERROR("TRACKER1 --------------------\n set %s PARAM [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", camera_name, global_name[0],global_name[1],global_name[2],global_name[3],global_name[4],global_name[5]);
			
			SN_KINECT1_OFFSET_X 	= global_name[0]*1000.0;
			SN_KINECT1_OFFSET_Y 	= global_name[1]*1000.0;
			SN_KINECT1_OFFSET_Z 	= global_name[2]*1000.0;
			SN_KINECT1_OFFSET_RPY_R = global_name[3];
			SN_KINECT1_OFFSET_RPY_P = global_name[4];
			SN_KINECT1_OFFSET_RPY_Y = global_name[5];                    
			cnt = 0;
		}
		if(cnt==0)
			ROS_ERROR("read a new frame: g_Context.WaitAndUpdateAll() is calling...");	
		nRetVal = g_Context.WaitAndUpdateAll();
		if( nRetVal != XN_STATUS_OK)
		{
			ROS_ERROR("read failed: %s", xnGetStatusString(nRetVal));
			return 1;	
		}
		publishTransforms(frame_id);
		ros::spinOnce();
	
		r.sleep();
		cnt++;
	}
	ROS_ERROR("--------------------------------------------------------");
	g_Context.Shutdown();
	return 0;
}
