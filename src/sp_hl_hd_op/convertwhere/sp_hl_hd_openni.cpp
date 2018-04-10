/*
 * sp_hl_hd_op.cpp
 * It generates topic(/sn_kinect/detector) by using sn_kinect{1:4}/rgb/image_color/compressed
 * and /sn_kinect{1:4}/depth/image_raw/compressedDepth
 *
 * Designed by Kijin An and Sang-Seok Yun
 *
 * kijin.an@gmail.com, yssmecha@gmail.com
 *
 * Center for Robotics Research, KIST, 2015
 *
 * Interface template for SimonPic project
 *
 * Example code for reading a compressed binary depth image file.
 *
 * Please make sure that you need to modity comment on this template without erasing any code.
 *
 *
 * THE SOFTWARE IS PROVIDED �~@~\AS IS�~@~] AND THE PROVIDER GIVES NO EXPRESS OR IMPLIED WARRANTIES OF ANY KIND,
 * INCLUDING WITHOUT LIMITATION THE WARRANTIES OF FITNESS FOR ANY PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL THE PROVIDER BE HELD RESPONSIBLE FOR LOSS OR DAMAGE CAUSED BY THE USE OF THE SOFTWARE.
 *
 *
 */
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

float SN_KINECT1_OFFSET_X = 0*1000;
//float SN_KINECT1_OFFSET_Y = (1.855-0.3)*1000;
float SN_KINECT1_OFFSET_Y = 0;
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
/////////////////////??/////////////////////??/////////////////////??/////////////////////??/////////////////////??
	int User_action[10];
/////////////////////??/////////////////////??/////////////////////??/////////////////////??/////////////////////??
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



void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
	ROS_INFO("Lost user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
	ROS_INFO("Calibration started for user %d", nId);
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

	//if(child_frame_id=="head")
	 //ROS_ERROR("HEAD %f %f %f",x,y,z);

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

	// #4994
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
        
		

}
/////////////////////??/////////////////////??/////////////////////??/////////////////////??/////////////////////??
//2: sit or stand
//1: ask;
int action_judge(XnPoint3D p_Head, XnPoint3D p_Sh_Left, XnPoint3D p_El_Left, XnPoint3D p_Ha_Left, XnPoint3D p_Sh_Right, XnPoint3D p_El_Right,XnPoint3D p_Ha_Right)
{
	if((p_Head.Y < p_El_Left.Y) || (p_Head.Y < p_Ha_Left.Y)){return 1;}
	else if((p_Head.Y < p_El_Right.Y) || (p_Head.Y < p_Ha_Right.Y)) {return 1;}
	//else if((p_Sh_Left
	else { return 2; }
}

/////////////////////??/////////////////////??/////////////////////??/////////////////////??/////////////////////??
void publishTransforms(const std::string& frame_id) {
	XnUserID users[10];
	XnUInt16 users_count = 10;
	g_UserGenerator.GetUsers(users, users_count);

	pub_msgs::where_msgs g_track;
	sensor_msgs::RegionOfInterest my_roi;
	geometry_msgs::Point32 my_point;
	std_msgs::String my_str;

	float X[10], Y[10], Z[10];
	int User_action[10];
	for(int i=0; i<10; i++)
	{
		User_action[i] = 0;
		X[i] = 0;
		Y[i] = 0;
		Z[i] = 0;
	}
	

	for (int i = 0; i < users_count; ++i)
	{

		XnPoint3D p_Head;
		XnPoint3D p_Sh_Left, p_El_Left, p_Ha_Left;
		XnPoint3D p_Sh_Right, p_El_Right, p_Ha_Right;

		XnUserID user = users[i];
		XnPoint3D p;
		if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
			continue;
		g_UserGenerator.GetCoM(users[i],p);
		
		publishTransform(user, XN_SKEL_HEAD,           frame_id, "head", p_Head);
//		publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
//		publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

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


		//1: ask
		//2: sit or stand
		User_action[users[i]] = action_judge(p_Head, p_Sh_Left, p_El_Left, p_Ha_Left, p_Sh_Right, p_El_Right, p_Ha_Right);
// %d  User Action: %d\n",i,User_action[i]);}
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


		//X[users[i]] = (float)-p.X + SN_KINECT2_OFFSET_X;
	//	Y[users[i]] = (float)p.Y + SN_KINECT2_OFFSET_Y;
		//Z[users[i]] = (float)p.Z + SN_KINECT2_OFFSET_Z;
		//		printf("SN_KINECT2  %f %f %f\n",X[users[i]],Y[users[i]],Z[users[i]]);
	}

	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;

	g_UserGenerator.GetUserPixels(0,sceneMD);

	unsigned short* depthPixels = (unsigned short*)sceneMD.Data();

	cv::Mat mat(480,640,CV_8UC1,cv::Scalar::all(0));
	int u_rx[10], u_lx[10], u_ry[10], u_ly[10];
	int tmp_u_rx=0, tmp_u_lx=0, tmp_u_ry=0, tmp_u_ly=0;
	int ii =0;
	IplImage* imagePixels;
	imagePixels = cvCreateImage(cvSize(640,480),8,3);

	for(int j=0; j<10; j++)
	{
		u_rx[j] = 0;
		u_lx[j] = 640;
		u_ry[j] = 0;
		u_ly[j] = 480;
	}

	for(int y=0;y<480;y++){
		for(int x=0;x<640;x++,ii++){
			imagePixels->imageData[(y*imagePixels->widthStep) + x*3+0] = 0;
			imagePixels->imageData[(y*imagePixels->widthStep) + x*3+1] = 0;
			imagePixels->imageData[(y*imagePixels->widthStep) + x*3+2] = 0;

			if (depthPixels[ii] > 0)
			{
				imagePixels->imageData[(y*imagePixels->widthStep) + x*3+0] = Colors[depthPixels[ii]%10][0];
				imagePixels->imageData[(y*imagePixels->widthStep) + x*3+1] = Colors[depthPixels[ii]%10][1];
				imagePixels->imageData[(y*imagePixels->widthStep) + x*3+2] = Colors[depthPixels[ii]%10][2];

				tmp_u_rx = u_rx[depthPixels[ii]%10];
				tmp_u_lx = u_lx[depthPixels[ii]%10];
				tmp_u_ry = u_ry[depthPixels[ii]%10];
				tmp_u_ly = u_ly[depthPixels[ii]%10];

				tmp_u_rx = std::max(tmp_u_rx,x);
				tmp_u_lx = std::min(tmp_u_lx,x);
				tmp_u_ry = std::max(tmp_u_ry,y);
				tmp_u_ly = std::min(tmp_u_ly,y);

				u_rx[depthPixels[ii]%10] = tmp_u_rx;
				u_lx[depthPixels[ii]%10] = tmp_u_lx;
				u_ry[depthPixels[ii]%10] = tmp_u_ry;
				u_ly[depthPixels[ii]%10] = tmp_u_ly;
			}
		}
	}
	int cnt=0;
	float tmp_x=0, tmp_y=0, tmp_z=0;
	for(int i=0; i<10; i++)
	{
		char tmp[100];
		if((u_rx[i] - u_lx[i]) > 0 && (u_rx[i] - u_lx[i]) <= 640 && (u_ry[i] - u_ly[i]) > 0 && (u_ry[i] - u_ly[i]) <= 480)
		{
			if(u_rx[i]>640)
				u_rx[i] = 640;
			if(u_lx[i]<0)
				u_lx[i] = 0;
			if(u_ry[i]>480)
				u_ry[i] = 480;
			if(u_ly[i]<0)
				u_ly[i] = 0;

			sprintf(tmp,"sn_kinect_1");
			//my_str.data = tmp;
			my_str.data = camera_name;
			my_roi.x_offset = u_lx[i];
			my_roi.y_offset = u_ly[i];
			my_roi.width = abs(u_rx[i] - u_lx[i]);
			my_roi.height = abs(u_ry[i] - u_ly[i]);

			tf::Vector3 v1,v2;
			m1.setEulerZYX(SN_KINECT1_OFFSET_RPY_Y,SN_KINECT1_OFFSET_RPY_P,SN_KINECT1_OFFSET_RPY_R);
			X[i] = ((float)(320 - (float)(u_lx[i]+abs(u_rx[i] - u_lx[i])/2)) / 525 * Z[i]);
			Y[i] = ((float)(240 - u_ly[i]) / 525 * Z[i]);

			v1.setX(X[i]);v1.setY(Y[i]);v1.setZ(Z[i]);
			v2	=	m1*v1;

//			ROS_ERROR("%.2f, %.2f", X[i], Y[i]);

			CvPoint pt1 = {u_lx[i], u_ly[i]};
			CvPoint pt2 = {u_rx[i], u_ry[i]};
			cvRectangle(imagePixels, pt1, pt2, CV_RGB(255,255,255), 1, 8, 0);
//User_action[i]
			my_point.x = -((float)v2.getX() + SN_KINECT1_OFFSET_X)*0.001;
			my_point.y = ((float)v2.getY() + SN_KINECT1_OFFSET_Y)*0.001;
			my_point.z = ((float)Z[i] + SN_KINECT1_OFFSET_Z)*0.001;

			
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			printf("Sit;stand User ID :%d  User Action: %d\n",i-1,User_action[i]);
			
			int gesture_;
			if((User_action[i] == 1) && (my_point.y>=1)) {printf("Asking!\n");
			gesture_=1;}			
			else if(User_action[i] == 2) //sit or stand or sleep
			{
				if(my_point.y >=1.5){printf("Stand!\n"); 
				gesture_=2;}
				else if(my_point.y >=1.1){printf("Sit!\n"); 
				User_action[i-1]=3;gesture_=3;}
				else{printf("Sleep\n");
				User_action[i-1] = 4;gesture_=4;}
			}

			my_point.x = (float)X[i]/1000;
			my_point.y = (float)Y[i]/1000;
			my_point.z = (float)Z[i]/1000;

			//printf("ID : %d // User_action : %d\n",i,User_action[i]);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			CvFont font;
			cvInitFont(&font, CV_FONT_VECTOR0, 0.5, 0.5, 0, 2);
			char pos_text[100], roi_text[100];
			sprintf(pos_text, "x : %.2f, y : %.2f, z : %.2f", my_point.x, my_point.y, my_point.z);
			sprintf(roi_text, "roi : %d", my_roi.width * my_roi.height);
			cvPutText(imagePixels, pos_text, cvPoint(u_lx[i], u_ly[i]-10), &font, CV_RGB(255,255,255));
			cvPutText(imagePixels, roi_text, cvPoint(u_lx[i], u_ly[i]-30), &font, CV_RGB(255,255,255));

			if(my_point.z!=0){
				g_track.cam_id.push_back(my_str);
				g_track.roi.push_back(my_roi);
				g_track.location.push_back(my_point);
				g_track.user_id.push_back(gesture_);
				cnt++;
			}

			//			g_track.id.push_back(my_str);
			//			g_track.roi.push_back(my_roi);
			//			g_track.pos.push_back(my_point);

			//			cnt++;
		}
		else
		{
			continue;
			quit = cv::waitKey(10);
		}
	}
	//cvShowImage("tracker2",imagePixels);
	//quit = cv::waitKey(10);
	
	cvReleaseImage(&imagePixels);

	g_track.total = cnt;
	g_track.header.stamp = ros::Time::now();
	g_track.header.frame_id = frame_id;
	pub.publish(g_track);
}




#define CHECK_RC(nRetVal, what)                                     \
		if (nRetVal != XN_STATUS_OK)                                    \
		{                                                               \
			ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
			return nRetVal;                                             \
		}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tracker_1");
	ros::NodeHandle nh;

	//cv::namedWindow("tracker1");
	//cv::resizeWindow("tracker1",640,480);
	//cv::moveWindow("tracker1",0,0);

	string configFilename = ros::package::getPath("sp_hl_hd_op") + "/openni_tracker.xml";
	printf("path: %s\n", configFilename.c_str());
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

	printf("The following devices were found:\n");

	int i = 1;
	for (xn::NodeInfoList::Iterator it = list.Begin(); it != list.End(); ++it, ++i)
	{
		xn::NodeInfo deviceNodeInfo = *it;
		xn::Device deviceNode;
		deviceNodeInfo.GetInstance(deviceNode);
		XnBool bExists = deviceNode.IsValid();
		if (!bExists){
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

		}else{
			printf("[%d] %s\n", i, deviceNodeInfo.GetCreationInfo());
		}

		// release the device if we created it
		if (!bExists && deviceNode.IsValid()){
			deviceNode.Release();
		}
	}
	printf("\nChoose device to open (1): ");

	//int chosen = atoi(argv[1]);
	if(argc==1)
		sprintf(camera_name, "sn_kinect_1");
	else
		sprintf(camera_name,"%s",argv[1]);

	printf("cam name: %s\n", camera_name);
	int chosen = 1;

	// create it
	xn::NodeInfoList::Iterator it2 = list.Begin();
	for (i = 1; i < chosen; ++i)	it2++;

	xn::NodeInfo deviceNode = *it2;
	nRetVal = g_Context.CreateProductionTree(deviceNode, g_Device);
	printf("Production tree of the device created.\n");

	// SELECTION OF THE DEPTH GENERATOR
	nRetVal = g_Context.EnumerateProductionTrees(XN_NODE_TYPE_DEPTH, NULL, list_depth, &errors);
	XN_IS_STATUS_OK(nRetVal);

	printf("The following devices were found:\n");
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
	printf("\nChoose device to open (1): ");

	int chosen_depth = 1;
//	int nRetval_depth = scanf("%d", &chosen);

	// create it
	xn::NodeInfoList::Iterator it_depth = list_depth.Begin();
	xn::NodeInfo depthNode = *it_depth;
	nRetVal = g_Context.CreateProductionTree(depthNode, g_DepthGenerator);
	printf("Production tree of the DepthGenerator created.\n");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	printf("Production tree of the depth generator created.\n");
	XN_IS_STATUS_OK(nRetVal);
	printf("XN_IS_STATUS_OK(nRetVal).\n");


	CHECK_RC(nRetVal, "Find depth generator");
	printf("CHECK_RC(nRetVal, Find depth generator);\n");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	printf("User generator found.\n");
	if (nRetVal != XN_STATUS_OK) {
		nRetVal = g_UserGenerator.Create(g_Context);
		printf("User generator created.\n");
		CHECK_RC(nRetVal, "Find user generator");
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
		printf("Supplied user generator doesn't support skeleton.\n");
		ROS_INFO("Supplied user generator doesn't support skeleton");
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

	ros::Rate r(15);


	ros::NodeHandle pnh;
//	ros::Subscriber sub = pnh.subscribe("sn_kinect1/rgb/image_color", 10, imageCb_rgb);

	string frame_id("/sn_kinect1_depth_frame");
	pnh.getParam("camera_frame_id", frame_id);
	pub = pnh.advertise<pub_msgs::where_msgs>("/sn_kinect/detector", 2);
	image_transport::ImageTransport it(pnh);
	
	while (ros::ok()&& quit!='q')
	{
		g_Context.WaitAndUpdateAll();
		publishTransforms(frame_id);
		ros::spinOnce();
		
		std::vector<double> global_name;

       	if (pnh.getParam("/psn_unit1", global_name))
       	{
			for(int i=0; i < global_name.size(); i++){
			//	std::cout << " "<<global_name[i];
			}
	
			//pnhgetParam("/psn_unit1", global_name);
			//ROS_INFO("PARAM %f %f %f %f %f %f",global_name[0],global_name[1],global_name[2],global_name[3],global_name[4],global_name[5]);
			SN_KINECT1_OFFSET_X = global_name[0]*1000;
			SN_KINECT1_OFFSET_Y = global_name[1]*1000;
			SN_KINECT1_OFFSET_Z = global_name[2]*1000;
			SN_KINECT1_OFFSET_RPY_R 	= global_name[3];
			SN_KINECT1_OFFSET_RPY_P 	= global_name[4];
			SN_KINECT1_OFFSET_RPY_Y 	= global_name[5];
		}
    	r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}
