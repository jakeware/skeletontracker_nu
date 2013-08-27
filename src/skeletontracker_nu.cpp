// nu_skeletaltracker.cpp
// Jake Ware and Jarvis Schultz
// Winter 2011


//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <kdl/frames.hpp>

#include "skeletonmsgs_nu/Skeletons.h"

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#include <math.h>

// this is a macro and needs the backslashes at the end of the line
#define CHECK_RC(nRetVal, what)						\
    if (nRetVal != XN_STATUS_OK)					\
    {									\
	printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));	\
	return nRetVal;							\
    }

//---------------------------------------------------------------------------
// Global variables
//---------------------------------------------------------------------------
xn::Context        g_Context;  
xn::DepthGenerator g_DepthGenerator;  
xn::UserGenerator  g_UserGenerator;
    
XnBool g_bNeedPose = false;  
XnChar g_strPose[20]; 

//---------------------------------------------------------------------------
// Classes and Globals
//---------------------------------------------------------------------------
// Callback: New user was detected mit and ros used slightly different
// versions of this function by eliminating different if/else
// components
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator,
				   XnUserID nId, void* pCookie)
{
    printf("New User %d\n", nId); 

    if (g_bNeedPose)
	g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
    else
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}


// Callback: An existing user was lost
// same
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator,
				    XnUserID nId, void* pCookie)
{
    printf("Lost user %d\n", nId);  
}


// Callback: Started calibration
// same
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(
    xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
    printf("Calibration started for user %d\n", nId);
}

// Callback: Finished calibration
// slightly modified
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(
    xn::SkeletonCapability& capability,
    XnUserID nId, XnBool bSuccess, void* pCookie)
{
    if (bSuccess) {
	printf("Calibration complete, start tracking user %d\n", nId);  
	// missing two lines here
	g_UserGenerator.GetSkeletonCap().StartTracking(nId); 
    }
    else {
	printf("Calibration failed for user %d\n", nId); 

	if (g_bNeedPose)
	    g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	else
	    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    }
}


// Callback: Detected a pose
// missing if/else statements and a few lines
void XN_CALLBACK_TYPE UserPose_PoseDetected(
    xn::PoseDetectionCapability& capability,
    XnChar const* strPose, XnUserID nId, void* pCookie)
{
    printf("Pose %s detected for user %d\n", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE); 
}
 
//---------------------------------------------------------------------------
// MIT Functions
//---------------------------------------------------------------------------

// Point is a triplet of 64 bit floats
geometry_msgs::Point vecToPt(XnVector3D pt)
{
    geometry_msgs::Point ret;
    ret.x=pt.X/1000.0;
    ret.y=-pt.Y/1000.0;
    ret.z=pt.Z/1000.0;
    return ret;
}


// Point32 is a triplet of 32 bit floats
geometry_msgs::Point32 vecToPt3(XnVector3D pt)
{
    geometry_msgs::Point32 ret;
    ret.x=pt.X/1000.0;
    ret.y=-pt.Y/1000.0;
    ret.z=pt.Z/1000.0;
    return ret;
}


class TrackerClass{


private:
    ros::NodeHandle nh_;
    ros::Publisher pmap_pub;
    ros::Publisher skel_pub;
    ros::Time tstamp, tstamp_last;
    tf::TransformBroadcaster br;
    ros::Timer timer;

public:
    TrackerClass(){
	
	// define ros publishers for skels
	skel_pub = nh_.advertise<skeletonmsgs_nu::Skeletons> ("skeletons", 100);
	timer = nh_.createTimer(ros::Duration(0.01), &TrackerClass::timerCallback, this);

	ROS_INFO("Starting Tracker...\n");
    }

//---------------------------------------------------------------------------
// NU Functions
//---------------------------------------------------------------------------

    void getTransform(XnUserID user, XnSkeletonJoint name,
		      std::string const& child_frame_id,
		      skeletonmsgs_nu::SkeletonJoint &j)
	{
	    geometry_msgs::Point position;

	    XnSkeletonJointPosition joint;
	    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user,
								      name,
								      joint);
	    position = vecToPt(joint.position);
	    j.confidence = joint.fConfidence;

	    XnSkeletonJointOrientation joint_orientation;
	    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(
		user, name, joint_orientation);

	    XnFloat* m = joint_orientation.orientation.elements;
	    m[0] = -m[0];
	    m[1] = -m[1];
	    m[2] = -m[2];
	    m[6] = -m[6];
	    m[7] = -m[7];
	    m[8] = -m[8];
    
	    KDL::Rotation rotation(m[0], m[1], m[2],
				   m[3], m[4], m[5],
				   m[6], m[7], m[8]);

	    double qx, qy, qz, qw;
	    rotation.GetQuaternion(qx, qy, qz, qw);

	    // publish the tf data
	    tf::Transform transform;
	    tf::StampedTransform t_wj;
	    ros::Time t;
	    t = ros::Time::now();
	    std::stringstream frame;
	    frame << child_frame_id << "_" << user;
	    transform.setOrigin(tf::Vector3(position.x, position.y, position.z));
	    transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
	    br.sendTransform(tf::StampedTransform(
				 transform,
				 t,
				 "camera_depth_optical_frame",
				 frame.str()));
	    
	    j.transform.translation.x = transform.getOrigin().x();
	    j.transform.translation.y = transform.getOrigin().y();
	    j.transform.translation.z = transform.getOrigin().z();
  
	    j.transform.rotation.x = transform.getRotation().x();
	    j.transform.rotation.y = transform.getRotation().y();
	    j.transform.rotation.z = transform.getRotation().z();
	    j.transform.rotation.w = transform.getRotation().w();

	    return;
	}

 
    void publishData(void)
	{
	    tstamp_last = tstamp;
	    tstamp=ros::Time::now();

	    int users_count = 0;
	    skeletonmsgs_nu::Skeletons skels;
  
	    XnUserID users[15];
	    XnUInt16 users_max = 15;
	    g_UserGenerator.GetUsers(users, users_max);
  
	    for (int i = 0; i < users_max; ++i) {
		XnUserID user = users[i];

		if (g_UserGenerator.GetSkeletonCap().IsTracking(user)) {
		    users_count++;

		    skeletonmsgs_nu::Skeleton skel;
		    skel.userid=user;
		    getTransform(user, XN_SKEL_HEAD, "head", skel.head);
		    getTransform(user, XN_SKEL_NECK, "neck", skel.neck);
		    getTransform(user, XN_SKEL_TORSO, "torso", skel.torso);
		    getTransform(user, XN_SKEL_LEFT_SHOULDER, "left_shoulder",
				 skel.left_shoulder);
		    getTransform(user, XN_SKEL_LEFT_ELBOW, "left_elbow",
				 skel.left_elbow);
		    getTransform(user, XN_SKEL_LEFT_HAND, "left_hand",
				 skel.left_hand);
		    getTransform(user, XN_SKEL_RIGHT_SHOULDER, "right_shoulder",
				 skel.right_shoulder);
		    getTransform(user, XN_SKEL_RIGHT_ELBOW, "right_elbow",
				 skel.right_elbow);
		    getTransform(user, XN_SKEL_RIGHT_HAND, "right_hand",
				 skel.right_hand);
		    getTransform(user, XN_SKEL_LEFT_HIP, "left_hip",
				 skel.left_hip);
		    getTransform(user, XN_SKEL_LEFT_KNEE, "left_knee",
				 skel.left_knee);
		    getTransform(user, XN_SKEL_LEFT_FOOT,  "left_foot",
				 skel.left_foot);
		    getTransform(user, XN_SKEL_RIGHT_HIP, "right_hip",
				 skel.right_hip);
		    getTransform(user, XN_SKEL_RIGHT_KNEE, "right_knee",
				 skel.right_knee);
		    getTransform(user, XN_SKEL_RIGHT_FOOT, "right_foot",
				 skel.right_foot);
      
		    skels.skeletons.push_back(skel); 
		}
	    }
  
	    ROS_DEBUG("users_count: %i", users_count);
	    if(users_count > 0)
	    {
		skels.header.stamp=tstamp;
		skels.header.frame_id="camera_depth_frame";
		skel_pub.publish(skels);
		// ros::spinOnce();
	    }
	}


    void timerCallback(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("timerCallback triggered");
	    g_Context.WaitAndUpdateAll();  // sits and waits for new set of user
	    // data from kinect (30 Hz)
	    publishData();  // everything important happens in this function
	}

}; // end of class TrackerClass
    

//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char **argv) 
{
    sleep(5);  // delay for openni_camera to start
    
    // startup node
    ros::init(argc, argv, "skeletontracker_nu");
    ros::NodeHandle node;

    // init from XML
    std::string configFilename = ros::package::getPath("skeletontracker_nu") +
	"/kinectconfig.xml";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml"); 

    // find depth generator
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator); 
    CHECK_RC(nRetVal, "Find depth generator");

    // find user generator
    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
    if (nRetVal != XN_STATUS_OK) {
        nRetVal = g_UserGenerator.Create(g_Context);
        CHECK_RC(nRetVal, "Find user generator");
    }

    // check user generator support
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
        printf("Supplied user generator doesn't support skeleton\n");
        return 1;
    }

    // needs comment
    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks(User_NewUser,
					  User_LostUser,
					  NULL,
					  hUserCallbacks);
    XnCallbackHandle hCalibrationCallbacks;
    g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(
	UserCalibration_CalibrationStart,
	UserCalibration_CalibrationEnd,
	NULL,
	hCalibrationCallbacks);

    // check if pose if supported
    if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
	g_bNeedPose = TRUE;
	if (!g_UserGenerator.IsCapabilitySupported(
		XN_CAPABILITY_POSE_DETECTION))
	{
	    printf("Pose required, but not supported\n");
	    return 1;
	}

	// needs comment
	XnCallbackHandle hPoseCallbacks;
	g_UserGenerator.GetPoseDetectionCap().
	    RegisterToPoseCallbacks(UserPose_PoseDetected,
				    NULL, NULL, hPoseCallbacks);
	g_UserGenerator.GetSkeletonCap().
	    GetCalibrationPose(g_strPose);
    }

    g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(
	XN_SKEL_PROFILE_ALL);  // needs comment

    nRetVal = g_Context.StartGeneratingAll();
    CHECK_RC(nRetVal, "StartGenerating");

    TrackerClass tracker;

    ros::spin();

    g_Context.Shutdown();
    return 0;
}
