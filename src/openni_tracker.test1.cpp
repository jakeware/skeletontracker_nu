// openni_tracker.cpp

#include <ros/ros.h>  // same
#include <ros/package.h>  // same

#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>  // same

using std::string;

xn::Context        g_Context;  // same
xn::DepthGenerator g_DepthGenerator;  // same
xn::UserGenerator  g_UserGenerator;  // same

XnBool g_bNeedPose   = FALSE;  // same
XnChar g_strPose[20] = "";  // same


//---------------------------------------------------------------------------
// PRIME SENSE FUNCTIONS
//---------------------------------------------------------------------------

// Callback: New user was detected
// mit and ros used slightly different versions of this function by eliminating different if/else components
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("New User %d\n", nId);  // same

  if (g_bNeedPose)
    g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
  else
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}


// Callback: An existing user was lost
// same
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) 
{
  printf("Lost user %d\n", nId);  // same
}


// Callback: Started calibration
// same
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) 
{
  printf("Calibration started for user %d\n", nId);
}


// Callback: Finished calibration
// slightly modified
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) 
{
  if (bSuccess) {
    printf("Calibration complete, start tracking user %d\n", nId);  // same
    // missing two lines here
    g_UserGenerator.GetSkeletonCap().StartTracking(nId);  // same
  }
  else {
    printf("Calibration failed for user %d\n", nId);  // same

    if (g_bNeedPose)
      g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);  // same
    else
      g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);  // same
  }
}


// Callback: Detected a pose
// missing if/else statements and a few lines
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie) 
{
  printf("Pose %s detected for user %d\n", strPose, nId);  // same
  g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);  // same
  g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);  // same
}


// NEEDS COMMENT
void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, string const& frame_id, string const& child_frame_id) 
{
  static tf::TransformBroadcaster br;

  XnSkeletonJointPosition joint_position;
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
  double x = joint_position.position.X / 1000.0;
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

  tf::Transform transform;
  transform.setOrigin(tf::Vector3(x, y, z));
  transform.setRotation(tf::Quaternion(qx, qy, qz, qw));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}


// NEEDS COMMENT
void publishTransforms() 
{
  XnUserID users[15];
  XnUInt16 users_count = 15;
  g_UserGenerator.GetUsers(users, users_count);

  for (int i = 0; i < users_count; ++i) {
    XnUserID user = users[i];
    if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
      continue;

    string frame_id("openni_depth");

    publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
    publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
    publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");

    publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
    publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
    publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");

    publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
    publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
    publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");

    publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
    publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
    publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");

    publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
    publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
    publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
  }
}


// NEEDS COMMENT
// this is a macro and needs the backslashes at the end of the line
#define CHECK_RC(nRetVal, what)						\
  if (nRetVal != XN_STATUS_OK)						\
    {									\
      printf("%s failed: %s\n", what, xnGetStatusString(nRetVal));	\
      return nRetVal;							\
    }


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "openni_tracker");  // same
  ros::NodeHandle nh;  // same

  // missing if/else statement around these three lines
  string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
  XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());  // same
  CHECK_RC(nRetVal, "InitFromXml");  // same

  // all of this is the same
  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator); 
  CHECK_RC(nRetVal, "Find depth generator");

  // all of this is the same
  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
  if (nRetVal != XN_STATUS_OK) {
    nRetVal = g_UserGenerator.Create(g_Context);
    CHECK_RC(nRetVal, "Find user generator");
  }

  // all of this is the same
  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON)) {
    printf("Supplied user generator doesn't support skeleton\n");
    return 1;
  }

  // all of this is the same
  XnCallbackHandle hUserCallbacks;
  g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  XnCallbackHandle hCalibrationCallbacks;
  g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

  // all of this is the same
  if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration()) {
    g_bNeedPose = TRUE;
    if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
      printf("Pose required, but not supported\n");
      return 1;
    }

    // all of this is the same
    XnCallbackHandle hPoseCallbacks;
    g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
    g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
  }

  // all of this is the same
  g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

  // all of this is the same
  nRetVal = g_Context.StartGeneratingAll();
  CHECK_RC(nRetVal, "StartGenerating");

  ros::Rate r(30);
  while (ros::ok()) {
    g_Context.WaitAndUpdateAll();
    publishTransforms();
    r.sleep();
  }

  g_Context.Shutdown();
  return 0;
}
