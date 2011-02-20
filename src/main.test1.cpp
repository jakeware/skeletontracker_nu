/*****************************************************************************
*                                                                            *
*  OpenNI 1.0 Alpha                                                          *
*  Copyright (C) 2010 PrimeSense Ltd.                                        *
*                                                                            *
*  This file is part of OpenNI.                                              *
*                                                                            *
*  OpenNI is free software: you can redistribute it and/or modify            *
*  it under the terms of the GNU Lesser General Public License as published  *
*  by the Free Software Foundation, either version 3 of the License, or      *
*  (at your option) any later version.                                       *
*                                                                            *
*  OpenNI is distributed in the hope that it will be useful,                 *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the              *
*  GNU Lesser General Public License for more details.                       *
*                                                                            *
*  You should have received a copy of the GNU Lesser General Public License  *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.            *
*                                                                            *
*****************************************************************************/



//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <ros/ros.h>
#include <ros/package.h>

#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>
#include <body_msgs/Skeletons.h>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include "SceneDrawer.test1.h"

#include <GL/glut.h>


//---------------------------------------------------------------------------
// Globals
//---------------------------------------------------------------------------
#define GL_WIN_SIZE_X 720  // opengl window width
#define GL_WIN_SIZE_Y 480  // opengl window height

xn::Context g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator g_UserGenerator;

XnBool g_bhasCal = FALSE;
XnBool g_bNeedPose = FALSE;
XnChar g_strPose[20] = "";
XnBool g_bDrawBackground = TRUE;
XnBool g_bDrawPixels = TRUE;
XnBool g_bDrawSkeleton = TRUE;
XnBool g_bPrintID = TRUE;
XnBool g_bPrintState = TRUE;

XnBool g_bPause = false;  // pause flag
XnBool g_bRecord = false;  // record flag

XnBool g_bQuit = false;  // quit flag

ros::Publisher pmap_pub,skel_pub;  // ros publishers for polygonal maps and skeletons



//---------------------------------------------------------------------------
// MIT FUNCTIONS
//---------------------------------------------------------------------------

// Shutdown prime sense software
void CleanupExit()
{
  g_Context.Shutdown();
  exit (1);
}


// Convert a XnVector3D or XnSkeletonJointPosition to xyz coordinates using type Point in package geometry_msgs
geometry_msgs::Point vecToPt(XnVector3D pt)
{
  geometry_msgs::Point ret;  // create a Point variable called ret from the package geometry_msgs

  ret.x=pt.X/1000.0;  // set x in ret of type Point equal to X in pt of type XnVector3D
  ret.y=-pt.Y/1000.0;
  ret.z=pt.Z/1000.0;
 
  return ret;  // return xyz data in ret of type geometry_msgs
}


// Same as above but with Point32 for higher precision
geometry_msgs::Point32 vecToPt3(XnVector3D pt)
{
  geometry_msgs::Point32 ret;

  ret.x=pt.X/1000.0;
  ret.y=-pt.Y/1000.0;
  ret.z=pt.Z/1000.0;
   
  return ret;
}


// Get a specified joint position (XnSkeletonJoint name) and its confidence from a specified user (XnUserID player) in xyz coordinates using vecToPt and store the position and confidence in a specified body_msgs variable (SkeletonJoint j)
void getSkeletonJoint(XnUserID player, body_msgs::SkeletonJoint &j, XnSkeletonJoint name)
{
  XnSkeletonJointPosition joint1;  // create a XnSkeletonJointPosition variable called joint 1 that will be used to get hold joint data before converting to body_msgs format

  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, name, joint1);  // get joint position of type XnSkeletonJointPosition by passing a specified joint (XnSkeletonJoint name) and a specified user (XnUserID player)
  j.position= vecToPt(joint1.position);  // use vecToPt to convert joint1 of type XnSkeletonJointPosition to j of type body_sgs::SkeletonJoint
  j.confidence = joint1.fConfidence;  // take confidence values from joint1 and put them in j
}


// use previously defined getSkeletonJoint to convert all Prime Sense skeleton joint positions (XN_SKEL...) and store them in a Skeleton variable from package body_msgs
void getSkeleton(XnUserID player, body_msgs::Skeleton &skel)
{
  skel.playerid=player;  // set specified skeleton playerid as specified player (XnUserID player)

  // convert joint positions to body_msgs format
  getSkeletonJoint(player,skel.head,XN_SKEL_HEAD);
  getSkeletonJoint(player,skel.neck,XN_SKEL_NECK);
  getSkeletonJoint(player,skel.left_shoulder,XN_SKEL_LEFT_SHOULDER);
  getSkeletonJoint(player,skel.left_elbow,XN_SKEL_LEFT_ELBOW);
  getSkeletonJoint(player,skel.left_hand,XN_SKEL_LEFT_HAND);
  getSkeletonJoint(player,skel.right_shoulder,XN_SKEL_RIGHT_SHOULDER);
  getSkeletonJoint(player,skel.right_elbow,XN_SKEL_RIGHT_ELBOW);
  getSkeletonJoint(player,skel.right_hand,XN_SKEL_RIGHT_HAND);
  getSkeletonJoint(player,skel.torso,XN_SKEL_TORSO);
  getSkeletonJoint(player,skel.left_hip,XN_SKEL_LEFT_HIP);
  getSkeletonJoint(player,skel.left_knee,XN_SKEL_LEFT_KNEE);
  getSkeletonJoint(player,skel.left_foot,XN_SKEL_LEFT_FOOT);
  getSkeletonJoint(player,skel.right_hip,XN_SKEL_RIGHT_HIP);
  getSkeletonJoint(player,skel.right_knee,XN_SKEL_RIGHT_KNEE);
  getSkeletonJoint(player,skel.right_foot,XN_SKEL_RIGHT_FOOT);
}


// get specified player (XnUserID player) skeleton joint positions (XnSkeletonJointPosition joint1, joint2) corresponding to specified joints (XnSkeletonJoint ejoint1, ejoint2), check their confidence, put them in p of type Polygon from package geometry_msgs, and store them in pmap of type PolygonalMap from package mapping_msgs
void getPolygon(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2, mapping_msgs::PolygonalMap &pmap)
{   
  XnSkeletonJointPosition joint1, joint2;  // create XnSkeletonJointPosition variable that will be used to hold joint positions

  // get joint positions joint1, joint2
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
  g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

  // check joint position confidence 
  if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
    {
      return;
    }

  geometry_msgs::Polygon p;  // create a Polygon variale called p that will hold the two joint positions

  // convert the two joint positions of type XnSkeletonJointPosition to type Point from package geometry_msgs using vecToPt3, and push them into p of type Polygon from package geometry_msgs
  p.points.push_back(vecToPt3(joint1.position));
  p.points.push_back(vecToPt3(joint2.position));

  // push p into pmap of type PolygonalMap from package mapping_msgs
  pmap.polygons.push_back(p);
}


// Display distance between two points given a specified Polygon p from package geometry_msgs
void ptdist(geometry_msgs::Polygon p)
{  
  geometry_msgs::Point32 p1=p.points.back(),p2=p.points.front();  // get first and last points from Polygon p

  // print distance between two points
  printf(" Shoulder dist %.02f \n", sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z)));
}


// create polygonal maps (PolygonalMap pmaps) of each skeleton being tracked
void getSkels(std::vector<mapping_msgs::PolygonalMap> &pmaps, body_msgs::Skeletons &skels)
{
  XnUserID aUsers[15];  // create an array of type XnUserID
  XnUInt16 nUsers = 15;  // set max number of users

  g_UserGenerator.GetUsers(aUsers, nUsers);  // get user data

  // get skeleton pmap for each user being tracked
  for (int i = 0; i < nUsers; ++i)
    {
      // check to see if we want to draw skeletons and if we are tracking users
      if (g_bDrawSkeleton && g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
	{
	  body_msgs::Skeleton skel;
	  getSkeleton(aUsers[i],skel);  // convert skeleton joint positions into body_msgs format using vecToPt for each joint
	  skels.skeletons.push_back(skel);  // store Skeleton skel in Skeletons skels

	  // create polygon for each of the following two Prime Sense joint positions and store them in a polygonal map
	  mapping_msgs::PolygonalMap pmap;
	  getPolygon(aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK, pmap);
	  //               printPt(aUsers[i], XN_SKEL_RIGHT_HAND);
	  getPolygon(aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER, pmap);
	  getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW, pmap);
	  getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_RIGHT_SHOULDER, pmap);
	  //               ptdist(pmap.polygons.back());
	  getPolygon(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND, pmap);
	  getPolygon(aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER, pmap);
	  getPolygon(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW, pmap);
	  getPolygon(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND, pmap);
	  getPolygon(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO, pmap);
	  getPolygon(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO, pmap);
	  getPolygon(aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP, pmap);
	  getPolygon(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE, pmap);
	  getPolygon(aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT, pmap);
	  getPolygon(aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP, pmap);
	  getPolygon(aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE, pmap);
	  getPolygon(aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT, pmap);
	  getPolygon(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP, pmap);
	  //               getSkel(aUsers[i],pmap);

	  // store this pmap in pmaps
	  pmaps.push_back(pmap);
	}
    }
}


//---------------------------------------------------------------------------
// PRIME SENSE FUNCTIONS
//---------------------------------------------------------------------------

// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("New User %d\n", nId);
  // New user found
  //	if (g_bNeedPose)
  //	{
  //		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
  //	}
  //	else{
  if(g_bhasCal)
    {
      g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, 0);
      g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
  else{  //never gotten calibration before
    //			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
    g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
  }
  //	}
}


// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
  printf("Lost user %d\n", nId);
}


// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie)
{
  printf("Pose %s detected for user %d\n", strPose, nId);
  g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
  if(g_bhasCal)
    {
      g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, 0);
      g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
  else{  //never gotten calibration before
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
}


// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
  printf("Calibration started for user %d\n", nId);
}


// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
  if (bSuccess)
    {
      // Calibration succeeded
      printf("Calibration complete, start tracking user %d\n", nId);
      g_bhasCal=TRUE;
      g_UserGenerator.GetSkeletonCap().SaveCalibrationData(nId, 0);
      g_UserGenerator.GetSkeletonCap().StartTracking(nId);
    }
  else
    {
      // Calibration failed
      printf("Calibration failed for user %d\n", nId);
      if (g_bNeedPose)
	{
	  g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
      else
	{
	  g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
    }
}


//---------------------------------------------------------------------------
// OPENGL FUNCTIONS
//---------------------------------------------------------------------------

// setup opengl, get skeleton points, get polygonal maps, and publish to ros
// this function is called each frame
void glutDisplay (void)
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // clear buffers (GL_COLOR_BUFFER_BIT and GL_DEPTH_BUFFER_BIT to preset values

  // Setup the OpenGL viewpoint
  glMatrixMode(GL_PROJECTION);  // specifies the GL_PROJECTION matrix as the target matrix for subsequent matrix operations
  glPushMatrix();  // sets the matrix on top of the stack to be identical to the one below it
  glLoadIdentity();  // sets the matrix on top of the stack to be the identity matrix

  xn::SceneMetaData sceneMD;
  xn::DepthMetaData depthMD;
  g_DepthGenerator.GetMetaData(depthMD);

  // glOrtho: multiplies the current matrix with an orthogonal matrix
  // arg1: coordinates of left clipping plane (0)
  // arg2: coordinates of right clipping plane (depthMD.XRes())
  // arg3: coordinates of bottom clipping plane (depthMD.YRes())
  // arg4: coordinates of top clipping plane (0)
  // arg5: nearer clipping plane (-1.0)
  // arg6: farther clipping plane (1.0)
  glOrtho(0, depthMD.XRes(), depthMD.YRes(), 0, -1.0, 1.0);

  glDisable(GL_TEXTURE_2D);  // disable two dimensional texturing

  // check to see if prime sense software is paused
  if (!g_bPause)
    {
      // Read next available data
      g_Context.WaitAndUpdateAll();
    }

  ros::Time tstamp=ros::Time::now();  // define Time variable from ros named tstamp and set it equal to current time

  // Process the data
  g_DepthGenerator.GetMetaData(depthMD);
  g_UserGenerator.GetUserPixels(0, sceneMD);
  DrawDepthMap(depthMD, sceneMD);  // call DrawDepthMap from SceneDrawer.cpp

  std::vector<mapping_msgs::PolygonalMap> pmaps;  // create a vector of PolygonalMap variables called pmaps
  body_msgs::Skeletons skels;  // create a Skeletons variable called skells from body_msgs
  getSkels(pmaps,skels);  // use getSkels to get polygonal maps and skeleton points for all users

  ROS_DEBUG("skels size %d \n",pmaps.size());  // print size of skels to ros under debug (use rxconsole?)

  // check to see if there are any users currently being tracked, fill in relevant information, and publish to ros
  if(pmaps.size())
    {
      skels.header.stamp=tstamp;
      skels.header.seq = depthMD.FrameID();
      skels.header.frame_id="/openni_depth_optical_frame";

      skel_pub.publish(skels);  // publish skels

      pmaps.front().header.stamp=tstamp;
      pmaps.front().header.seq = depthMD.FrameID();
      pmaps.front().header.frame_id="/openni_depth_optical_frame";

      pmap_pub.publish(pmaps[0]);  // publish pmaps
    }

  glutSwapBuffers();  // swap the buffers from the back buffer to the current window
}


// redisplay the current data because no new data is available
void glutIdle (void)
{
  if (g_bQuit) 
    {
      CleanupExit();
    }

  // Display the frame
  glutPostRedisplay();  // marks the current window as needing to be redisplayed
}


// keyboard shortcuts that choose what to display in the viewer
void glutKeyboard (unsigned char key, int x, int y)
{
  switch (key)
    {
    case 27:
      // Exit
      CleanupExit();
    case 'b':
      // Draw background?
      g_bDrawBackground = !g_bDrawBackground;
      break;
    case 'x':
      // Draw pixels at all?
      g_bDrawPixels = !g_bDrawPixels;
      break;
    case 's':
      // Draw Skeleton?
      g_bDrawSkeleton = !g_bDrawSkeleton;
      break;
    case 'i':
      // Print label?
      g_bPrintID = !g_bPrintID;
      break;
    case 'l':
      // Print ID & state as label, or only ID?
      g_bPrintState = !g_bPrintState;
      break;
    case'p':
      // Pause?
      g_bPause = !g_bPause;
      break;
    }
}


void glInit (int * pargc, char ** argv)
{
  // glutInit: initialize the glut library and take in argc and argv from main
  // arg1: number of command line arguments
  // arg2: array of command line arguments
  glutInit(pargc, argv);  

  // glutInitDisplayMode: sets the display mode 
  // arg1: bitmask for opengl display modes
  // GLUT_RGB: bit mask to select an RGBA mode window
  // GLUT_DOUBLE: bit mask to select a double buffered window
  // GLUT_DEPTH: bit mask to select a window with a depth buffer
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);  

  // glutInitWindowSize: sets window size
  // arg1: window width (defined at top)
  // arg2: window height (defined at top)
  glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);

  glutCreateWindow ("Prime Sense User Tracker Viewer");  // creates a top-level window
  //glutFullScreen();  // makes window full screen
  glutSetCursor(GLUT_CURSOR_NONE);  // changes the cursor image of the current window to invisible
  glutKeyboardFunc(glutKeyboard);  // sets the keyboard callback for the current window
  glutDisplayFunc(glutDisplay); // sets the display callback function for the current window
  glutIdleFunc(glutIdle);  // sets the global idle callback

  glDisable(GL_DEPTH_TEST);  // disable depth comparisons and do not update the depth buffer
  glEnable(GL_TEXTURE_2D);  // enable 2D shading
  glEnableClientState(GL_VERTEX_ARRAY);  // enable the vertex array is enabled for writing and used during rendering
  glDisableClientState(GL_COLOR_ARRAY);  // disable color arrays
}

//#define SAMPLE_XML_PATH "/home/garratt/ros/kinect/ni/openni/lib/SamplesConfig.xml"

// NEEDS COMMENT
// this is a macro and needs the backslashes at the end of the line
#define CHECK_RC(nRetVal, what)					 \					
if (nRetVal != XN_STATUS_OK)					 \	
  {								 \					
    printf("%s failed: %s\n", what, xnGetStatusString(nRetVal)); \
    return nRetVal;						 \						
  }


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  sleep(10);  // pause for 10 seconds to let openni_camera startup

  ros::init(argc, argv, "NUTracker");  // initialize ros node, take in argc and argv from main, and name it NUTracker
  ros::NodeHandle nh_;  // start ros node

  // Read the device_id parameter from the server
  int device_id;
  //   param_nh.param ("device_id", device_id, argc > 1 ? atoi (argv[1]) : 0);

  // create ros publishers for the PolygonalMaps and Skeletons data and set to publish every 100 ms
  pmap_pub = nh_.advertise<mapping_msgs::PolygonalMap> ("skeletonpmaps", 100);
  skel_pub = nh_.advertise<body_msgs::Skeletons> ("skeletons", 100);

  XnStatus nRetVal = XN_STATUS_OK;

  // check if config file name was specified in command line argument use specified config file if it was
  if (argc > 1)
    {
      nRetVal = g_Context.Init();
      CHECK_RC(nRetVal, "Init");
      nRetVal = g_Context.OpenFileRecording(argv[1]);  // open config file

      // check returned value from OpenFileRecording and print error if not OK
      if (nRetVal != XN_STATUS_OK)
	{
	  printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
	  return 1;
	}
    }

  // if no file name was specified in command line use default config
  else
    {
      std::string configFilename = ros::package::getPath("openni") + "/lib/SamplesConfig.xml";  // find default config file in openni package path
      nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
      CHECK_RC(nRetVal, "InitFromXml");
    }

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
  CHECK_RC(nRetVal, "Find depth generator");

  nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
  if (nRetVal != XN_STATUS_OK)
    {
      nRetVal = g_UserGenerator.Create(g_Context);
      CHECK_RC(nRetVal, "Find user generator");
    }

  XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;

  // check if user generator is supported
  if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
    {
      printf("Supplied user generator doesn't support skeleton\n");
      return 1;
    }

  g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
  g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

  if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
    {
      g_bNeedPose = TRUE;
      if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
	{
	  printf("Pose required, but not supported\n");
	  return 1;
	}
      g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);
      g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
    }

  g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

  nRetVal = g_Context.StartGeneratingAll();
  CHECK_RC(nRetVal, "StartGenerating");

  glInit(&argc, argv);  // initialize opengl and pass it the number of command line arguments and numbers
  glutMainLoop();  // start opengl main loop and stay there forever
}
