/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:

  HelloSphere.cpp

Description: 

  This example demonstrates basic haptic rendering of a shape.

******************************************************************************/
//ROS includes 
#include <string>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <stdlib.h>
#include <math.h>
#include <assert.h>

#if defined(WIN32)
#include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <GLUT/glut.h>
#endif

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>

#include <HLU/hlu.h>

#include <vector>

#include <time.h>

/* Haptic device and rendering context handles. */
static HHD ghHD = HD_INVALID_HANDLE;
static HHLRC ghHLRC = 0;

/* Structure with position and rotation data from the device */
typedef struct 
{
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
	HLdouble m_quaternion[4]; /* Quaternion defining the rotation */ 
} DevicePose;

/* Structure with force and torque data from the device */
typedef struct
{
	HDdouble forceValues[3];
    HDdouble jointTorqueValues[3];   
    HDdouble gimbalTorqueValues[3]; 
} DeviceForces;

static DevicePose poseData;
static DeviceForces forceData;
static DeviceForces verForceData; // to verify force data --> delete later 

/* Initialize the output file for the data */
FILE *ofp;
char *mode = "w";
char outputFilename[] = "C:\\Users\\robo328\\Desktop\\Haptics_Project\\data.txt";

/* Time at the start of the program - Windows Specific */
SYSTEMTIME begin;

/* Publisher for ROS */
ros::Publisher pose; 

/* Listener for forces */
ros::Subscriber listener; 

/*******************************************************************************
 ROS code for publishing messages. 
*******************************************************************************/
void rosPubPose() 
{
    geometry_msgs::Pose pose_msg;
    geometry_msgs::Point point_msg;
    geometry_msgs::Quaternion or_msg;

    point_msg.x = poseData.m_devicePosition[0];
    point_msg.y = poseData.m_devicePosition[1];
    point_msg.z = poseData.m_devicePosition[2];

    or_msg.x = poseData.m_quaternion[0];
    or_msg.y = poseData.m_quaternion[1];
    or_msg.z = poseData.m_quaternion[2];
    or_msg.w = poseData.m_quaternion[3];

    pose_msg.position = point_msg;
    pose_msg.orientation = or_msg;

	pose.publish(pose_msg); 
}


/*******************************************************************************
Gets the position and orientation information using HLAPI. 
*******************************************************************************/
void hlPoseInfo()
{
	/* Get the time right now */
	SYSTEMTIME now;
	GetSystemTime(&now);

    HLerror error;

    while (HL_ERROR(error = hlGetError()))
    {
        fprintf(stderr, "HL Error: %s\n", error.errorCode);
        
        if (error.errorCode == HL_DEVICE_ERROR)
        {
            hduPrintError(stderr, &error.errorInfo,
                "Error during haptic rendering\n");
        }
    }

	/* Set the current quaternion and device position */
	hlGetDoublev(HL_DEVICE_ROTATION, poseData.m_quaternion);
	hdGetDoublev(HD_CURRENT_POSITION, poseData.m_devicePosition); 

	////fprintf(ofp, "(x, y, z, r1, r2, r3, w, time): (%g, %g, %g, %g, %g, %g, %g, %ld)\n",
	//fprintf(stdout, "%g, %g, %g, %g, %g, %g, %g, %ld\n",
 //               poseData.m_devicePosition[0], 
 //               poseData.m_devicePosition[1], 
 //               poseData.m_devicePosition[2],
	//			poseData.m_quaternion[0],
	//			poseData.m_quaternion[1],
	//			poseData.m_quaternion[2],
	//			poseData.m_quaternion[3],
	//			(now.wMinute*60*1000 + now.wSecond*1000 + now.wMilliseconds) - 
	//			(begin.wMinute*60*1000 + begin.wSecond * 1000 + begin.wMilliseconds));
}


/*******************************************************************************
 Initialize the HDAPI.  This involves initing a device configuration, enabling
 forces, and scheduling a haptic thread callback for servicing the device.
*******************************************************************************/
void initHL()
{
    HDErrorInfo error;

    ghHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "Press any key to exit");
        getchar();
        exit(-1);
    }
    
    ghHLRC = hlCreateContext(ghHD);
    hlMakeCurrent(ghHLRC);

}

/*******************************************************************************
 This handler is called when the application is exiting.  Deallocates any state 
 and cleans up.
*******************************************************************************/
void exitHandler()
{
    // Free up the haptic rendering context.
    hlMakeCurrent(NULL);
    if (ghHLRC != NULL)
    {
        hlDeleteContext(ghHLRC);
    }

    // Free up the haptic device.
    if (ghHD != HD_INVALID_HANDLE)
    {
        hdDisableDevice(ghHD);
    }
}

/*******************************************************************************
Read out forces to verify if they are the same as the ones applied.  
*******************************************************************************/
void verifyForces() 
{
	ghHD = hdGetCurrentDevice();
	hdBeginFrame(ghHD);

	hdGetDoublev(HD_CURRENT_FORCE, verForceData.forceValues);
	fprintf(stdout, "%g, %g, %g \n", verForceData.forceValues[0], verForceData.forceValues[1], verForceData.forceValues[2]);

	hdEndFrame(ghHD);
}

/*******************************************************************************
 Code to apply force feedback to the haptic device. 
*******************************************************************************/
void applyForce()
{
	ghHD = hdGetCurrentDevice();
	hdBeginFrame(ghHD);

	forceData.forceValues[0] = -0.3;
	forceData.forceValues[1] = -0.3;
	forceData.forceValues[2] = -0.3;

	hdSetDoublev(HD_CURRENT_FORCE, forceData.forceValues);
	//fprintf(ofp, "%g, %g, %g \n", forceData.forceValues[0], forceData.forceValues[1], forceData.forceValues[2]);

	hdEndFrame(ghHD);
}

/*******************************************************************************
 HD code to apply force feedback to the haptic device. 
*******************************************************************************/
HDCallbackCode HDCALLBACK applyForceHD(void* pUserData)
{
	ghHD = hdGetCurrentDevice();
	hdBeginFrame(ghHD);

	forceData.forceValues[0] = -0.1;
	forceData.forceValues[1] = -0.1;
	forceData.forceValues[2] = -0.1;

	hdSetDoublev(HD_CURRENT_FORCE, forceData.forceValues);
	//fprintf(ofp, "%g, %g, %g \n", forceData.forceValues[0], forceData.forceValues[1], forceData.forceValues[2]);

	hdEndFrame(ghHD);

	return HD_CALLBACK_CONTINUE; 
}
/*******************************************************************************
 ROS Callback code for listening to forces. *Listening to pose right now*
*******************************************************************************/
void forceCallback(const geometry_msgs::Pose::ConstPtr& force) 
{
	ROS_INFO("x coordinate for pose: [%g]", force->position.x);
}

/*******************************************************************************
 Initializes GLUT for displaying a simple haptic scene.
*******************************************************************************/
int main(int argc, char *argv[])
{
	/* Assign the beginning time of the program */
	GetSystemTime(&begin);

	/* Open the output file and check for NULL */
	ofp = fopen(outputFilename, "w");

	if (ofp == NULL) {
		fprintf(stderr, "Can't open output file %s!\n", outputFilename);
	 exit(1);
	}  

	initHL(); // Start HL

    // Provide a cleanup routine for handling application exit.
    atexit(exitHandler);

	/* Publish ROS messages */
	std::string name("talker");
    ros::init(argc, argv, name);
    ros::NodeHandle n;
    pose = n.advertise<geometry_msgs::Pose>("pose", 1000);

	/* Create a listener for forces also */
	std::string name2("listener");
	ros::init(argc, argv, name2);
	ros::NodeHandle n2;
	listener = n2.subscribe("pose", 1000, forceCallback);


	/* Synchornous call to update the force applied to the device */
	hdScheduleAsynchronous(applyForceHD, 
        (void*) 0, HD_DEFAULT_SCHEDULER_PRIORITY);

	ros::Rate rate(1000.0);
	while (ros::ok()){
		hlBeginFrame();

		rosPubPose();
		hlPoseInfo();
		verifyForces(); // to confirm if forces are the ones applied 
		//applyForce(); 

		hlEndFrame();

		//ros::spinOnce(); 
		rate.sleep();
  }

	/* Close the file */
	fclose(ofp);

    return 0;
}
/******************************************************************************/
