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

/* Shape id for shape we will render haptically. */
HLuint gSphereShapeId;

#define CURSOR_SIZE_PIXELS 20
static double gCursorScale;
static GLuint gCursorDisplayList = 0;

/* Structure with position and rotation data from the device */
typedef struct 
{
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
	HLdouble m_quaternion[4]; /* Quaternion defining the rotation */ 
} DevicePose;

typedef struct
{
	HDdouble forceValues[3];
    HDdouble jointTorqueValues[3];   
    HDdouble gimbalTorqueValues[3]; 
} DeviceForces;

static DevicePose poseData;
static DeviceForces forceData; 

/* Initialize the output file for the data */
FILE *ofp;
char *mode = "w";
char outputFilename[] = "C:\\Users\\robo328\\Desktop\\Haptics_Project\\data.txt";

/* Time at the start of the program - Windows Specific */
SYSTEMTIME begin;

/* Publisher for ROS */
ros::Publisher pose; 

/* Function prototypes. */
void glutDisplay(void);
void glutReshape(int width, int height);
void glutIdle(void);   
void glutMenu(int);

void rosCall(void); 

void exitHandler(void);

void initGL();
void initHL();
void initScene();
void drawSceneHaptics();
void drawSceneGraphics();
void drawCursor();
void updateWorkspace();

/*******************************************************************************
 HD callback code to apply force feedback to the haptic device. 
*******************************************************************************/
HDCallbackCode HDCALLBACK applyForce(void *pUserData)
{
	HHD hHD = hdGetCurrentDevice();
	hdBeginFrame(hHD);

	forceData.forceValues[0] = -0.3;
	forceData.forceValues[1] = -0.3;
	forceData.forceValues[2] = -0.3;

	hdSetDoublev(HD_CURRENT_FORCE, forceData.forceValues);
	fprintf(stdout, "%g, %g, %g \n", forceData.forceValues[0], forceData.forceValues[1], forceData.forceValues[2]);

	hdEndFrame(hHD);

    return HD_CALLBACK_CONTINUE;
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

    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

    glutInitWindowSize(500, 500);
    glutCreateWindow("HelloSphere Example");

    // Set glut callback functions.
    glutDisplayFunc(glutDisplay);
    glutReshapeFunc(glutReshape);
    glutIdleFunc(glutIdle);
    
    glutCreateMenu(glutMenu);
    glutAddMenuEntry("Quit", 0);
    glutAttachMenu(GLUT_RIGHT_BUTTON);    
    
    // Provide a cleanup routine for handling application exit.
    atexit(exitHandler);

    initScene();

	/* Publish ROS messages */
	std::string name("talker");
 
    ros::init(argc, argv, name);

    ros::NodeHandle n;

    pose = n.advertise<geometry_msgs::Pose>("pose", 1000);

	/* Synchornous call to update the force applied to the device */
	hdScheduleAsynchronous(applyForce, 
        &poseData, HD_MIN_SCHEDULER_PRIORITY);

    glutMainLoop(); // Starts all registered callbacks 

	/* Close the file */
	fclose(ofp);

    return 0;
}

/*******************************************************************************
 ROS code for publishing messages. 
*******************************************************************************/
void rosCall() 
{
  if (ros::ok())
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
}

/*******************************************************************************
 GLUT callback for redrawing the view.
*******************************************************************************/
void glutDisplay()
{   
    drawSceneHaptics();

    drawSceneGraphics();

    glutSwapBuffers();
}

/*******************************************************************************
 GLUT callback for reshaping the window.  This is the main place where the 
 viewing and workspace transforms get initialized.
*******************************************************************************/
void glutReshape(int width, int height)
{
    static const double kPI = 3.1415926535897932384626433832795;
    static const double kFovY = 40;

    double nearDist, farDist, aspect;

    glViewport(0, 0, width, height);

    // Compute the viewing parameters based on a fixed fov and viewing
    // a canonical box centered at the origin.

    nearDist = 1.0 / tan((kFovY / 2.0) * kPI / 180.0);
    farDist = nearDist + 2.0;
    aspect = (double) width / height;
   
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(kFovY, aspect, nearDist, farDist);

    // Place the camera down the Z axis looking at the origin.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();            
    gluLookAt(0, 0, nearDist + 1.0,
              0, 0, 0,
              0, 1, 0);
    
    updateWorkspace();
}

/*******************************************************************************
 GLUT callback for idle state.  Use this as an opportunity to request a redraw.
 Checks for HLAPI errors that have occurred since the last idle check.
*******************************************************************************/
void glutIdle()
{
	rosCall();

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

	//forceData.forceValues[0] = 1;
	//forceData.forceValues[1] = 1;
	//forceData.forceValues[2] = 1;

	//hdSetDoublev(HD_CURRENT_FORCE, forceData.forceValues);
	fprintf(stdout, "%g, %g, %g \n", forceData.forceValues[0], forceData.forceValues[1], forceData.forceValues[2]);

    glutPostRedisplay();
}

/******************************************************************************
 Popup menu handler.
******************************************************************************/
void glutMenu(int ID)
{
    switch(ID) {
        case 0:
            exit(0);
            break;
    }
}

/*******************************************************************************
 Initializes the scene.  Handles initializing both OpenGL and HL.
*******************************************************************************/
void initScene()
{
    initGL();
    initHL();
}

/*******************************************************************************
 Sets up general OpenGL rendering properties: lights, depth buffering, etc.
*******************************************************************************/
void initGL()
{
    static const GLfloat light_model_ambient[] = {0.3f, 0.3f, 0.3f, 1.0f};
    static const GLfloat light0_diffuse[] = {0.9f, 0.9f, 0.9f, 0.9f};   
    static const GLfloat light0_direction[] = {0.0f, -0.4f, 1.0f, 0.0f};    
    
    // Enable depth buffering for hidden surface removal.
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    
    // Cull back faces.
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    
    // Setup other misc features.
    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    
    // Setup lighting model.
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);    
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, light_model_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
    glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
    glEnable(GL_LIGHT0);   
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

    // Enable optimization of the viewing parameters when rendering
    // geometry for OpenHaptics.
    hlEnable(HL_HAPTIC_CAMERA_VIEW);

    // Generate id for the shape.
    gSphereShapeId = hlGenShapes(1);

    hlTouchableFace(HL_FRONT);
}

/*******************************************************************************
 This handler is called when the application is exiting.  Deallocates any state 
 and cleans up.
*******************************************************************************/
void exitHandler()
{
    // Deallocate the sphere shape id we reserved in initHL.
    hlDeleteShapes(gSphereShapeId, 1);

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
 Use the current OpenGL viewing transforms to initialize a transform for the
 haptic device workspace so that it's properly mapped to world coordinates.
*******************************************************************************/
void updateWorkspace()
{
    GLdouble modelview[16];
    GLdouble projection[16];
    GLint viewport[4];

    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    glGetIntegerv(GL_VIEWPORT, viewport);

    hlMatrixMode(HL_TOUCHWORKSPACE);
    hlLoadIdentity();
    
    // Fit haptic workspace to view volume.
    hluFitWorkspace(projection);

    // Compute cursor scale.
    gCursorScale = hluScreenToModelScale(modelview, projection, viewport);
    gCursorScale *= CURSOR_SIZE_PIXELS;
}

/*******************************************************************************
 The main routine for displaying the scene.  Gets the latest snapshot of state
 from the haptic thread and uses it to display a 3D cursor.
*******************************************************************************/
void drawSceneGraphics()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);           

    // Draw 3D cursor at haptic device position.
    drawCursor();

    // Draw a sphere using OpenGL.
    //glutSolidSphere(0.5, 32, 32);
}

/*******************************************************************************
 The main routine for rendering scene haptics.
*******************************************************************************/
void drawSceneHaptics()
{    
    // Start haptic frame.  (Must do this before rendering any haptic shapes.)
    hlBeginFrame();

    // Set material properties for the shapes to be drawn.
    hlMaterialf(HL_FRONT_AND_BACK, HL_STIFFNESS, 0.7f);
    hlMaterialf(HL_FRONT_AND_BACK, HL_DAMPING, 0.1f);
    hlMaterialf(HL_FRONT_AND_BACK, HL_STATIC_FRICTION, 0.2f);
    hlMaterialf(HL_FRONT_AND_BACK, HL_DYNAMIC_FRICTION, 0.3f);

    // Start a new haptic shape.  Use the feedback buffer to capture OpenGL 
    // geometry for haptic rendering.
    hlBeginShape(HL_SHAPE_FEEDBACK_BUFFER, gSphereShapeId);

    // Use OpenGL commands to create geometry.
    //glutSolidSphere(0.5, 32, 32);

    // End the shape.
    hlEndShape();

    // End the haptic frame.
    hlEndFrame();
}


/*******************************************************************************
 Draws a 3D cursor for the haptic device using the current local transform,
 the workspace to world transform and the screen coordinate scale.
*******************************************************************************/
void drawCursor()
{
    static const double kCursorRadius = 0.5;
    static const double kCursorHeight = 1.5;
    static const int kCursorTess = 15;
    HLdouble proxyxform[16];

    GLUquadricObj *qobj = 0;

    glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT);
    glPushMatrix();

    if (!gCursorDisplayList)
    {
        gCursorDisplayList = glGenLists(1);
        glNewList(gCursorDisplayList, GL_COMPILE);
        qobj = gluNewQuadric();
               
        gluCylinder(qobj, 0.0, kCursorRadius, kCursorHeight,
                    kCursorTess, kCursorTess);
        glTranslated(0.0, 0.0, kCursorHeight);
        gluCylinder(qobj, kCursorRadius, 0.0, kCursorHeight / 5.0,
                    kCursorTess, kCursorTess);
    
        gluDeleteQuadric(qobj);
        glEndList();
    }
    
    // Get the proxy transform in world coordinates.
    hlGetDoublev(HL_PROXY_TRANSFORM, proxyxform);
    glMultMatrixd(proxyxform);

    // Apply the local cursor scale factor.
    glScaled(gCursorScale, gCursorScale, gCursorScale);

    glEnable(GL_COLOR_MATERIAL);
    glColor3f(0.0, 0.5, 1.0);

    glCallList(gCursorDisplayList);

    glPopMatrix(); 
    glPopAttrib();
}

/******************************************************************************/
