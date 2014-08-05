/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:
  
  QueryDevice.c

Description:

  This example demonstrates how to retrieve information from the haptic device.

*******************************************************************************/
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
# include "conio.h"
# include <string.h>
#endif

#include <stdio.h>
#include <assert.h>

#include <HD/hd.h>
#include <HL/hl.h>

#include <HDU/hduVector.h>
#include <HDU/hduQuaternion.h>
#include <HDU/hduError.h>

#include <vector>

#include <time.h>

/* Holds data retrieved from HDAPI. */
typedef struct 
{
    HDboolean m_buttonState;       /* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition; /* Current device coordinates. */
	hduVector3Dd m_gimbalRotation; /* Current gimbal rotation - x, y and z */
	HLdouble m_quaternion[4]; /* Quaternion defining the rotation */ 
    HDErrorInfo m_error;
} DeviceData;

static DeviceData gServoDeviceData;

/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK updateDeviceCallback(void *pUserData)
{   
    int nButtons = 0;

    hdBeginFrame(hdGetCurrentDevice());

    /* Retrieve the current button(s). */
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
    
    /* In order to get the specific button 1 state, we use a bitmask to
       test for the HD_DEVICE_BUTTON_1 bit. */
    gServoDeviceData.m_buttonState = 
        (nButtons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
        
    /* Get the current location of the device (HD_GET_CURRENT_POSITION)
       We declare a vector of three doubles since hdGetDoublev returns 
       the information in a vector of size 3. */
    hdGetDoublev(HD_CURRENT_POSITION, gServoDeviceData.m_devicePosition);

	/* Get the current quaternion defining the gimbal rotation */
	hlGetDoublev(HL_DEVICE_ROTATION, gServoDeviceData.m_quaternion);

    /* Also check the error state of HDAPI. */
    gServoDeviceData.m_error = hdGetError();

    /* Copy the position into our device_data tructure. */
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_CONTINUE;    
}


/*******************************************************************************
 Checks the state of the gimbal button and gets the position of the device.
*******************************************************************************/
HDCallbackCode HDCALLBACK copyDeviceDataCallback(void *pUserData)
{
    DeviceData *pDeviceData = (DeviceData *) pUserData;

    memcpy(pDeviceData, &gServoDeviceData, sizeof(DeviceData));

    return HD_CALLBACK_DONE;
}


/*******************************************************************************
 Prints out a help string about using this example.
*******************************************************************************/
void printHelp(void)
{
    static const char help[] = {"\
Press and release the stylus button to print out the current device location.\n\
Press and hold the stylus button to exit the application\n"};

    fprintf(stdout, "%s\n", help);
}


/*******************************************************************************
 This routine allows the device to provide information about the current 
 location of the stylus, and contains a mechanism for terminating the 
 application.  
 Pressing the button causes the application to display the current location
 of the device.  
 Holding the button down for N iterations causes the application to exit. 
*******************************************************************************/
void mainLoop(void)
{ 
	FILE *ofp;
	char *mode = "w";
	char outputFilename[] = "C:\\Users\\robo328\\Desktop\\Haptics_Project\\data.txt"; 

    static const int kTerminateCount = 1000;
    int buttonHoldCount = 0;

    /* Instantiate the structure used to capture data from the device. */
    DeviceData currentData;
    DeviceData prevData;

	/* Vectors to store the device position and rotation */
	//HLdouble rotationVector[4]; //Quaternion
	//HLdouble positionVector[3];

	/* Get the time at the start of the loop - Windows Specific */
	SYSTEMTIME begin;
	GetSystemTime(&begin);

    /* Perform a synchronous call to copy the most current device state. */
    hdScheduleSynchronous(copyDeviceDataCallback, 
        &currentData, HD_MIN_SCHEDULER_PRIORITY);

    memcpy(&prevData, &currentData, sizeof(DeviceData));    

    printHelp();

	ofp = fopen(outputFilename, "w");

	if (ofp == NULL) {
		fprintf(stderr, "Can't open output file %s!\n", outputFilename);
	 exit(1);
	}

    /* Run the main loop until the gimbal button is held. */
    while (1)
    {
		/* Get the time right now */
		SYSTEMTIME now;
		GetSystemTime(&now);

		/* Get and assign the rotation and position vectors */
		//hlGetDoublev(HL_DEVICE_ROTATION, rotationVector);
		//hlGetDoublev(HL_DEVICE_POSITION, positionVector);

		/* Get the time difference in milliseconds from the start of the loop 
		This isn't working because the word declaration isn't right so I put the
		entire expression for printing, for now */
		/* WORD diff;
		diff = (now.wSecond * 1000 + now.wMilliseconds) - 
			(begin.wSecond * 1000 + begin.wMilliseconds); */

        /* Perform a synchronous call to copy the most current device state.
           This synchronous scheduler call ensures that the device state
           is obtained in a thread-safe manner. */


        hdScheduleSynchronous(copyDeviceDataCallback,
                              &currentData,
                              HD_MIN_SCHEDULER_PRIORITY);

        if (currentData.m_buttonState && prevData.m_buttonState)
        {
            /* Keep track of how long the user has been pressing the button.
               If this exceeds N ticks, then terminate the application. */
            buttonHoldCount++;

            if (buttonHoldCount > kTerminateCount)
            {
                /* Quit, since the user held the button longer than
                   the terminate count. */
				fclose(ofp);
                break;
            }
        }
        else if (!currentData.m_buttonState && prevData.m_buttonState)
        {
            /* Reset the button hold count, since the user stopped holding
               down the stylus button. */
            buttonHoldCount = 0;
        }

		/* If the user depresses the gimbal button, display the current 
        location information. */
        /*if (currentData.m_buttonState && !prevData.m_buttonState)*/
		else 
        {           
            fprintf(stdout, "(r1, r2, r3, w, x, y, z, time): (%g, %g, %g, %g, %g, %g, %g, %ld)\n", 
                currentData.m_devicePosition[0], 
                currentData.m_devicePosition[1], 
                currentData.m_devicePosition[2],
				currentData.m_quaternion[0],
				currentData.m_quaternion[1],
				currentData.m_quaternion[2],
				currentData.m_quaternion[3],
				(now.wMinute*60*1000 + now.wSecond*1000 + now.wMilliseconds) - 
				(begin.wMinute*60*1000 + begin.wSecond * 1000 + begin.wMilliseconds));

			/* Also write the coordinates to the file */

			fprintf(stdout, "(r1, r2, r3, w, x, y, z, time): (%g, %g, %g, %g, %g, %g, %g, %ld)\n", 
                currentData.m_devicePosition[0], 
                currentData.m_devicePosition[1], 
                currentData.m_devicePosition[2],
				currentData.m_quaternion[0],
				currentData.m_quaternion[1],
				currentData.m_quaternion[2],
				currentData.m_quaternion[3],
				(now.wMinute*60*1000 + now.wSecond*1000 + now.wMilliseconds) - 
				(begin.wMinute*60*1000 + begin.wSecond * 1000 + begin.wMilliseconds)); 
        }
        
        /* Check if an error occurred. */
        if (HD_DEVICE_ERROR(currentData.m_error))
        {
            hduPrintError(stderr, &currentData.m_error, "Device error detected");

            if (hduIsSchedulerError(&currentData.m_error))
            {
                /* Quit, since communication with the device was disrupted. */
                fprintf(stderr, "\nPress any key to quit.\n");
                getch();                
                break;
            }
        }

        /* Store off the current data for the next loop. */
        memcpy(&prevData, &currentData, sizeof(DeviceData));    
    }
}

/*******************************************************************************
 Main function.
 Sets up the device, runs main application loop, cleans up when finished.
*******************************************************************************/
int main(int argc, char* argv[])
{
    HDSchedulerHandle hUpdateHandle = 0;
    HDErrorInfo error;

    /* Initialize the device, must be done before attempting to call any hd 
       functions. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize the device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }

    /* Schedule the main scheduler callback that updates the device state. */
    hUpdateHandle = hdScheduleAsynchronous(
        updateDeviceCallback, 0, HD_MAX_SCHEDULER_PRIORITY);

    /* Start the servo loop scheduler. */
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;           
    }
    
    /* Run the application loop. */
    mainLoop();

    /* For cleanup, unschedule callbacks and stop the servo loop. */
    hdStopScheduler();
    hdUnschedule(hUpdateHandle);
    hdDisableDevice(hHD);

    return 0;
}

/******************************************************************************/
