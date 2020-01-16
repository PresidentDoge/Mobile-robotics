//
// simRemoteApi.start(19999)
//

#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include <iostream>
#include <time.h>

extern "C" {
#include "extApi.h"
}

int main(int argc, char* argv[])
{
	simxFinish(-1); // Clean up previous instances of VREP

	int clientID = simxStart((simxChar*)"127.0.0.1", 19999, true, true, 2000, 5);
	if (clientID != -1)
	{
		// Output connection success
		printf("Connected to remote API server\n");

		// Send text to VREP console (Non-blocking)
		simxAddStatusbarMessage(clientID, "Program START", simx_opmode_oneshot);

		// Sim start
		simxStartSimulation(clientID, simx_opmode_oneshot_wait); //starts simulation without having to tab to vrep

		//variables
		srand(time(NULL));		//seed for rand()
		const float speed = 1;  //speed of robot
		PROGRAM_RUNNING = true; //boolean logic for while loop
		int timeElapsed = 0;    //init time
		simxInt starttime = extApi_getTimeInMs(); //current time

		/* Create handles */

		//motor handles
		int leftMotor = simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &leftmotorHandle, simx_opmode_oneshot_wait);
		int rightMotor = simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &rightmotorHandle, simx_opmode_oneshot_wait);
		//sensor handles
		int frontSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor5", &senorHandle[0], simx_opmode_blocking);
		int leftSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor1", &senorHandle[1], simx_opmode_blocking);
		int rightSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor8", &senorHandle[2], simx_opmode_blocking);
		int backSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor12", &senorHandle[3], simx_opmode_blocking);


		// Start Motors
		motorControl(speed, speed);

		/* Main loop */

		while(PROGRAM_RUNNING)
		{

			
		}

		// Stop the motors
		motorControl(0, 0);

		// Send text to VREP console (Non-blocking)
		simxAddStatusbarMessage(clientID, "Program END", simx_opmode_oneshot);

		// Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
		int pingTime;
		simxGetPingTime(clientID, &pingTime);

		// Resets all dynamic objects back to original positions
		int simResetDynamicObject(sim_handle_all);

		// Stops sim
		simxStopSimulation(clientID, simx_opmode_oneshot_wait);

		// close the connection to V-REP:   
		simxFinish(clientID);
	}
	return(0);
}


void motorControl(float leftMotor, float rightMotor)
{
	// targetVelocity is rad/s for a revolute joint
	simxSetJointTargetVelocity(clientID, leftmotorHandle, leftMotor, simx_opmode_blocking);
	simxSetJointTargetVelocity(clientID, rightmotorHandle, rightMotor, simx_opmode_blocking);
}


float getSensorReading(int sensor)
{
	simxReadProximitySensor(clientID, senorHandle[sensor], detectionState, detectedPoint, NULL, NULL, simx_opmode_streaming);
	float jLength = (unsigned int)detectionState[0] == 1 ? std::sqrt((detectedPoint[0] * detectedPoint[0]) + (detectedPoint[1] * detectedPoint[1]) + (detectedPoint[2] * detectedPoint[2])) : 5.0f;
	return jLength;
}


void wait(int time)
{
	extApi_sleepMs(time);
}
