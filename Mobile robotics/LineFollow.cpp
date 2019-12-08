//
// simRemoteApi.start(19999)
//

#include <stdio.h>
#include <stdlib.h>
#include "LineFollow.h"
#include <iostream>
#include <time.h>
#include <math.h> 


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

		//Create handles
		createObjectHandles();

		/* variables */
		//random seed
		srand(time(NULL));
		//time
		int timeElapsed = 0;
		simxInt starttime = extApi_getTimeInMs();
		//initialize array
		float sensorArray[15]; 
		float* sensor;
		//boolean logic
		PROGRAM_RUNNING = true;
		bool LEFT_HOME = false;

		// Start Motors
		motorControl(SPEED, SPEED);

		/* Main loop */

		while(PROGRAM_RUNNING)
		{
			printf("%f : %f \n", leftSensors(), rightSensors());
			//if (leftSensors() < ? ? && rightSensors() < ? ? ) LEFT_HOME = true;
	
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
	float jLength = (unsigned int)detectionState[0] == 1 ? std::sqrt((detectedPoint[0] * detectedPoint[0]) + (detectedPoint[1] * detectedPoint[1]) + (detectedPoint[2] * detectedPoint[2])) : MAX_DIST;
	return jLength;
}


//This function polls all 16 sonar sensors at once to create an array of sensor readings
float * fillarr(float arr[], int length) {
	for (int i = 0; i < length; ++i) {
		arr[i] = getSensorReading(i); //Sensor readings
	}
	return arr;
}

double frontSensors()
{
	return (getSensorReading(3) + getSensorReading(4)) / 2;
}

double leftSensors()
{
	return (getSensorReading(0) + getSensorReading(15)) / 2;
}

double rightSensors()
{
	return (getSensorReading(7) + getSensorReading(8)) / 2;
}

double rearSensors()
{
	return (getSensorReading(11) + getSensorReading(12)) / 2;
}


void wait(int time)
{
	extApi_sleepMs(time);
}

void createObjectHandles()
{
	//beacon handles
	simxGetObjectHandle(clientID, "beacon", &beaconHandle, simx_opmode_blocking);
	simxGetDistanceHandle(clientID, "beacon", &distanceHandle, simx_opmode_blocking);
	simxReadDistance(clientID, distanceHandle, distance, simx_opmode_streaming);
	//motor handles
	simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &leftmotorHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &rightmotorHandle, simx_opmode_oneshot_wait);
	//sensor handles
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor0", &senorHandle[0], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor1", &senorHandle[1], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor2", &senorHandle[2], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor3", &senorHandle[3], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor4", &senorHandle[4], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor5", &senorHandle[5], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor6", &senorHandle[6], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor7", &senorHandle[7], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor8", &senorHandle[8], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor9", &senorHandle[9], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor10", &senorHandle[10], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor11", &senorHandle[11], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor12", &senorHandle[12], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor13", &senorHandle[13], simx_opmode_blocking);
	simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor15", &senorHandle[15], simx_opmode_blocking);
}

simxFloat findBeacon()
{
	double threshold = 7;

	simxReadDistance(clientID, distanceHandle, distance, simx_opmode_buffer);
	simxGetObjectPosition(clientID, beaconHandle, -1, beaconPosition, simx_opmode_blocking);

	return std::sqrt((distance[0] * distance[0]) + (distance[1] * distance[1]) + (distance[2] * distance[2]));
}
