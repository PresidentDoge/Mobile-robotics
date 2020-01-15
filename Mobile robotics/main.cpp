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

		/* variables */

		//random seed
		srand(time(NULL));

		//time
		int timeElapsed = 0;
		simxInt starttime = extApi_getTimeInMs();
		
		//initialize consts speed/distances
		const float speed = 1;               // Default motor speed 
		const float STOPPING_DIST = 0.25;    // Minimum distance, takes evasive action
		const float SAFE_DIST = 0.5;         // Maximum distance threshold of evasive actions 
		const float FOLLOW_DIST = 0.45;      // Distance maintained from the wall while following

		const float MIN_DIST = -1.0f;        // Minimum error distance 
		const float MAX_DIST = 1.0f;         // Maximum error distance

		//initialize sensor array
		float sensorArray[15]; 
		float* sensor;
		
		//boolean logic
		PROGRAM_RUNNING = true;
		bool followingLeft = false;
		bool followingRight = false;
		bool followingWall = false;
		

		/* Create handles */

		//beacon handles
		simxGetObjectHandle(clientID, "beacon", &beaconHandle, simx_opmode_blocking);
		simxGetDistanceHandle(clientID, "beacon", &distanceHandle, simx_opmode_blocking);
		simxReadDistance(clientID, distanceHandle, distance, simx_opmode_streaming);
		//motor handles
		simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &leftmotorHandle, simx_opmode_oneshot_wait);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &rightmotorHandle, simx_opmode_oneshot_wait);
		//sensor handles
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor0", &senorHandle[0], simx_opmode_blocking); //Left-Front Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor1", &senorHandle[1], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor2", &senorHandle[2], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor3", &senorHandle[3], simx_opmode_blocking); //Front-Left Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor4", &senorHandle[4], simx_opmode_blocking); //Front-Right Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor5", &senorHandle[5], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor6", &senorHandle[6], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor7", &senorHandle[7], simx_opmode_blocking); //Right-Front Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor8", &senorHandle[8], simx_opmode_blocking); //Right-Rear Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor9", &senorHandle[9], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor10", &senorHandle[10], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor11", &senorHandle[11], simx_opmode_blocking); //Rear-Right Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor12", &senorHandle[12], simx_opmode_blocking); //Rear-Left Sensor
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor13", &senorHandle[13], simx_opmode_blocking);
		simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor15", &senorHandle[15], simx_opmode_blocking); //Left-Rear Sensor
		FSM state = WANDER;
		//followState edgeFollowState = IDLE;

		//Initialize states

		// Start Motors
		motorControl(speed, speed);

		/* Main loop */

		while(PROGRAM_RUNNING)
		{

			sensor = fillarr(sensorArray, 16); //create array of sonar reading


			/*for (int i = 0; i < INT_MAX; i++)
			{
				printf("%F \n", findBeacon());
			}*/

			

			
			switch (state)
			{
			case WANDER:  //Search for wall
				
				motorControl(speed, speed);

				// If front sensors detect wall
				if (getMean(3, 4) < FOLLOW_DIST) 
				{
					int randomNumber = rand();

					if (randomNumber == 0)
					{
						//motorControl(-speed, speed);
						followingLeft = true;
						state = FOLLOW;
					}

					if (randomNumber == 1)
					{
						//motorControl(speed, -speed);
						followingRight = true;
						state = FOLLOW;
					}
				}

				//if left sensors detect wall
				if (getMean(0, 15) < FOLLOW_DIST)
				{
					followingLeft = true;
					state = FOLLOW;
				}

				//if right sensors detect wall
				if (getMean(7, 8) < FOLLOW_DIST)
				{
					followingRight = true;
					state = FOLLOW;
				}
			
			case AVOID: //Avoid wall

				break;
			case FOLLOW: //Follow wall

				break;
			case CENTER_WALL: //Find Center of wall

				break;
			case CENTER_ROOM: //Find center of room

				break;
			}

			


			


			

			
			

			//check if robot is at the center of the two walls
			/*if (x < 1 && x > -1)
			{
				printf("true \n");
			}
			else printf("false \n");
	        */
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
	float jLength = (unsigned int)detectionState[0] == 1 ? std::sqrt((detectedPoint[0] * detectedPoint[0]) + (detectedPoint[1] * detectedPoint[1]) + (detectedPoint[2] * detectedPoint[2])) : 1.0f;
	return jLength;
}


//polls all 16 sonar sensors at once and create an array of readings
float * fillarr(float arr[], int length) {
	for (int i = 0; i < length; ++i) {
		arr[i] = getSensorReading(i); //Sensor readings
	}
	return arr;
}

void wait(int time)
{
	extApi_sleepMs(time);
}

float getMean(float sensor1, float sensor2)
{
	return ((sensor1 + sensor2)/2);
}

float getError(float sensor1, float sensor2)
{
	float x = getSensorReading(sensor1);
	float y = getSensorReading(sensor2);
	return (x-y);
}


simxFloat findBeacon()
{
	double threshold = 7;

	simxReadDistance(clientID, distanceHandle, distance, simx_opmode_buffer);
	simxGetObjectPosition(clientID, beaconHandle, -1, beaconPosition, simx_opmode_blocking);

	return std::sqrt((distance[0] * distance[0]) + (distance[1] * distance[1]) + (distance[2] * distance[2]));

}