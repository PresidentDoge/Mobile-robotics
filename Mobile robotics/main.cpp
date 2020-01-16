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
		srand(time(NULL));
		const float speed = 1;               // Default motor speed 
		const float STOPPING_DIST = 0.25;    // Minimum distance, takes evasive action
		const float SAFE_DIST = 0.5;         // Maximum distance threshold of evasive actions 
		const float FOLLOW_DIST = 0.45;      // Distance maintained from the wall while following

		const float MIN_DIST = -0.5f;        // Minimum error distance 
		const float MAX_DIST = 0.5f;         // Maximum error distance

		float frontSensor;
		float rearSensor;
		float leftSensor;
		float rightSensor;

		//initialize sensor array
		float sensorArray[15]; 
		float* sensor;
		
		//boolean logic
		PROGRAM_RUNNING = true;
		bool followingLeft = false;
		bool followingRight = false;
		bool followingWall = false;
		bool centering = false;
		

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

		while (PROGRAM_RUNNING)
		{
			sensor = fillarr(sensorArray, 4); //create array of sonar reading - unused

			switch (state)
			{
			case WANDER:

				/*WANDER STATE - Explore map until something is found*/
				//printf("STATE: WANDER \n");

				//get current time
				currentTime = extApi_getTimeInMs();
				timeElapsed = currentTime - starttime;
				//printf("%i \n", timeElapsed);

				//init boolean
				followingLeft = false;
				followingRight = false;

				//If wall is detected infront of robot...
				if (getSensorReading(3) < SAFE_DIST)
				{
					motorControl(0, 0);

					starttime = extApi_getTimeInMs();

					int random = rand() % 2;

					if (random == 0 && getSensorReading(0) > SAFE_DIST) //turn left
					{
						/*motorControl(-1, 1);
						motorControl(speed, speed);*/
						followingLeft = true;
						state = AVOID;

					}
					if (random == 1 && getSensorReading(7) > SAFE_DIST) //turn right
					{
						/*motorControl(1, -1);
						motorControl(speed, speed);*/
						followingRight = true;
						state = AVOID;

					}
				}

				// If no wall is found - turn randomly
				if (timeElapsed > 10000 && getSensorReading(3) >= 5)
				{

					int random = rand() % 2;

					if (random == 0)
					{
						printf("Random turn (time) \n");
						motorControl(-1, 1);
						extApi_sleepMs(1000);
						motorControl(speed, speed);
						starttime = extApi_getTimeInMs(); //reset time
						break;
					}
					if (random == 1)
					{
						printf("Random turn (time) \n");
						motorControl(1, -1);
						extApi_sleepMs(1000);
						motorControl(speed, speed);
						starttime = extApi_getTimeInMs(); //reset time
						break;
					}
				}

				// If side sensors detect a wall...
				if (getSensorReading(0) < SAFE_DIST || getSensorReading(7) < SAFE_DIST)
				{
					state = FOLLOW;
					printf("STATE CHANGE: FOLLOW \n");
				}

			case FOLLOW:

				/*FOLLOW STATE - Wall is found, begin following wall*/
				//printf("STATE: FOLLOW \n");

				bool followingWall;

				//get current time
				currentTime = extApi_getTimeInMs();
				timeElapsed = currentTime - starttime;


				//Left sensor detected wall
				if (getSensorReading(0) < SAFE_DIST)
				{
					followingLeft = true;
					followingRight = false;
					followingWall = true;
					centering = false;

					//printf("WALL DETECTED LEFT \n");

					while (followingWall == true)
					{
						motorControl(speed, speed);

						frontSensor = (getSensorReading(3) + getSensorReading(4)) / 2;
						rearSensor = (getSensorReading(11) + getSensorReading(12)) / 2;

						//Logic for finding the center of the walls
						if (frontSensor - rearSensor > MIN_DIST&& frontSensor - rearSensor < MAX_DIST)
						{
							//followingLeft = false;
							followingRight = false;
							followingWall = false;
							centering = true;
							state = CENTER_WALL;
							printf("STATE CHANGE: CENTER_WALL \n");
						}

						if (getSensorReading(0) < STOPPING_DIST)
						{
							motorControl(1, -1); //Turn RIGHT away from wall
							starttime = extApi_getTimeInMs();

							printf("TOO CLOSE - Turning RIGHT \n");
						}

						if (getSensorReading(0) >= FOLLOW_DIST)
						{
							motorControl(-1, 1); //Turn LEFT towards wall
							starttime = extApi_getTimeInMs();

							printf("FAR OUT - Turning LEFT \n");
						}

						if (getSensorReading(3) <= SAFE_DIST)
						{
							followingWall = false;
							starttime = extApi_getTimeInMs();
							printf("STATE CHANGE: AVOID \n");
							state = AVOID;
						}

						if (/*followingLeft == false && */getSensorReading(3) > SAFE_DIST&& getSensorReading(0) > SAFE_DIST&& timeElapsed > 10000)
						{
							followingWall = false;
							starttime = extApi_getTimeInMs();
							printf("STATE CHANGE: WANDER \n");
							state = WANDER;
						}

					}
				}

				//Right sensor detected wall
				if (getSensorReading(7) < SAFE_DIST)
				{

					followingLeft = false;
					followingRight = true;
					followingWall = true;
					centering = false;

					//printf("WALL DETECTED RIGHT \n");

					while (followingWall == true)
					{
						motorControl(speed, speed);

						frontSensor = (getSensorReading(3) + getSensorReading(4)) / 2;
						rearSensor = (getSensorReading(11) + getSensorReading(12)) / 2;

						//Logic for finding the center of the walls
						if (frontSensor - rearSensor > MIN_DIST&& frontSensor - rearSensor < MAX_DIST)
						{
							followingLeft = false;
							//followingRight = false;
							//followingWall = false;
							centering = true;
							state = CENTER_WALL;
							printf("STATE CHANGE: CENTER_WALL \n");
						}

						if (getSensorReading(7) < STOPPING_DIST)
						{
							motorControl(-1, 1); //Turn LEFT away from wall
							starttime = extApi_getTimeInMs();

							printf("TOO CLOSE - Turning LEFT \n");
						}

						if (getSensorReading(7) >= FOLLOW_DIST)
						{
							motorControl(1, -1); //Turn RIGHT towards wall
							starttime = extApi_getTimeInMs();

							printf("FAR OUT - Turning RIGHT \n");
						}

						if (getSensorReading(3) <= SAFE_DIST)
						{
							followingWall = false;
							starttime = extApi_getTimeInMs();
							printf("STATE CHANGE: AVOID \n");
							state = AVOID;
						}

						if (/*followingRight == false && */getSensorReading(3) > SAFE_DIST&& getSensorReading(7) > SAFE_DIST&& timeElapsed > 10000)
						{
							followingWall = false;
							starttime = extApi_getTimeInMs();
							printf("STATE CHANGE: WANDER \n");
							state = WANDER;
						}

					}
				}

			case AVOID:

				/* FRONT SENSOR LOGIC */

				if (getSensorReading(3) < SAFE_DIST && getSensorReading(3) > STOPPING_DIST)
				{
					motorControl(0, 0);

					if (followingRight == true && getSensorReading(0) > SAFE_DIST) //turn left
					{
						motorControl(-1, 1);
						motorControl(speed, speed);
						printf("AVOIDNG LEFT \n");
						//state = FOLLOW;
					}
					if (followingLeft == true && getSensorReading(7) > SAFE_DIST) //turn right
					{
						motorControl(1, -1);
						motorControl(speed, speed);
						printf("AVOIDNG RIGHT \n");
						//state = FOLLOW;
					}
				}

				if (getSensorReading(3) < STOPPING_DIST)
				{
					followingLeft = false;
					followingRight = false;
					motorControl(-1, -1); //reverse
				}


				if (getSensorReading(3) > SAFE_DIST&& getSensorReading(0) < SAFE_DIST)
				{
					followingLeft = true;
					followingRight = false;
					state = FOLLOW;
					printf("STATE CHANGE: FOLLOW \n");
				}

				if (getSensorReading(3) > SAFE_DIST&& getSensorReading(2) < SAFE_DIST)
				{
					followingLeft = true;
					followingRight = false;
					state = FOLLOW;
					printf("STATE CHANGE: FOLLOW \n");
				}

			case CENTER_WALL:
				
				followingWall = false;

				while (centering && followingLeft)
				{
					printf("centering .... \n ");

					leftSensor = (getSensorReading(0) + getSensorReading(15)) / 2;
					rightSensor = (getSensorReading(7) + getSensorReading(8)) / 2;
					frontSensor = (getSensorReading(3) + getSensorReading(4)) / 2;
					rearSensor = (getSensorReading(11) + getSensorReading(12)) / 2;

					if (leftSensor - rightSensor > MIN_DIST&& leftSensor - rightSensor < MAX_DIST)
					{
						motorControl(speed, -speed);
					}
				}
				
				
				//if (followingLeft)
				//{
				//	leftSensor = getSensorReading(0);
				//	rightSensor = getSensorReading(7);

				//	motorControl(speed, -speed);

				//	if (frontSensor - rearSensor > MIN_DIST&& frontSensor - rearSensor < MAX_DIST)
				//	{
				//		printf("turning to find center of room \n");
				//		motorControl(speed, -speed);
				//	}

				//	if (leftSensor - rightSensor > MIN_DIST&& leftSensor - rightSensor < MAX_DIST)
				//	{
				//		followingLeft = false;
				//		followingRight = false;
				//		followingWall = false;

				//		motorControl(speed, -speed);
				//	}
				//	
				//}
				//
				//if (followingRight)
				//{
				//	leftSensor = getSensorReading(0);
				//	rightSensor = getSensorReading(7);

				//	if (frontSensor - rearSensor > MIN_DIST&& frontSensor - rearSensor < MAX_DIST)
				//	{
				//		printf("turning to find center of room \n");
				//		motorControl(-speed, speed);
				//	}

				//	if (leftSensor - rightSensor > MIN_DIST&& leftSensor - rightSensor < MAX_DIST)
				//	{
				//		//followingLeft = false;
				//		//followingRight = false;
				//		//followingWall = false;

				//		motorControl(-speed, speed);
				//	}
				//}

			case CENTER_ROOM:

				printf("Center room \n");

			}
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
