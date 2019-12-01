//
// simRemoteApi.start(19999)
//

#include <stdio.h>
#include <stdlib.h>
#include "LineFollow.h"
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
		srand(time(NULL));
		const float speed = 1;
		const float STOPPING_DIST = 0.25;
		const float SAFE_DIST = 0.5;
		const float FOLLOW_DIST = 0.45;
		float sensorArray[4];
		float *sensor;
		PROGRAM_RUNNING = true;
		bool followingLeft = false;
		bool followingRight = false;
		int timeElapsed = 0;
		simxInt starttime = extApi_getTimeInMs();

		/* Create handles */

		//motor handles
		int leftMotor = simxGetObjectHandle(clientID, "Pioneer_p3dx_leftMotor", &leftmotorHandle, simx_opmode_oneshot_wait);
		int rightMotor = simxGetObjectHandle(clientID, "Pioneer_p3dx_rightMotor", &rightmotorHandle, simx_opmode_oneshot_wait);
		//sensor handles
		int frontSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor5", &senorHandle[0], simx_opmode_blocking);
		int leftSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor1", &senorHandle[1], simx_opmode_blocking);
		int rightSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor8", &senorHandle[2], simx_opmode_blocking);
		int backSensor = simxGetObjectHandle(clientID, "Pioneer_p3dx_ultrasonicSensor12", &senorHandle[3], simx_opmode_blocking);

		//Initialize states
		FSM state = WANDER;
		//followState edgeFollowState = IDLE;

		// Start Motors
		motorControl(speed, speed);

		/* Main loop */

		while(PROGRAM_RUNNING)
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
				if (getSensorReading(0) < SAFE_DIST)
				{
					motorControl(0, 0);

					starttime = extApi_getTimeInMs();

					int random = rand() % 2;

					if (random == 0 && getSensorReading(1) > SAFE_DIST) //turn left
					{
						/*motorControl(-1, 1);
						motorControl(speed, speed);*/
						followingLeft = true;
						state = AVOID;

					}
					if (random == 1 && getSensorReading(2) > SAFE_DIST) //turn right
					{
						/*motorControl(1, -1);
						motorControl(speed, speed);*/
						followingRight = true;
						state = AVOID;

					}
				}
				
				// If no wall is found - turn randomly
				if (timeElapsed > 10000 && getSensorReading(0) >= 5)
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
				if (getSensorReading(1) < SAFE_DIST || getSensorReading(2) < SAFE_DIST)
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
					if (getSensorReading(1) < SAFE_DIST)
					{
						followingLeft = true;
						followingRight = false;
						followingWall = true;

						//printf("WALL DETECTED LEFT \n");

						while(followingWall == true)
						{
							motorControl(speed, speed);

							if (getSensorReading(1) < STOPPING_DIST)
							{
								motorControl(1, -1); //Turn RIGHT away from wall
								starttime = extApi_getTimeInMs();

								printf("TOO CLOSE - Turning RIGHT \n");
							}

							if (getSensorReading(1) >= FOLLOW_DIST)
							{
								motorControl(-1, 1); //Turn LEFT towards wall
								starttime = extApi_getTimeInMs();

								printf("FAR OUT - Turning LEFT \n");
							}

							if (getSensorReading(0) <= SAFE_DIST)
							{
								followingWall = false;
								starttime = extApi_getTimeInMs();
								printf("STATE CHANGE: AVOID \n");
								state = AVOID;
							}

							if (/*followingLeft == false && */getSensorReading(0) > SAFE_DIST && getSensorReading(1) > SAFE_DIST && timeElapsed > 10000)
							{
								followingWall = false;
								starttime = extApi_getTimeInMs();
								printf("STATE CHANGE: WANDER \n");
								state = WANDER;
							}

						}
					}

					//Right sensor detected wall
					if (getSensorReading(2) < SAFE_DIST)
					{

						followingLeft = false;
						followingRight = true;
						followingWall = true;

						//printf("WALL DETECTED RIGHT \n");

						while(followingWall == true)
						{

							motorControl(speed, speed);

							if (getSensorReading(2) < STOPPING_DIST)
							{
								motorControl(-1, 1); //Turn LEFT away from wall
								starttime = extApi_getTimeInMs();

								printf("TOO CLOSE - Turning LEFT \n");
							}

							if (getSensorReading(2) >= FOLLOW_DIST)
							{
								motorControl(1, -1); //Turn RIGHT towards wall
								starttime = extApi_getTimeInMs();

								printf("FAR OUT - Turning RIGHT \n");
							}
						
							if (getSensorReading(0) <= SAFE_DIST)
							{
								followingWall = false;
								starttime = extApi_getTimeInMs();
								printf("STATE CHANGE: AVOID \n");
								state = AVOID;
							}

							if (/*followingRight == false && */getSensorReading(0) > SAFE_DIST && getSensorReading(2) > SAFE_DIST && timeElapsed > 10000)
							{
								followingWall = false;
								starttime = extApi_getTimeInMs();
								printf("STATE CHANGE: WANDER \n");
								state = WANDER;
							}

						}
					}

			case AVOID:

				//printf("STATE: AVOID \n");

				if (getSensorReading(0) < SAFE_DIST && getSensorReading(0) > STOPPING_DIST)
				{
					motorControl(0, 0);
					
					if (followingRight == true && getSensorReading(1) > SAFE_DIST) //turn left
					{
						motorControl(-1, 1);
						motorControl(speed, speed);
						printf("AVOIDNG LEFT \n");
						//state = FOLLOW;
					}
					if (followingLeft == true && getSensorReading(2) > SAFE_DIST) //turn right
					{
						motorControl(1, -1);
						motorControl(speed, speed);
						printf("AVOIDNG RIGHT \n");
						//state = FOLLOW;
					}
				}

				if (getSensorReading(0) < STOPPING_DIST)
				{
					followingLeft = false;
					followingRight = false;
					motorControl(-1, -1); //reverse
				}

				if (getSensorReading(0) > SAFE_DIST && getSensorReading(1) < SAFE_DIST)
				{
					followingLeft = true;
					followingRight = false;
					state = FOLLOW;
					printf("STATE CHANGE: FOLLOW \n");
				}

				if (getSensorReading(0) > SAFE_DIST && getSensorReading(2) < SAFE_DIST)
				{
					followingLeft = true;
					followingRight = false;
					state = FOLLOW;
					printf("STATE CHANGE: FOLLOW \n");
				}
			
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
	float jLength = (unsigned int)detectionState[0] == 1 ? std::sqrt((detectedPoint[0] * detectedPoint[0]) + (detectedPoint[1] * detectedPoint[1]) + (detectedPoint[2] * detectedPoint[2])) : 5.0f;
	return jLength;
}



//This function exists incase I want to poll all 16 sonar sensors at once and create an array of readings
//Using getSonarReading() for now
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
