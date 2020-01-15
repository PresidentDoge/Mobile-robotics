#pragma once
#include "extApi.h"


int clientID;
int beaconHandle;
int distanceHandle;
int leftmotorHandle;
int rightmotorHandle;
int senorHandle[15];

simxFloat beaconPosition[3];
simxUChar detectionState[1];
simxFloat detectedPoint[3];
simxFloat distance[3];

// Control variables
bool PROGRAM_RUNNING;
bool followingLeft;
bool followingRight;
int currentTime;

enum FSM{WANDER, AVOID, FOLLOW, CENTER_WALL, CENTER_ROOM};
enum followState{LEFT_CLOSE, LEFT_FAR, RIGHT_CLOSE, RIGHT_FAR, L_R_CLOSE, FOLLOW_WALL, L_R_FAR, IDLE};
followState edgeFollowState;
FSM state;

void motorControl(float leftMotor, float rightMotor);
float getSensorReading(int sensor);
float * fillarr(float arr[], int length);
void wait(int time);
float getMean(float sensor1, float sensor2);
float getError(float sensor1, float sensor2);

simxFloat findBeacon();