#pragma once
#include "extApi.h"
#include <vector>

int clientID;
int beaconHandle;
int distanceHandle;
int leftmotorHandle;
int rightmotorHandle;
int senorHandle[15];

//initialize consts speed/distances
const double SPEED = 1.0f;
const double STOPPING_DIST = 0.25f;
const double SAFE_DIST = 0.5f;
const double FOLLOW_DIST = 0.4f;
const double MAX_DIST = 5.0f;

//PID
double error;

double pGain = 1;
double iGain = 0.0001;
double dGain = 90;

double pTerm;
double iTerm;
double dTerm;

simxFloat beaconPosition[3];
simxUChar detectionState[1];
simxFloat detectedPoint[3];
simxFloat distance[3];

// Control variables
bool PROGRAM_RUNNING;
bool followingLeft;
bool followingRight;
int currentTime;

enum FSM{HOME, FOLLOW, FIND_HOME, AVOID};
FSM state;

void motorControl(float leftMotor, float rightMotor);
void wait(int time);
void createObjectHandles();

float getSensorReading(int sensor);
float* fillarr(float arr[], int length);

double frontSensors();
double leftSensors();
double rightSensors();
double rearSensors();

simxFloat findBeacon();