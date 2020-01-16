#pragma once
#include "extApi.h"
#include <vector>

int clientID;
int leftmotorHandle;
int rightmotorHandle;
int senorHandle[4];

bool PROGRAM_RUNNING;

simxUChar detectionState[1];
simxFloat detectedPoint[3];

// Control variables
int currentTime;

void motorControl(float leftMotor, float rightMotor);
float getSensorReading(int sensor);
void wait(int time);

 