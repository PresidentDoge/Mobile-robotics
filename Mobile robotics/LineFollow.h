#pragma once
#include "extApi.h"
#include <vector>

int clientID;
int leftmotorHandle;
int rightmotorHandle;
int senorHandle[15];

bool PROGRAM_RUNNING;

simxUChar detectionState[1];
simxFloat detectedPoint[3];

// Control variables
bool followingLeft;
bool followingRight;
int currentTime;

enum FSM{WANDER, FOLLOW, AVOID};
enum followState{LEFT_CLOSE, LEFT_FAR, RIGHT_CLOSE, RIGHT_FAR, L_R_CLOSE, FOLLOW_WALL, L_R_FAR, IDLE};
followState edgeFollowState;
FSM state;

void motorControl(float leftMotor, float rightMotor);
float getSensorReading(int sensor);
float * fillarr(float arr[], int length);
void wait(int time);

 