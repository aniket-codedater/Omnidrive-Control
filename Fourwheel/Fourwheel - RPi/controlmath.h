#ifndef controlmath
#define controlmath

#define PI 3.1415926
#include "driveConfig.h"

struct point { float x; float y; };
struct uniCycleState {float vx; float vy; float w;};
struct omniDriveState {int aRPM; int bRPM; int cRPM; int dRPM};

float sigmoid(float x);
float degreeToRadian(float degree);
float radianToDegree(float radian);
float normalizeAngle(float degree);
float getDistanceBetweenPoints(struct point pointA, struct point pointB);
struct point rotationalTransform(struct point point_, float theta);
struct point rotationalTransform(float x_, float y_, float theta);
struct omniDriveState transformUniToOmni(struct uniCycleState unistate, float alpha); //alpha is the angular offset given to the robot 
																					  //heading from the zero reference

#endif
