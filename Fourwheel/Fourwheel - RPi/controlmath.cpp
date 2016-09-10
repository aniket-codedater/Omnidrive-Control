#include "controlmath.h"
#include <math.h>

struct uniCycleState velocityLimiter(struct uniCycleState state) {
	float max,factor = 1,dividend = 1;
	(fabs(state.vx) >= fabs(state.vy)) ? max = fabs(state.vx) : max = fabs(state.vy);
	dividend = max;
	if(max > maxRPM) {
		while(dividend > maxRPM) {
			dividend = max / factor;
			factor += 0.5;
		}
		factor -= 0.5;
		state.vx /= factor;	
		state.vy /= factor;	
	}
	return state;
}

struct omniDriveState rpmLimiter(struct omniDriveState state) {
	float max,factor = 1,dividend = 1;
	(fabs(state.aRPM) >= fabs(state.bRPM)) ? max = fabs(state.aRPM) : max = fabs(state.bRPM);
	(max >= fabs(state.cRPM)) ? max = max : max = fabs(state.cRPM);
	(max >= fabs(state.dRPM)) ? max = max : max = fabs(state.dRPM);
	dividend = max;
	if(max > maxRPM) {
		while(dividend > maxRPM) {
			dividend = max / factor;
			factor += 0.5;
		}
		factor -= 0.5;
		state.aRPM /= factor;	
		state.bRPM /= factor;	
		state.cRPM /= factor;	
		state.dRPM /= factor;	
	}
	return state;
}


float sigmoid(float x) {
	return ((1/(1+exp(-x))) - 0.5) * 2;
}

float degreeToRadian(float degree) {
    return degree * PI / 180;
}

float radianToDegree(float radian) {
    return radian * 180 / PI;
}

float normalizeAngle(float degree) {
    return radianToDegree(atan2(sin(degreeToRadian(degree)), cos(degreeToRadian(degree))));
}

float getDistanceBetweenPoints(struct point pointA, struct point pointB) {
	return sqrt(pow((pointB.x - pointA.x), 2) + pow((pointB.y - pointA.y), 2));
}

struct point rotationalTransform(struct point point_, float theta) {
	return rotationalTransform(point_.x, point_.y, theta);
}

struct point rotationalTransform(float x, float y, float theta){
    struct point pos;
    theta = degreeToRadian(theta);
    pos.x = x * cos(theta) - y * sin(theta);
    pos.y = y * cos(theta) + x * sin(theta);
    return pos;
}

struct omniDriveState transformUniToOmni(struct uniCycleState unistate, float alpha) {	
    struct omniDriveState transformedState;
    float wR = unistate.w * botRadius;
    alpha = degreeToRadian(alpha);
    unistate = velocityLimiter(unistate);		//Limit velocity
    float v = sqrt(unistate.vx * unistate.vx + unistate.vy * unistate.vy);
    float phi = atan2(unistate.vy, unistate.vx);
    
    float cosphi = cos(phi);
    float sinphi = sin(phi);
   
    transformedState.aRPM = ((v * ((cos(alpha + theta_A) * cosphi) + (sin(alpha + theta_A) * sinphi))) + wR ) * 60.0 / wheelCircumference;
    transformedState.bRPM = ((v * ((cos(alpha + theta_B) * cosphi) + (sin(alpha + theta_B) * sinphi))) + wR ) * 60.0 / wheelCircumference;
    transformedState.cRPM = ((v * ((cos(alpha + theta_C) * cosphi) + (sin(alpha + theta_C) * sinphi))) + wR ) * 60.0 / wheelCircumference;
    transformedState.dRPM = ((v * ((cos(alpha + theta_D) * cosphi) + (sin(alpha + theta_D) * sinphi))) + wR ) * 60.0 / wheelCircumference;
    return rpmLimiter(transformedState);		//Limit RPM
}

