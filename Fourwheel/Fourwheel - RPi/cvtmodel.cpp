#include "cvtmodel.h"
#include <stdlib.h>
#include <math.h>

using namespace cvtmodel;

float radius = 90.0;
int maxrpm = 200;

k[ENCODER_X] = (float)(2 * M_PI * encoderX.radius / encoderX.PPR);
k[ENCODER_Y] = (float)(2 * M_PI * encoderY.radius / encoderY.PPR);


float degreeToRadian(float headingError) {
    return (headingError * M_PI / 180);
}

float sigmoid_t (float err) {
    return (1/(1+exp((-1)*err*0.05)));
}

float *cvtArenaToBot(float pos_x,float pos_y) {
    float pos_bot[2];
    pos_bot[0] = (pos_x * sin(theta)) - (pos_y * cos(theta));
    pos_bot[1] = (pos_x * cos(theta)) - (pos_y * sin(theta));
    pos_x = pos_bot[0];
    pos_y = pos_bot[1];
    return pos_bot;
}

struct motorOutput cvtHeadingToRpm(float headingError) {
    struct motorOutput rpm;
    if(headingError > 0) {
        float headingErrorRadian = degreeToRadian(headingError);
        rpm.mtr1_rpm = radius * headingErrorRadian;
        rpm.mtr2_rpm = radius * headingErrorRadian;
        rpm.mtr3_rpm = radius * headingErrorRadian;
        rpm.mtr4_rpm = radius * headingErrorRadian;
    }
    return rpm;
}

struct motorOutput getOutput_d (float headingError) {
    struct motorOutput temp;
    ///converting heading in terms of motor rpm output
    temp = cvtHeadingToRpm(headingError);
    return temp;
}

struct motorOutput getOutput_j (float pos_x,float pos_y) {
    bool pos_achieved = false;
    while(!pos_achieved) {

    }
}

struct motorOutput calcRpm(float pos_x_err,float pos_y_err) {
    float pos_x_arena = pos_x_err;
    float pos_y_arena = pos_y_err;
    float pos_bot[2];
    pos_bot = cvtArenaToBot(pos_x_arena,pos_y_arena);
    struct motorOutput temp_state;
    temp_state.mtr1_rpm = (maxrpm * sigmoid_t(pos_bot[0]));
    temp_state.mtr3_rpm = (maxrpm * sigmoid_t(pos_bot[0]));
    temp_state.mtr2_rpm = (maxrpm * sigmoid_t(pos_bot[1]));
    temp_state.mtr4_rpm = (maxrpm * sigmoid_t(pos_bot[1]));
    return temp_state;
}

void encoderInterrupt_x() {
    float curhead = getHeading();
    if(digitalRead(encoderX.channelB_pin)) {
        encoderX.dist += k[ENCODER_X] * cos(curhead);
        encoderX.dist += k[ENCODER_X] * sin(curhead);
    }
    else {
        encoderX.dist -= k[ENCODER_X] * cos(curhead);
        encoderX.dist -= k[ENCODER_X] * sin(curhead);
    }
}

void encoderInterrupt_y() {
    float curhead = getHeading();
    if(digitalRead(encoderY.channelB_pin)) {
        encoderX.dist += k[ENCODER_Y] * cos(curhead);
        encoderX.dist += k[ENCODER_Y] * sin(curhead);
    }
    else {
        encoderX.dist -= k[ENCODER_Y] * cos(curhead);
        encoderX.dist -= k[ENCODER_Y] * sin(curhead);
    }
}
