#include "drivemtr.h"
#include "cvtmodel.h"
#include <stdlib.h>

using namespace drivemtr;

void driveMotor(struct motorOutput outputToBeGiven) {
    ///transfer data to lower controller
    serialPutchar(rpiPort,encodeByte(outputToBeGiven.mtr1_rpm);
    serialPutchar(rpiPort,encodeByte(outputToBeGiven.mtr2_rpm);
    serialPutchar(rpiPort,encodeByte(outputToBeGiven.mtr3_rpm);
    serialPutchar(rpiPort,encodeByte(outputToBeGiven.mtr4_rpm);
}

void gotopos(float pos_x,float pos_y) {
    struct motorOutput temp;
    temp = getOutput_j(pos_x,pos_y);
}

void gotonxt(void) {
    float pos_x_err = 0.0;
    float pos_y_err = 200.0;
    while (pos_x_err != 0.0 && pos_y_err != 0.0) {
        drvmtr_j(pos_x_err,pos_y_err);
        pos_x_err -= encoderX.dist();
        pos_y_err -= encoderY.dist();
    }
}

void drvmtr_j(float pos_x_err,float pos_y_err) {
    ///return rpm according to sigmoid function
    struct motorOutput temp_state;
    temp_state = calcRpm(pos_x_err,pos_y_err);
    driveMotor(temp_state);
}
