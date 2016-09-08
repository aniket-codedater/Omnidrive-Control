#include <iostream>
#include <math.h>
#include <stdlib.h>
#include "cvtmodel.h"
#include "drivemtr.h"
//#include <Wiring.h>

using namespace std;
using namespace drivemtr;
using namespace cvtmodel;

float junction[7][2] {
    {0.0,0.0},
    {0.0,200.0},
    {0.0,400.0},
    {0.0,600.0},
    {0.0,800.0},
    {0.0,1200.0},
    {0.0,1500.0},
};

float heading = 0.0;

struct motorOutput{
    int mtr1_rpm;
    int mtr2_rpm;
    int mtr3_rpm;
    int mtr4_rpm;
} outputToBeGiven;

///struct motorOutput *outputToBeGiven = NULL;

//float desiredHeading = 25.0;
//float curHeading = 0.0;
//float headingError = 0.0;
//int cur_junc = 0;

struct position {
    float positionX;
    float positionY;
};

struct position initialpos = {0.0,0.0};
struct encoder encoderX;
///encoderX.channelA_pin = pin no;
///encoderX.channelB_pin = pin no;
///encoderX.PPR = some value;
///encoderX.radius = radius of encoder;
struct encoder encoderY;
///encoderY.channelA_pin = pin no;
///encoderY.channelB_pin = pin no;
///encoderY.PPR = some value;
///encoderY.radius = radius of encoder;


int main() {
   	rpiPort = serialOpen("/dev/ttyS0",38400);
    ///curHeading = some input feedback
    ///calculate the output to be given
    drvbot(cur_junc);
    ///headingError = curHeading - desiredHeading;
    ///outputToBeGiven = getOutput(headingError);
    ///give output
    ///driveMotor(outputToBeGiven);
    return 0;
}
