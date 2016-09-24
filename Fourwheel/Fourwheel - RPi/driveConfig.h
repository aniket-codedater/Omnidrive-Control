#ifndef driveConfig
#define driveConfig

//Bot specifications
#define wheelRadius 5.0
#define wheelCircumference (2 * PI * wheelRadius)

#define botRadius  46.2
#define theta_A degreeToRadian(45.0)
#define theta_B degreeToRadian(135.0)
#define theta_C degreeToRadian(225.0)
#define theta_D degreeToRadian(315.0)

//Motion constraints specifications
#define maxRPM			252	//in rpm
#define maxVelocity		300 //in cm/s

//Frequency specifications
#define PIDfrequency	40

#endif
