#ifndef CVTMODEL_H_INCLUDED
#define CVTMODEL_H_INCLUDED

struct encoder { int channelA_pin ; int channelB_pin ; int channelZ_pin ; int PPR ; float radius ; float dist};
enum { ENCODER_X , ENCODER_Y };
float k[2];

namespace cvtmodel {
    float degreeToRadian(float);
    struct motorOutput getOutput_d (float);
    struct motorOutput getOutput_j (float,float);
}
#endif
