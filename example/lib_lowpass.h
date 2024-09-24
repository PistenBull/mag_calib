#ifndef LIB_LOWPASS_H
#define LIB_LOWPASS_H

#include "stdlib.h"

#ifndef M_2PI
#define M_2PI	6.28318530717958623200 /* 2 pi */
#endif


float lowpass(float sample, float cutoff, double dt);
float compute_alpha(float cutoff, double dt);
float constrain_float(double val, float low, float high);
#endif
