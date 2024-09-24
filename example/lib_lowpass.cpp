#include "lib_lowpass.h"


float lowpass(float sample, float cutoff, double dt) {
    static float output = 0;
    float alpha = compute_alpha(cutoff, dt);

    output += (sample - output) * alpha;
    return output;
}


float compute_alpha(float cutoff, double dt) {
    float rc = 1.0f / (M_2PI * cutoff);
    float alpha = constrain_float(dt / (dt + rc), 0.0f, 1.0f);
    return alpha;
}


float constrain_float(double val, float low, float high) {
    if (val < low) return low;
    if (val > high) return high;
    return val;
}
