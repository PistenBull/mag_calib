#ifndef LIB_QUATERNION_H
#define LIB_QUATERNION_H

#include "Arduino.h"
#include "data_structures.h"
#include "helpers.h"

#define beta_AHRS							            0.3
#define beta_IMU										      0.5

//#define USE_FILTERED_DATA
#define USE_SCALED_DATA

#define DEBUG_IMU_FILTER
#define DEBUG_AHRS_FILTER
//#define DEBUG_COMPASS_VECTOR_DEVIATION

#define SPB_DEVIATION									    0.2068
#define ALLOWABLE_DEVIATION								30.0 // 30%

void AHRS_quaternion_update(AHRS_sensor * ahrs_sensor);
void IMU_quaternion_update(AHRS_sensor* ahrs_sensor);
void AHRS_filter_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void IMU_filter_update(float ax, float ay, float az, float gx, float gy, float gz);
float invSqrt(float x);

float get_roll(void);
float get_pitch(void);
float get_yaw(void);
float get_heading(void);
Quaternion get_quaternion(void);
#endif
