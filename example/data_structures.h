#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include "stdint.h"
#include <stdbool.h>

#define OFFSETS_ADDR  666


typedef struct  {
    float w;
    float x, y, z;
} Quaternion;


typedef struct {
	float accel_offsets[3];
	float gyro_offsets[3];
  float compass_offsets[3];

  float compass_vector;

	float bias_matrix[3];
	float transformation_matrix[3][3];
} Sensor_offsets;


typedef struct {
	float raw_accel[3];
	float raw_gyro[3];
	float raw_compass[3];

	float scaled_accel[3];
	float scaled_gyro[3];
	float scaled_compass[3];

	float filtered_accel[3];
	float filtered_gyro[3];
	float filtered_compass[3];

	Sensor_offsets offsets;
} AHRS_sensor;


typedef struct {
    float X_plus_0[3];      bool x_plus_0;
    float X_plus_180[3];    bool x_plus_180;

    float X_minus_0[3];     bool x_minus_0;
    float X_minus_180[3];   bool x_minus_180;

    float Y_plus_0[3];      bool y_plus_0;
    float Y_plus_180[3];    bool y_plus_180;

    float Y_minus_0[3];     bool y_minus_0;
    float Y_minus_180[3];   bool y_minus_180;

    float Z_plus_0[3];      bool z_plus_0;
    float Z_plus_180[3];    bool z_plus_180;

    float Z_minus_0[3];     bool z_minus_0;
    float Z_minus_180[3];   bool z_minus_180;
} Raw_data;


typedef struct {
	float X_plus_center[3];
	float X_minus_center[3];
	float Y_plus_center[3];
	float Y_minus_center[3];
	float Z_plus_center[3];
	float Z_minus_center[3];
} Segment_centers;
#endif
