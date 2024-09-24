#ifndef LIB_COMPASS_H
#define LIB_COMPASS_H

#include <math.h>
#include "Arduino.h"
#include "data_structures.h"
#include "lib_quaternion.h"
#include "lib_ahrs.h"
#include <EEPROM.h>

#define DEBUG_COMPASS
//#define DEBUG_IMU_READ

// Данные коэффициенты настраиваются эксперементальным путём
#define ANGLE 3.0f
#define D 0.07f

/* Function that starts the calibration process */
void compass_calibration(Raw_data* raw_data, AHRS_sensor* ahrs_sensor);

void calculate_segment_centers(Raw_data* raw_data, Segment_centers* seg_centers);
void calculate_vector_matrix(Segment_centers* seg_centers, float vector_matrix[3][3]);
void calculate_inverse_matrix(float vector_matrix[3][3], AHRS_sensor* ahrs_sensor);
void calculate_bias_matrix(Segment_centers* seg_centers, AHRS_sensor* ahrs_sensor);
void calculate_transformation_matrix(Raw_data* raw_data, Segment_centers* seg_centers, AHRS_sensor* ahrs_sensor);
void matrix_multiplication(float left_matrix[3][3], float right_matrix[3], float result[3]);

// Выделить в отдельный файл (дополнительный к файлу калибровки)
void calculate_compass_vector(Raw_data* raw_data, AHRS_sensor* ahrs_sensor);
void reading_raw_compass_for_calibration(Raw_data* raw_data, AHRS_sensor* ahrs_sensor);

/* Function for calculating calibrated magnetometer data */
void get_calibrated_compass(float raw_array[3], float res_array[3], float trans_matrix[3][3], float bias_matrix[3]);
#endif
