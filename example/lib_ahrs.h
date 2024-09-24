#ifndef LIB_AHRS_H
#define LIB_AHRS_H

// Подчистить эти подключения
#include "Arduino.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <EEPROM.h>
#include "helpers.h"
#include "data_structures.h"
#include "lib_compass.h"
#include "lib_lowpass.h"
#include "lib_quaternion.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "HMC5883L.h"

#define PASS_BUFFER_SIZE					2000
#define CALIB_BUFFER_SIZE					5000

// Сделать массивы для различных настроек чувствительности сенсоров
#define ACCEL_IN_G							256
#define GYRO_IN_MS							14.375
#define COMPASS_IN_UT						0.15

// TODO:  
#define MPU_DEFAULT_ACCEL_LPF_CUTOFF		10000
#define MPU_DEFAULT_GYRO_LPF_CUTOFF			150000 
#define MPU_DEFAULT_COMPASS_LPF_CUTOFF		10000

//#define DEBUG_AHRS

void get_raw_accel(AHRS_sensor * ahrs_sensor);
void get_raw_gyro(AHRS_sensor * ahrs_sensor);
void get_raw_compass(AHRS_sensor * ahrs_sensor);
void get_raw_AHRS(AHRS_sensor * ahrs_sensor);
void get_scaled_AHRS(AHRS_sensor * ahrs_sensor);
void get_filtered_AHRS(AHRS_sensor * ahrs_sensor);

void IMU_calibration(AHRS_sensor* ahrs_sensor);
#endif
