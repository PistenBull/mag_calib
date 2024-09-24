#include "lib_ahrs.h"

#define DEBUG_AHRS_CALIBRATION

extern ADXL345 accel;
extern ITG3200 gyro;
extern HMC5883L mag;

static double deltaT = 0;
static uint32_t new_time = 0, old_time = 0;


void get_raw_accel(AHRS_sensor * ahrs_sensor) {
    int16_t ax, ay, az;
    accel.getAcceleration(&ax, &ay, &az);

    ahrs_sensor->raw_accel[0] = (float)ax;
    ahrs_sensor->raw_accel[1] = (float)ay;
    ahrs_sensor->raw_accel[2] = (float)az;

#ifdef DEBUG_AHRS
    Serial.print("raw accel: "); Serial.print(String(ahrs_sensor->raw_accel[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->raw_accel[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->raw_accel[2], 4));
#endif
}


void get_raw_gyro(AHRS_sensor * ahrs_sensor) {
    int16_t gx, gy, gz;
    gyro.getRotation(&gx, &gy, &gz);

    ahrs_sensor->raw_gyro[0] = (float)gx;
    ahrs_sensor->raw_gyro[1] = (float)gy;
    ahrs_sensor->raw_gyro[2] = (float)gz;

#ifdef DEBUG_AHRS
    Serial.print("raw gyro: "); Serial.print(String(ahrs_sensor->raw_gyro[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->raw_gyro[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->raw_gyro[2], 4));
#endif
}


void get_raw_compass(AHRS_sensor * ahrs_sensor) {
    int16_t mx, my, mz;
    mag.getHeading(&mx, &my, &mz);

    ahrs_sensor->raw_compass[0] = (float)mx;
    ahrs_sensor->raw_compass[1] = (float)my;
    ahrs_sensor->raw_compass[2] = (float)mz;

#ifdef DEBUG_AHRS
    Serial.print("raw compass: "); Serial.print(String(ahrs_sensor->raw_compass[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->raw_compass[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->raw_compass[2], 4));
#endif
}


void get_raw_AHRS(AHRS_sensor * ahrs_sensor) {
    get_raw_accel(ahrs_sensor);
    get_raw_gyro(ahrs_sensor);
    get_raw_compass(ahrs_sensor);
}


void get_scaled_AHRS(AHRS_sensor * ahrs_sensor) {
    get_raw_AHRS(ahrs_sensor);

    get_calibrated_compass(ahrs_sensor->raw_compass, ahrs_sensor->scaled_compass, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);

    // No transfer to uT has been performed, is this necessary?
    ahrs_sensor->scaled_compass[0] = ahrs_sensor->scaled_compass[0];
    ahrs_sensor->scaled_compass[1] = ahrs_sensor->scaled_compass[1];
    ahrs_sensor->scaled_compass[2] = ahrs_sensor->scaled_compass[2];

    ahrs_sensor->scaled_gyro[0] = to_radians((ahrs_sensor->raw_gyro[0] - ahrs_sensor->offsets.gyro_offsets[0]) / GYRO_IN_MS);
    ahrs_sensor->scaled_gyro[1] = to_radians((ahrs_sensor->raw_gyro[1] - ahrs_sensor->offsets.gyro_offsets[1]) / GYRO_IN_MS);
    ahrs_sensor->scaled_gyro[2] = to_radians((ahrs_sensor->raw_gyro[2] - ahrs_sensor->offsets.gyro_offsets[2]) / GYRO_IN_MS);

    ahrs_sensor->scaled_accel[0] = (ahrs_sensor->raw_accel[0] - ahrs_sensor->offsets.accel_offsets[0]) / ACCEL_IN_G;
    ahrs_sensor->scaled_accel[1] = (ahrs_sensor->raw_accel[1] - ahrs_sensor->offsets.accel_offsets[1]) / ACCEL_IN_G;
    ahrs_sensor->scaled_accel[2] = (ahrs_sensor->raw_accel[2] - ahrs_sensor->offsets.accel_offsets[2]) / ACCEL_IN_G;

#ifdef DEBUG_AHRS
    Serial.print("scaled accel: "); Serial.print(String(ahrs_sensor->scaled_accel[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->scaled_accel[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->scaled_accel[2], 4));
    Serial.print("scaled gyro: "); Serial.print(String(ahrs_sensor->scaled_gyro[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->scaled_gyro[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->scaled_gyro[2], 4));
    Serial.print("scaled compass: "); Serial.print(String(ahrs_sensor->scaled_compass[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->scaled_compass[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->scaled_compass[2], 4));
    Serial.println();
#endif
}


void get_filtered_AHRS(AHRS_sensor * ahrs_sensor) {
  get_scaled_AHRS(ahrs_sensor);

	new_time = micros();
	deltaT = new_time - old_time;
	old_time = new_time;
	deltaT = fabs(deltaT * 0.001 * 0.001);

	ahrs_sensor->filtered_compass[0] = lowpass(ahrs_sensor->scaled_compass[0], MPU_DEFAULT_COMPASS_LPF_CUTOFF, deltaT);
	ahrs_sensor->filtered_compass[1] = lowpass(ahrs_sensor->scaled_compass[1], MPU_DEFAULT_COMPASS_LPF_CUTOFF, deltaT);
	ahrs_sensor->filtered_compass[2] = lowpass(ahrs_sensor->scaled_compass[2], MPU_DEFAULT_COMPASS_LPF_CUTOFF, deltaT);

	ahrs_sensor->filtered_gyro[0] = lowpass(ahrs_sensor->scaled_gyro[0], MPU_DEFAULT_GYRO_LPF_CUTOFF, deltaT);
	ahrs_sensor->filtered_gyro[1] = lowpass(ahrs_sensor->scaled_gyro[1], MPU_DEFAULT_GYRO_LPF_CUTOFF, deltaT);
	ahrs_sensor->filtered_gyro[2] = lowpass(ahrs_sensor->scaled_gyro[2], MPU_DEFAULT_GYRO_LPF_CUTOFF, deltaT);

	ahrs_sensor->filtered_accel[0] = lowpass(ahrs_sensor->scaled_accel[0], MPU_DEFAULT_ACCEL_LPF_CUTOFF, deltaT);
	ahrs_sensor->filtered_accel[1] = lowpass(ahrs_sensor->scaled_accel[1], MPU_DEFAULT_ACCEL_LPF_CUTOFF, deltaT);
	ahrs_sensor->filtered_accel[2] = lowpass(ahrs_sensor->scaled_accel[2], MPU_DEFAULT_ACCEL_LPF_CUTOFF, deltaT);
}


void IMU_calibration(AHRS_sensor * ahrs_sensor) {
    for (int i = 0; i < 3; ++i) {
        ahrs_sensor->offsets.accel_offsets[i] = 0;
        ahrs_sensor->offsets.gyro_offsets[i] = 0;
        ahrs_sensor->offsets.compass_offsets[i] = 0;
    }

    for (int i = 0; i < PASS_BUFFER_SIZE + CALIB_BUFFER_SIZE; ++i) {
        get_raw_AHRS(ahrs_sensor);
        get_calibrated_compass(ahrs_sensor->raw_compass, ahrs_sensor->scaled_compass, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);

        // No transfer to uT has been performed, is this necessary?
        ahrs_sensor->scaled_compass[0] = ahrs_sensor->scaled_compass[0];
        ahrs_sensor->scaled_compass[1] = ahrs_sensor->scaled_compass[1];
        ahrs_sensor->scaled_compass[2] = ahrs_sensor->scaled_compass[2];

        if (i >= PASS_BUFFER_SIZE)
            for (int j = 0; j < 3; ++j) {
                ahrs_sensor->offsets.accel_offsets[j] += ahrs_sensor->raw_accel[j];
                ahrs_sensor->offsets.gyro_offsets[j] += ahrs_sensor->raw_gyro[j];
                ahrs_sensor->offsets.compass_offsets[j] += ahrs_sensor->scaled_compass[j];


                if (j == 2) ahrs_sensor->offsets.accel_offsets[j] += ACCEL_IN_G;
            }
    }

    for (int i = 0; i < 3; ++i) {
        ahrs_sensor->offsets.accel_offsets[i] /= CALIB_BUFFER_SIZE;
        ahrs_sensor->offsets.gyro_offsets[i] /= CALIB_BUFFER_SIZE;
        ahrs_sensor->offsets.compass_offsets[i] /= CALIB_BUFFER_SIZE;
    }

    ahrs_sensor->offsets.compass_vector = sqrt(ahrs_sensor->offsets.compass_offsets[0] * ahrs_sensor->offsets.compass_offsets[0] +
                                               ahrs_sensor->offsets.compass_offsets[1] * ahrs_sensor->offsets.compass_offsets[1] +
                                               ahrs_sensor->offsets.compass_offsets[2] * ahrs_sensor->offsets.compass_offsets[2]);

#ifdef DEBUG_AHRS_CALIBRATION
    Serial.print("accel offset: "); Serial.print(String(ahrs_sensor->offsets.accel_offsets[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->offsets.accel_offsets[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->offsets.accel_offsets[2], 4));
    Serial.print("gyro offset: "); Serial.print(String(ahrs_sensor->offsets.gyro_offsets[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->offsets.gyro_offsets[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->offsets.gyro_offsets[2], 4));
    Serial.print("compass vector: "); Serial.println(String(ahrs_sensor->offsets.compass_vector, 4));
    Serial.println();
#endif

    EEPROM.put(OFFSETS_ADDR, ahrs_sensor->offsets);
}
