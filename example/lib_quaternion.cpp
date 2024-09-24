#include "lib_quaternion.h"

static Quaternion q = {1, 0, 0, 0};
static double deltaT = 0;
static uint32_t new_time = 0, old_time = 0;


void AHRS_quaternion_update(AHRS_sensor * ahrs_sensor) {
	new_time = micros();
	deltaT = new_time - old_time;
	old_time = new_time;
	deltaT = fabs(deltaT * 0.001 * 0.001);
	
// Data distribution depending on sensor orientation
#ifdef USE_FILTERED_DATA
	float ax = ahrs_sensor->filtered_accel[0], ay = ahrs_sensor->filtered_accel[1], az = ahrs_sensor->filtered_accel[2];
	float gx = ahrs_sensor->filtered_gyro[0], gy = ahrs_sensor->filtered_gyro[1], gz = ahrs_sensor->filtered_gyro[2];
	float mx = ahrs_sensor->filtered_compass[0], my = ahrs_sensor->filtered_compass[1], mz = ahrs_sensor->filtered_compass[2];
#endif
#ifdef USE_SCALED_DATA
	float ax = ahrs_sensor->scaled_accel[0], ay = ahrs_sensor->scaled_accel[1], az = ahrs_sensor->scaled_accel[2];
	float gx = ahrs_sensor->scaled_gyro[0], gy = ahrs_sensor->scaled_gyro[1], gz = ahrs_sensor->scaled_gyro[2];
	float mx = ahrs_sensor->scaled_compass[0], my = ahrs_sensor->scaled_compass[1], mz = ahrs_sensor->scaled_compass[2];
#endif

#ifdef DEBUG_COMPASS_VECTOR_DEVIATION
  Serial.print("COMPASS_VECTOR: ");
  Serial.println(String(sqrt(mx * mx + my * my + mz * mz), 4));

  Serial.print("COMPASS_OFFSETS_VECTOR: ");
  Serial.println(String(ahrs_sensor->offsets.compass_vector, 4));

	Serial.print("DEVIATION: ");
	Serial.println(String(fabs(sqrt(mx * mx + my * my + mz * mz) - ahrs_sensor->offsets.compass_vector) / ahrs_sensor->offsets.compass_vector * 100, 4));
	Serial.println();
#endif

	if ((mx == 0.0 && my == 0.0 && mz == 0.0) || (fabs(sqrt(mx * mx + my * my + mz * mz) - ahrs_sensor->offsets.compass_vector) / ahrs_sensor->offsets.compass_vector * 100 > ALLOWABLE_DEVIATION)) {
    IMU_filter_update(ax, -ay, -az, gx, -gy, -gz);
#ifdef DEBUG_IMU_FILTER
		Serial.println("IMU");
#endif 
	}
	else {
    AHRS_filter_update(ax, -ay, -az, gx, -gy, -gz, mx, -my, -mz);
#ifdef DEBUG_AHRS_FILTER
		Serial.println("AHRS");
#endif 
	}
}


void IMU_quaternion_update(AHRS_sensor * ahrs_sensor) {
	new_time = micros();
	deltaT = new_time - old_time;
	old_time = new_time;
	deltaT = fabs(deltaT * 0.001 * 0.001);
	
	// Data distribution depending on sensor orientation
#ifdef USE_FILTERED_DATA
	float ax = ahrs_sensor->filtered_accel[0], ay = ahrs_sensor->filtered_accel[1], az = ahrs_sensor->filtered_accel[2];
	float gx = ahrs_sensor->filtered_gyro[0], gy = ahrs_sensor->filtered_gyro[1], gz = ahrs_sensor->filtered_gyro[2];
#endif
#ifdef USE_SCALED_DATA
	float ax = ahrs_sensor->scaled_accel[0], ay = ahrs_sensor->scaled_accel[1], az = ahrs_sensor->scaled_accel[2];
	float gx = ahrs_sensor->scaled_gyro[0], gy = ahrs_sensor->scaled_gyro[1], gz = ahrs_sensor->scaled_gyro[2];
#endif

	IMU_filter_update(ax, -ay, -az, gx, -gy, -gz);
}


void AHRS_filter_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    // TODO:
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        IMU_filter_update(ax, ay, az, gx, gy, gz);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz);
    qDot2 = 0.5f * (q.w * gx + q.y * gz - q.z * gy);
    qDot3 = 0.5f * (q.w * gy - q.x * gz + q.z * gx);
    qDot4 = 0.5f * (q.w * gz + q.x * gy - q.y * gx);

    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q.w * mx;
        _2q0my = 2.0f * q.w * my;
        _2q0mz = 2.0f * q.w * mz;
        _2q1mx = 2.0f * q.x * mx;
        _2q0 = 2.0f * q.w;
        _2q1 = 2.0f * q.x;
        _2q2 = 2.0f * q.y;
        _2q3 = 2.0f * q.z;
        _2q0q2 = 2.0f * q.w * q.y;
        _2q2q3 = 2.0f * q.y * q.z;
        q0q0 = q.w * q.w;
        q0q1 = q.w * q.x;
        q0q2 = q.w * q.y;
        q0q3 = q.w * q.z;
        q1q1 = q.x * q.x;
        q1q2 = q.x * q.y;
        q1q3 = q.x * q.z;
        q2q2 = q.y * q.y;
        q2q3 = q.y * q.z;
        q3q3 = q.z * q.z;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q.z + _2q0mz * q.y + mx * q1q1 + _2q1 * my * q.y + _2q1 * mz * q.z - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q.z + my * q0q0 - _2q0mz * q.x + _2q1mx * q.y - my * q1q1 + my * q2q2 + _2q2 * mz * q.z - my * q3q3;
        _2bx = (float)sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q.y + _2q0my * q.x + mz * q0q0 + _2q1mx * q.z - mz * q1q1 + _2q2 * my * q.z - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q.y * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.z + _2bz * q.x) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.y * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.x * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q.z * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.y + _2bz * q.w) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.z - _4bz * q.x) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.y * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q.y - _2bz * q.w) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.x + _2bz * q.z) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.w - _4bz * q.y) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q.z + _2bz * q.x) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.w + _2bz * q.y) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.x * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);    // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta_AHRS * s0;
        qDot2 -= beta_AHRS * s1;
        qDot3 -= beta_AHRS * s2;
        qDot4 -= beta_AHRS * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.w += qDot1 * deltaT;
    q.x += qDot2 * deltaT;
    q.y += qDot3 * deltaT;
    q.z += qDot4 * deltaT;

    recipNorm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);    // normalise quaternion
	q.w *= recipNorm;
	q.x *= recipNorm;
	q.y *= recipNorm;
	q.z *= recipNorm;

    
	
//	float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;   // short name local variable for readability
//	float norm;
//	float hx, hy, _2bx, _2bz;
//	float s1, s2, s3, s4;
//	float qDot1, qDot2, qDot3, qDot4;
//
//	/* Auxiliary variables to avoid repeated arithmetic */
//	float _2q1mx;
//	float _2q1my;
//	float _2q1mz;
//	float _2q2mx;
//	float _4bx;
//	float _4bz;
//	float _2q1 = 2.0f * q1;
//	float _2q2 = 2.0f * q2;
//	float _2q3 = 2.0f * q3;
//	float _2q4 = 2.0f * q4;
//	float _2q1q3 = 2.0f * q1 * q3;
//	float _2q3q4 = 2.0f * q3 * q4;
//	float q1q1 = q1 * q1;
//	float q1q2 = q1 * q2;
//	float q1q3 = q1 * q3;
//	float q1q4 = q1 * q4;
//	float q2q2 = q2 * q2;
//	float q2q3 = q2 * q3;
//	float q2q4 = q2 * q4;
//	float q3q3 = q3 * q3;
//	float q3q4 = q3 * q4;
//	float q4q4 = q4 * q4;
//
//	/* Normalise accelerometer measurement */
//	norm = (float)sqrt(ax * ax + ay * ay + az * az);
//	if (norm == 0.0f) return; // handle NaN
//	norm = 1 / norm;        // use reciprocal for division
//	ax *= norm;
//	ay *= norm;
//	az *= norm;
//
//	/* Normalise magnetometer measurement */
//	norm = (float)sqrt(mx * mx + my * my + mz * mz);
//	if (norm == 0.0f) return; // handle NaN
//	norm = 1 / norm;        // use reciprocal for division
//	mx *= norm;
//	my *= norm;
//	mz *= norm;
//
//	/* Reference direction of Earth's magnetic field */
//	_2q1mx = 2.0f * q1 * mx;
//	_2q1my = 2.0f * q1 * my;
//	_2q1mz = 2.0f * q1 * mz;
//	_2q2mx = 2.0f * q2 * mx;
//	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
//	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
//	_2bx = (float)sqrt(hx * hx + hy * hy);
//	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
//	_4bx = 2.0f * _2bx;
//	_4bz = 2.0f * _2bz;
//
//	/* Gradient decent algorithm corrective step */
//	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
//	norm = invSqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
//	s1 *= norm;
//	s2 *= norm;
//	s3 *= norm;
//	s4 *= norm;
//
//	/* Compute rate of change of quaternion */
//	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta_AHRS * s1;
//	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta_AHRS * s2;
//	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta_AHRS * s3;
//	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta_AHRS * s4;
//
//	/* Integrate to yield quaternion */ 
//	q1 += qDot1 * deltaT;
//	q2 += qDot2 * deltaT;
//	q3 += qDot3 * deltaT;
//	q4 += qDot4 * deltaT;
//
//	norm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
//	q.w = q1 * norm;
//	q.x = q2 * norm;
//	q.y = q3 * norm;
//	q.z = q4 * norm;
}


void IMU_filter_update(float ax, float ay, float az, float gx, float gy, float gz) {
	float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;  // short name local variable for readability
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	/* Rate of change of quaternion from gyroscope */
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	/* Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation) */
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		/* Normalise accelerometer measurement */
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		/* Auxiliary variables to avoid repeated arithmetic */
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		/* Gradient decent algorithm corrective step */
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		/* Apply feedback step */
		qDot1 -= beta_IMU * s0;
		qDot2 -= beta_IMU * s1;
		qDot3 -= beta_IMU * s2;
		qDot4 -= beta_IMU * s3;
	}

	/* Integrate rate of change of quaternion to yield quaternion */
	q0 += qDot1 * deltaT;
	q1 += qDot2 * deltaT;
	q2 += qDot3 * deltaT;
	q3 += qDot4 * deltaT;

	/* Normalise quaternion */
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

	q.w = q0;
	q.x = q1;
	q.y = q2;
	q.z = q3;
}


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


float get_roll() { // attitude
	return to_degrees(atan2f(2.0f * (q.w * q.x + q.y * q.z), 1 - 2.0 * (q.x * q.x + q.y * q.y))); // values in degrees
}


float get_pitch() { // bank
	return to_degrees(-asinf(2.0f * (q.w * q.y - q.z * q.x))); // values in degrees
}


float get_yaw() {
	float yaw = -atan2(2.0f * (q.w * q.z + q.x * q.y), 1 - 2.0f * (q.y * q.y + q.z * q.z));
	
	if (yaw < 0)
		yaw += 2 * PI;
	if (yaw > 2 * PI)
		yaw -= 2 * PI;

	return to_degrees(yaw); // values in degrees
}


float get_heading() {
	float yaw = -atan2(2.0f * (q.w * q.z + q.x * q.y), 1 - 2.0f * (q.y * q.y + q.z * q.z));

	yaw += SPB_DEVIATION; // Saint-Petersburg

	if (yaw < 0)
		yaw += 2 * PI;
	if (yaw > 2 * PI)
		yaw -= 2 * PI;

	return to_degrees(yaw); // values in degrees
}


Quaternion get_quaternion() {
    return q;
}
