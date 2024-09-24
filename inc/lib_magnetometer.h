#ifndef _LIB_MAGNETOMETER_H_
#define _LIB_MAGNETOMETER_H_

#include <math.h>

#define VECTOR_3(type, alias) \
typedef struct {    \
    type x, y, z;   \
} vector_ ## 3 ## alias

VECTOR_3(int, i);
VECTOR_3(float, f);
VECTOR_3(short, s);
VECTOR_3(double, d);

typedef struct {
    vector_3f X_plus_0;
    vector_3f X_plus_180;

    vector_3f X_minus_0;
    vector_3f X_minus_180;

    vector_3f Y_plus_0;
    vector_3f Y_plus_180;

    vector_3f Y_minus_0;
    vector_3f Y_minus_180;

    vector_3f Z_plus_0;
    vector_3f Z_plus_180;

    vector_3f Z_minus_0;
    vector_3f Z_minus_180;
} calibration_raw_data;

typedef struct {
	vector_3f X_plus_center;
	vector_3f X_minus_center;
	vector_3f Y_plus_center;
	vector_3f Y_minus_center;
	vector_3f Z_plus_center;
	vector_3f Z_minus_center;
} segment_centers;

typedef struct {
	vector_3f raw_magnetometer;
	vector_3f scaled_magnetometer;

	calibration_raw_data raw_data;
	float magnetometer_offsets[3];
	float magnetometer_transformation[3][3];
} magnetometer;

int magnetometer_calibration(magnetometer * sensor);
vector_3f magnetometer_apply_offsets(vector_3f raw_magnetometer, float bias_matrix[3], float transformation_matrix[3][3]);

#endif /* _LIB_MAGNETOMETER_H_ */
