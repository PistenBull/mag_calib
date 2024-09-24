#include "lib_magnetometer.h"


static void calculate_segment_centers(calibration_raw_data * raw_data, segment_centers * centers);
static void calculate_vector_matrix(segment_centers * centers, float vector_matrix[3][3]);
static void calculate_inverse_matrix(float vector_matrix[3][3], magnetometer * sensor);
static void calculate_offsets_matrix(segment_centers * centers, magnetometer * sensor);
static void calculate_transformation_matrix(calibration_raw_data * raw_data, segment_centers * centers, magnetometer * sensor);
static vector_3f matrix_multiplication(float left_matrix[3][3], vector_3f right_matrix);


int magnetometer_calibration(magnetometer * sensor) {
	segment_centers centers;
	float vector_matrix[3][3];

	calculate_segment_centers(&sensor->raw_data, &centers);
	calculate_vector_matrix(&centers, vector_matrix);
	calculate_inverse_matrix(vector_matrix, sensor);
	calculate_offsets_matrix(&centers, sensor);
	calculate_transformation_matrix(&sensor->raw_data, &centers, sensor);

	return 1;
}


static void calculate_segment_centers(calibration_raw_data * raw_data, segment_centers * centers) {
	centers->X_plus_center.x = (raw_data->X_plus_0.x + raw_data->X_plus_180.x) / 2.0f;
	centers->X_plus_center.y = (raw_data->X_plus_0.y + raw_data->X_plus_180.y) / 2.0f;
	centers->X_plus_center.z = (raw_data->X_plus_0.z + raw_data->X_plus_180.z) / 2.0f;

	centers->X_minus_center.x = (raw_data->X_minus_0.x + raw_data->X_minus_180.x) / 2.0f;
	centers->X_minus_center.y = (raw_data->X_minus_0.y + raw_data->X_minus_180.y) / 2.0f;
	centers->X_minus_center.z = (raw_data->X_minus_0.z + raw_data->X_minus_180.z) / 2.0f;

	centers->Y_plus_center.x = (raw_data->Y_plus_0.x + raw_data->Y_plus_180.x) / 2.0f;
	centers->Y_plus_center.y = (raw_data->Y_plus_0.y + raw_data->Y_plus_180.y) / 2.0f;
	centers->Y_plus_center.z = (raw_data->Y_plus_0.z + raw_data->Y_plus_180.z) / 2.0f;

	centers->Y_minus_center.x = (raw_data->Y_minus_0.x + raw_data->Y_minus_180.x) / 2.0f;
	centers->Y_minus_center.y = (raw_data->Y_minus_0.y + raw_data->Y_minus_180.y) / 2.0f;
	centers->Y_minus_center.z = (raw_data->Y_minus_0.z + raw_data->Y_minus_180.z) / 2.0f;

	centers->Z_plus_center.x = (raw_data->Z_plus_0.x + raw_data->Z_plus_180.x) / 2.0f;
	centers->Z_plus_center.y = (raw_data->Z_plus_0.y + raw_data->Z_plus_180.y) / 2.0f;
	centers->Z_plus_center.z = (raw_data->Z_plus_0.z + raw_data->Z_plus_180.z) / 2.0f;

	centers->Z_minus_center.x = (raw_data->Z_minus_0.x + raw_data->Z_minus_180.x) / 2.0f;
	centers->Z_minus_center.y = (raw_data->Z_minus_0.y + raw_data->Z_minus_180.y) / 2.0f;
	centers->Z_minus_center.z = (raw_data->Z_minus_0.z + raw_data->Z_minus_180.z) / 2.0f;
}


static void calculate_vector_matrix(segment_centers * centers, float vector_matrix[3][3]) {
	vector_3f X_vector;
	vector_3f Y_vector;
	vector_3f Z_vector;

	X_vector.x = centers->X_plus_center.x - centers->X_minus_center.x;
	X_vector.y = centers->X_plus_center.y - centers->X_minus_center.y;
	X_vector.z = centers->X_plus_center.z - centers->X_minus_center.z;

	Y_vector.x = centers->Y_plus_center.x - centers->Y_minus_center.x;
	Y_vector.y = centers->Y_plus_center.y - centers->Y_minus_center.y;
	Y_vector.z = centers->Y_plus_center.z - centers->Y_minus_center.z;

	Z_vector.x = centers->Z_plus_center.x - centers->Z_minus_center.x;
	Z_vector.y = centers->Z_plus_center.y - centers->Z_minus_center.y;
	Z_vector.z = centers->Z_plus_center.z - centers->Z_minus_center.z;

	vector_matrix[0][0] = X_vector.x / X_vector.x;
	vector_matrix[0][1] = Y_vector.x / Y_vector.y;
	vector_matrix[0][2] = Z_vector.x / Z_vector.z;
	vector_matrix[1][0] = X_vector.y / X_vector.x;
	vector_matrix[1][1] = Y_vector.y / Y_vector.y;
	vector_matrix[1][2] = Z_vector.y / Z_vector.z;
	vector_matrix[2][0] = X_vector.z / X_vector.x;
	vector_matrix[2][1] = Y_vector.z / Y_vector.y;
	vector_matrix[2][2] = Z_vector.z / Z_vector.z;
}


static void calculate_inverse_matrix(float vector_matrix[3][3], magnetometer * sensor) {
	float det = vector_matrix[0][0] * (vector_matrix[1][1] * vector_matrix[2][2] - vector_matrix[2][1] * vector_matrix[1][2]) -
                vector_matrix[0][1] * (vector_matrix[1][0] * vector_matrix[2][2] - vector_matrix[1][2] * vector_matrix[2][0]) +
                vector_matrix[0][2] * (vector_matrix[1][0] * vector_matrix[2][1] - vector_matrix[1][1] * vector_matrix[2][0]);

	float adj[3][3];

	adj[0][0] = ((vector_matrix[1][1] * vector_matrix[2][2]) - (vector_matrix[2][1] * vector_matrix[1][2]));
	adj[0][1] = ((vector_matrix[0][2] * vector_matrix[2][1]) - (vector_matrix[0][1] * vector_matrix[2][2]));
	adj[0][2] = ((vector_matrix[0][1] * vector_matrix[1][2]) - (vector_matrix[0][2] * vector_matrix[1][1]));
	adj[1][0] = ((vector_matrix[1][2] * vector_matrix[2][0]) - (vector_matrix[1][0] * vector_matrix[2][2]));
	adj[1][1] = ((vector_matrix[0][0] * vector_matrix[2][2]) - (vector_matrix[0][2] * vector_matrix[2][0]));
	adj[1][2] = ((vector_matrix[0][2] * vector_matrix[1][0]) - (vector_matrix[0][0] * vector_matrix[1][2]));
	adj[2][0] = ((vector_matrix[1][0] * vector_matrix[2][1]) - (vector_matrix[2][0] * vector_matrix[1][1]));
	adj[2][1] = ((vector_matrix[0][1] * vector_matrix[2][0]) - (vector_matrix[0][0] * vector_matrix[2][1]));
	adj[2][2] = ((vector_matrix[0][0] * vector_matrix[1][1]) - (vector_matrix[1][0] * vector_matrix[0][1]));

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			sensor->magnetometer_transformation[i][j] = adj[i][j] / det;
}


static void calculate_offsets_matrix(segment_centers * centers, magnetometer * sensor) {
	vector_3f X_plus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->X_plus_center);
	vector_3f X_minus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->X_minus_center);
	vector_3f Y_plus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Y_plus_center);
	vector_3f Y_minus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Y_minus_center);
	vector_3f Z_plus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Z_plus_center);
	vector_3f Z_minus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Z_minus_center);

	sensor->magnetometer_offsets[0] = (X_plus_vector.x + X_minus_vector.x + Y_plus_vector.x + Y_minus_vector.x + Z_plus_vector.x + Z_minus_vector.x) / 6.0f;
	sensor->magnetometer_offsets[1] = (X_plus_vector.y + X_minus_vector.y + Y_plus_vector.y + Y_minus_vector.y + Z_plus_vector.y + Z_minus_vector.y) / 6.0f;
	sensor->magnetometer_offsets[2] = (X_plus_vector.z + X_minus_vector.z + Y_plus_vector.z + Y_minus_vector.z + Z_plus_vector.z + Z_minus_vector.z) / 6.0f;
}


static void calculate_transformation_matrix(calibration_raw_data * raw_data, segment_centers * centers, magnetometer * sensor) {
	vector_3f X_plus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->X_plus_center);
	vector_3f X_minus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->X_minus_center);
	vector_3f Y_plus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Y_plus_center);
	vector_3f Y_minus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Y_minus_center);
	vector_3f Z_plus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Z_plus_center);
	vector_3f Z_minus_vector = matrix_multiplication(sensor->magnetometer_transformation, centers->Z_minus_center);

	float intValue1 = fabs(X_plus_vector.x - X_minus_vector.x) / 2.0f;
	float intValue2 = fabs(Y_plus_vector.y - Y_minus_vector.y) / 2.0f;
	float intValue3 = fabs(Z_plus_vector.z - Z_minus_vector.z) / 2.0f;
	
	vector_3f X_plus_0 = matrix_multiplication(sensor->magnetometer_transformation, raw_data->X_plus_0);
	vector_3f Y_plus_0 = matrix_multiplication(sensor->magnetometer_transformation, raw_data->Y_plus_0);
	vector_3f Z_plus_0 = matrix_multiplication(sensor->magnetometer_transformation, raw_data->Z_plus_0);

	float intValue4 = (sqrt(intValue1 * intValue1 + X_plus_0.y * X_plus_0.y + X_plus_0.z * X_plus_0.z) +
                       sqrt(Y_plus_0.x * Y_plus_0.x + intValue2 * intValue2 + Y_plus_0.z * Y_plus_0.z) +
                       sqrt(Z_plus_0.x * Z_plus_0.x + Z_plus_0.y * Z_plus_0.y + intValue3 * intValue3)) / 3.0f * 2.0f;

	float scaleX = fabs(intValue4 / (X_plus_vector.x - X_minus_vector.x));
	float scaleY = fabs(intValue4 / (Y_plus_vector.y - Y_minus_vector.y));
	float scaleZ = fabs(intValue4 / (Z_plus_vector.z - Z_minus_vector.z));

	sensor->magnetometer_transformation[0][0] = sensor->magnetometer_transformation[0][0] * scaleX;
	sensor->magnetometer_transformation[0][1] = sensor->magnetometer_transformation[0][1] * scaleY;
	sensor->magnetometer_transformation[0][2] = sensor->magnetometer_transformation[0][2] * scaleZ;
	sensor->magnetometer_transformation[1][0] = sensor->magnetometer_transformation[1][0] * scaleX;
	sensor->magnetometer_transformation[1][1] = sensor->magnetometer_transformation[1][1] * scaleY;
	sensor->magnetometer_transformation[1][2] = sensor->magnetometer_transformation[1][2] * scaleZ;
	sensor->magnetometer_transformation[2][0] = sensor->magnetometer_transformation[2][0] * scaleX;
	sensor->magnetometer_transformation[2][1] = sensor->magnetometer_transformation[2][1] * scaleY;
	sensor->magnetometer_transformation[2][2] = sensor->magnetometer_transformation[2][2] * scaleZ;
}


static vector_3f matrix_multiplication(float left_matrix[3][3], vector_3f right_matrix) {
	vector_3f result;

	result.x = left_matrix[0][0] * right_matrix.x + left_matrix[0][1] * right_matrix.y + left_matrix[0][2] * right_matrix.z;
	result.y = left_matrix[1][0] * right_matrix.x + left_matrix[1][1] * right_matrix.y + left_matrix[1][2] * right_matrix.z;
	result.z = left_matrix[2][0] * right_matrix.x + left_matrix[2][1] * right_matrix.y + left_matrix[2][2] * right_matrix.z;
	
	return result;
}


vector_3f magnetometer_apply_offsets(vector_3f raw_magnetometer, float bias_matrix[3], float transformation_matrix[3][3]) {
    vector_3f scaled_magnetometer;
	vector_3f temp_magnetometer = {raw_magnetometer.x, raw_magnetometer.y, raw_magnetometer.z};
	
	temp_compass.x -=  bias_matrix[0];
	temp_compass.y -=  bias_matrix[1];
	temp_compass.z -=  bias_matrix[2];
	
	scaled_magnetometer.x = transformation_matrix[0][0] * temp_compass.x + transformation_matrix[0][1] * temp_compass.y + transformation_matrix[0][2] * temp_compass.z;
	scaled_magnetometer.y = transformation_matrix[1][0] * temp_compass.x + transformation_matrix[1][1] * temp_compass.y + transformation_matrix[1][2] * temp_compass.z;
	scaled_magnetometer.z = transformation_matrix[2][0] * temp_compass.x + transformation_matrix[2][1] * temp_compass.y + transformation_matrix[2][2] * temp_compass.z;

    return scaled_magnetometer;
}
