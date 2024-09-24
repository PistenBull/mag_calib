#include "lib_compass.h"


static Quaternion q;


void compass_calibration(Raw_data* raw_data, AHRS_sensor* ahrs_sensor) {
	Segment_centers seg_centers;
	float vector_matrix[3][3];

	calculate_segment_centers(raw_data, &seg_centers);
	calculate_vector_matrix(&seg_centers, vector_matrix);
	calculate_inverse_matrix(vector_matrix, ahrs_sensor);
	calculate_bias_matrix(&seg_centers, ahrs_sensor);
	calculate_transformation_matrix(raw_data, &seg_centers, ahrs_sensor);
  	calculate_compass_vector(raw_data, ahrs_sensor);

  EEPROM.put(OFFSETS_ADDR, ahrs_sensor->offsets);
}


void calculate_segment_centers(Raw_data* raw_data, Segment_centers* seg_centers) {
	seg_centers->X_plus_center[0] = (raw_data->X_plus_0[0] + raw_data->X_plus_180[0]) / 2.0;
	seg_centers->X_plus_center[1] = (raw_data->X_plus_0[1] + raw_data->X_plus_180[1]) / 2.0;
	seg_centers->X_plus_center[2] = (raw_data->X_plus_0[2] + raw_data->X_plus_180[2]) / 2.0;

	seg_centers->X_minus_center[0] = (raw_data->X_minus_0[0] + raw_data->X_minus_180[0]) / 2.0;
	seg_centers->X_minus_center[1] = (raw_data->X_minus_0[1] + raw_data->X_minus_180[1]) / 2.0;
	seg_centers->X_minus_center[2] = (raw_data->X_minus_0[2] + raw_data->X_minus_180[2]) / 2.0;

	seg_centers->Y_plus_center[0] = (raw_data->Y_plus_0[0] + raw_data->Y_plus_180[0]) / 2.0;
	seg_centers->Y_plus_center[1] = (raw_data->Y_plus_0[1] + raw_data->Y_plus_180[1]) / 2.0;
	seg_centers->Y_plus_center[2] = (raw_data->Y_plus_0[2] + raw_data->Y_plus_180[2]) / 2.0;

	seg_centers->Y_minus_center[0] = (raw_data->Y_minus_0[0] + raw_data->Y_minus_180[0]) / 2.0;
	seg_centers->Y_minus_center[1] = (raw_data->Y_minus_0[1] + raw_data->Y_minus_180[1]) / 2.0;
	seg_centers->Y_minus_center[2] = (raw_data->Y_minus_0[2] + raw_data->Y_minus_180[2]) / 2.0;

	seg_centers->Z_plus_center[0] = (raw_data->Z_plus_0[0] + raw_data->Z_plus_180[0]) / 2.0;
	seg_centers->Z_plus_center[1] = (raw_data->Z_plus_0[1] + raw_data->Z_plus_180[1]) / 2.0;
	seg_centers->Z_plus_center[2] = (raw_data->Z_plus_0[2] + raw_data->Z_plus_180[2]) / 2.0;

	seg_centers->Z_minus_center[0] = (raw_data->Z_minus_0[0] + raw_data->Z_minus_180[0]) / 2.0;
	seg_centers->Z_minus_center[1] = (raw_data->Z_minus_0[1] + raw_data->Z_minus_180[1]) / 2.0;
	seg_centers->Z_minus_center[2] = (raw_data->Z_minus_0[2] + raw_data->Z_minus_180[2]) / 2.0;
}


void calculate_vector_matrix(Segment_centers* seg_centers, float vector_matrix[3][3]) {
	float X_vector[3];
	float Y_vector[3];
	float Z_vector[3];

	X_vector[0] = seg_centers->X_plus_center[0] - seg_centers->X_minus_center[0];
	X_vector[1] = seg_centers->X_plus_center[1] - seg_centers->X_minus_center[1];
	X_vector[2] = seg_centers->X_plus_center[2] - seg_centers->X_minus_center[2];

	Y_vector[0] = seg_centers->Y_plus_center[0] - seg_centers->Y_minus_center[0];
	Y_vector[1] = seg_centers->Y_plus_center[1] - seg_centers->Y_minus_center[1];
	Y_vector[2] = seg_centers->Y_plus_center[2] - seg_centers->Y_minus_center[2];

	Z_vector[0] = seg_centers->Z_plus_center[0] - seg_centers->Z_minus_center[0];
	Z_vector[1] = seg_centers->Z_plus_center[1] - seg_centers->Z_minus_center[1];
	Z_vector[2] = seg_centers->Z_plus_center[2] - seg_centers->Z_minus_center[2];

	vector_matrix[0][0] = X_vector[0] / X_vector[0];
	vector_matrix[0][1] = Y_vector[0] / Y_vector[1];
	vector_matrix[0][2] = Z_vector[0] / Z_vector[2];
	vector_matrix[1][0] = X_vector[1] / X_vector[0];
	vector_matrix[1][1] = Y_vector[1] / Y_vector[1];
	vector_matrix[1][2] = Z_vector[1] / Z_vector[2];
	vector_matrix[2][0] = X_vector[2] / X_vector[0];
	vector_matrix[2][1] = Y_vector[2] / Y_vector[1];
	vector_matrix[2][2] = Z_vector[2] / Z_vector[2];
}


void calculate_inverse_matrix(float vector_matrix[3][3], AHRS_sensor* ahrs_sensor) {
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
			ahrs_sensor->offsets.transformation_matrix[i][j] = adj[i][j] / det;
}


void calculate_bias_matrix(Segment_centers* seg_centers, AHRS_sensor* ahrs_sensor) {
	float X_plus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->X_plus_center, X_plus_vector);
	float X_minus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->X_minus_center, X_minus_vector);
	float Y_plus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Y_plus_center, Y_plus_vector);
	float Y_minus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Y_minus_center, Y_minus_vector);
	float Z_plus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Z_plus_center, Z_plus_vector);
	float Z_minus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Z_minus_center, Z_minus_vector);

	ahrs_sensor->offsets.bias_matrix[0] = (X_plus_vector[0] + X_minus_vector[0] + Y_plus_vector[0] + Y_minus_vector[0] + Z_plus_vector[0] + Z_minus_vector[0]) / 6.0;
	ahrs_sensor->offsets.bias_matrix[1] = (X_plus_vector[1] + X_minus_vector[1] + Y_plus_vector[1] + Y_minus_vector[1] + Z_plus_vector[1] + Z_minus_vector[1]) / 6.0;
	ahrs_sensor->offsets.bias_matrix[2] = (X_plus_vector[2] + X_minus_vector[2] + Y_plus_vector[2] + Y_minus_vector[2] + Z_plus_vector[2] + Z_minus_vector[2]) / 6.0;

#ifdef DEBUG_COMPASS
    Serial.println("bias matrix: ");
    Serial.print(String(ahrs_sensor->offsets.bias_matrix[0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->offsets.bias_matrix[1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->offsets.bias_matrix[2], 4));
    Serial.println();
#endif
}


void calculate_transformation_matrix(Raw_data* raw_data, Segment_centers* seg_centers, AHRS_sensor* ahrs_sensor) {
	float X_plus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->X_plus_center, X_plus_vector);
	float X_minus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->X_minus_center, X_minus_vector);
	float Y_plus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Y_plus_center, Y_plus_vector);
	float Y_minus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Y_minus_center, Y_minus_vector);
	float Z_plus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Z_plus_center, Z_plus_vector);
	float Z_minus_vector[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, seg_centers->Z_minus_center, Z_minus_vector);

	float intValue1 = fabs(X_plus_vector[0] - X_minus_vector[0]) / 2.0;
	float intValue2 = fabs(Y_plus_vector[1] - Y_minus_vector[1]) / 2.0;
	float intValue3 = fabs(Z_plus_vector[2] - Z_minus_vector[2]) / 2.0;

	float X_plus_0[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, raw_data->X_plus_0, X_plus_0);
	float Y_plus_0[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, raw_data->Y_plus_0, Y_plus_0);
	float Z_plus_0[3]; matrix_multiplication(ahrs_sensor->offsets.transformation_matrix, raw_data->Z_plus_0, Z_plus_0);

	float intValue4 = (sqrt(intValue1 * intValue1 + X_plus_0[1] * X_plus_0[1] + X_plus_0[2] * X_plus_0[2]) +
                        sqrt(Y_plus_0[0] * Y_plus_0[0] + intValue2 * intValue2 + Y_plus_0[2] * Y_plus_0[2]) +
                        sqrt(Z_plus_0[0] * Z_plus_0[0] + Z_plus_0[1] * Z_plus_0[1] + intValue3 * intValue3)) / 3.0 * 2.0;

	float scaleX = fabs(intValue4 / (X_plus_vector[0] - X_minus_vector[0]));
	float scaleY = fabs(intValue4 / (Y_plus_vector[1] - Y_minus_vector[1]));
	float scaleZ = fabs(intValue4 / (Z_plus_vector[2] - Z_minus_vector[2]));

	ahrs_sensor->offsets.transformation_matrix[0][0] = ahrs_sensor->offsets.transformation_matrix[0][0] * scaleX;
	ahrs_sensor->offsets.transformation_matrix[0][1] = ahrs_sensor->offsets.transformation_matrix[0][1] * scaleY;
	ahrs_sensor->offsets.transformation_matrix[0][2] = ahrs_sensor->offsets.transformation_matrix[0][2] * scaleZ;
	ahrs_sensor->offsets.transformation_matrix[1][0] = ahrs_sensor->offsets.transformation_matrix[1][0] * scaleX;
	ahrs_sensor->offsets.transformation_matrix[1][1] = ahrs_sensor->offsets.transformation_matrix[1][1] * scaleY;
	ahrs_sensor->offsets.transformation_matrix[1][2] = ahrs_sensor->offsets.transformation_matrix[1][2] * scaleZ;
	ahrs_sensor->offsets.transformation_matrix[2][0] = ahrs_sensor->offsets.transformation_matrix[2][0] * scaleX;
	ahrs_sensor->offsets.transformation_matrix[2][1] = ahrs_sensor->offsets.transformation_matrix[2][1] * scaleY;
	ahrs_sensor->offsets.transformation_matrix[2][2] = ahrs_sensor->offsets.transformation_matrix[2][2] * scaleZ;

#ifdef DEBUG_COMPASS
    Serial.println("transform matrix: ");
    Serial.print(String(ahrs_sensor->offsets.transformation_matrix[0][0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->offsets.transformation_matrix[0][1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->offsets.transformation_matrix[0][2], 4));
    Serial.print(String(ahrs_sensor->offsets.transformation_matrix[1][0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->offsets.transformation_matrix[1][1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->offsets.transformation_matrix[1][2], 4));
    Serial.print(String(ahrs_sensor->offsets.transformation_matrix[2][0], 4)); Serial.print(", "); Serial.print(String(ahrs_sensor->offsets.transformation_matrix[2][1], 4)); Serial.print(", "); Serial.println(String(ahrs_sensor->offsets.transformation_matrix[2][2], 4));
    Serial.println();
#endif
}


void matrix_multiplication(float left_matrix[3][3], float right_matrix[3], float result[3]) {
	
	result[0] = left_matrix[0][0] * right_matrix[0] + left_matrix[0][1] * right_matrix[1] + left_matrix[0][2] * right_matrix[2];
	result[1] = left_matrix[1][0] * right_matrix[0] + left_matrix[1][1] * right_matrix[1] + left_matrix[1][2] * right_matrix[2];
	result[2] = left_matrix[2][0] * right_matrix[0] + left_matrix[2][1] * right_matrix[1] + left_matrix[2][2] * right_matrix[2];
}


void get_calibrated_compass(float raw_array[3], float res_array[3], float trans_matrix[3][3], float bias_matrix[3]) {
	float temp_compass[3] = { raw_array[0], raw_array[1], raw_array[2] };

	temp_compass[0] -= bias_matrix[0];
	temp_compass[1] -= bias_matrix[1];
	temp_compass[2] -= bias_matrix[2];

	res_array[0] = trans_matrix[0][0] * temp_compass[0] + trans_matrix[0][1] * temp_compass[1] + trans_matrix[0][2] * temp_compass[2];
	res_array[1] = trans_matrix[1][0] * temp_compass[0] + trans_matrix[1][1] * temp_compass[1] + trans_matrix[1][2] * temp_compass[2];
	res_array[2] = trans_matrix[2][0] * temp_compass[0] + trans_matrix[2][1] * temp_compass[1] + trans_matrix[2][2] * temp_compass[2];
}


void calculate_compass_vector(Raw_data* raw_data, AHRS_sensor* ahrs_sensor) {
    get_calibrated_compass(raw_data->X_plus_0, raw_data->X_plus_0, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->X_plus_180, raw_data->X_plus_180, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->X_minus_0, raw_data->X_minus_0, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->X_minus_180, raw_data->X_minus_180, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Y_plus_0, raw_data->Y_plus_0, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Y_plus_180, raw_data->Y_plus_180, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Y_minus_0, raw_data->Y_minus_0, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Y_minus_180, raw_data->Y_minus_180, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Z_plus_0, raw_data->Z_plus_0, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Z_plus_180, raw_data->Z_plus_180, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Z_minus_0, raw_data->Z_minus_0, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);
    get_calibrated_compass(raw_data->Z_minus_180, raw_data->Z_minus_180, ahrs_sensor->offsets.transformation_matrix, ahrs_sensor->offsets.bias_matrix);

    ahrs_sensor->offsets.compass_vector = (sqrt(pow(raw_data->X_plus_0[0], 2) + pow(raw_data->X_plus_0[1], 2) + pow(raw_data->X_plus_0[2], 2)) +
            sqrt(pow(raw_data->X_plus_180[0], 2) + pow(raw_data->X_plus_180[1], 2) + pow(raw_data->X_plus_180[2], 2)) +
            sqrt(pow(raw_data->X_minus_0[0], 2) + pow(raw_data->X_minus_0[1], 2) + pow(raw_data->X_minus_0[2], 2)) +
            sqrt(pow(raw_data->X_minus_180[0], 2) + pow(raw_data->X_minus_180[1], 2) + pow(raw_data->X_minus_180[2], 2)) +
            sqrt(pow(raw_data->Y_plus_0[0], 2) + pow(raw_data->Y_plus_0[1], 2) + pow(raw_data->Y_plus_0[2], 2)) +
            sqrt(pow(raw_data->Y_plus_180[0], 2) + pow(raw_data->Y_plus_180[1], 2) + pow(raw_data->Y_plus_180[2], 2)) +
            sqrt(pow(raw_data->Y_minus_0[0], 2) + pow(raw_data->Y_minus_0[1], 2) + pow(raw_data->Y_minus_0[2], 2)) +
            sqrt(pow(raw_data->Y_minus_180[0], 2) + pow(raw_data->Y_minus_180[1], 2) + pow(raw_data->Y_minus_180[2], 2)) +
            sqrt(pow(raw_data->Z_plus_0[0], 2) + pow(raw_data->Z_plus_0[1], 2) + pow(raw_data->Z_plus_0[2], 2)) +
            sqrt(pow(raw_data->Z_plus_180[0], 2) + pow(raw_data->Z_plus_180[1], 2) + pow(raw_data->Z_plus_180[2], 2)) +
            sqrt(pow(raw_data->Z_minus_0[0], 2) + pow(raw_data->Z_minus_0[1], 2) + pow(raw_data->Z_minus_0[2], 2)) +
            sqrt(pow(raw_data->Z_minus_180[0], 2) + pow(raw_data->Z_minus_180[1], 2) + pow(raw_data->Z_minus_180[2], 2))) / 12.0f;

#ifdef DEBUG_COMPASS
    Serial.print("compass_vector: "); Serial.println(String(ahrs_sensor->offsets.compass_vector, 4));
#endif
}


void reading_raw_compass_for_calibration(Raw_data* raw_data, AHRS_sensor* ahrs_sensor) {
    /*
    Углы Эйлера /// Кватернионы:
    Сильно зависит от того как ориентирован датчик относительно платы
    
                                                       // Важно учитывать в какую сторону относительно оси идёт вращение  
    Z_minus_0 --> pitch = 0, roll = 0, yaw = 0;       /// w = 1, x = 0, y = 0, z = 0;             либо w = -1, x = 0, y = 0, z = 0;
    Z_minus_180 --> pitch = 0, roll = 0, yaw = 180;   /// w = 0, x = 0, y = 0, z = 1;             либо w = 0, x = 0, y = 0, z = -1;
    Z_plus_0 --> pitch = 0, roll = 180, yaw = 180;    /// w = 0, x = 1, y = 0, z = 0;             либо w = 0, x = -1, y = 0, z = 0;
    Z_plus_180 --> pitch = 0, roll = 180, yaw = 0;    /// w = 0, x = 0, y = 1, z = 0;             либо w = 0, x = 0, y = -1, z = 0;

    Y_minus_0 --> pitch = 0, roll = 90, yaw = 0;      /// w = 0.707, x = 0.707, y = 0, z = 0;     либо w = -0.707, x = -0.707, y = 0, z = 0;
    Y_minus_180 --> pitch = 0, roll = 90, yaw = 180;  /// w = 0, x = 0, y = -0.707, z = -0.707;   либо w = 0, x = 0, y = 0.707, z = 0.707;
    Y_plus_0 --> pitch = 0, roll = -90, yaw = 0;      /// w = 0.707, x = -0.707, y = 0, z = 0;    либо w = -0.707, x = 0.707, y = 0, z = 0;
    Y_plus_180 --> pitch = 0, roll = -90, yaw = 180;  /// w = 0, x = 0, y = -0.707, z = 0.707;    либо w = 0, x = 0, y = 0.707, z = -0.707;

    X_minus_0 --> pitch = -90, roll = 0, yaw = 270;   /// w = 0.5, x = -0.5, y = 0.5, z = 0.5;    либо w = -0.5, x = 0.5, y = -0.5, z = -0.5;
    X_minus_180 --> pitch = -90, roll = 0, yaw = 90;  /// w = 0.5, x = 0.5, y = 0.5, z = -0.5;    либо w = -0.5, x = -0.5, y = -0.5, z = 0.5;
    X_plus_0 --> pitch = 90, roll = 0, yaw = 270;     /// w = 0.5, x = 0.5, y = -0.5, z = 0.5;    либо w = -0.5, x = -0.5, y = 0.5, z = -0.5;
    X_plus_180 --> pitch = 90, roll = 0, yaw = 90;    /// w = 0.5, x = -0.5, y = -0.5, z = -0.5;  либо w = -0.5, x = 0.5, y = 0.5, z = 0.5;

    Формула для вычисления диапазона в котором может неходиться вектор поворота:

    A </> to_degrees(2 * acos(w));
    D > sqrt(pow(q.x - target_x, 2) + pow(q.y - target_y, 2) + pow(q.z - target_z, 2));

    Примерные параметры:
    ANGLE 3
    D 0.05
    */


    int count = 0;
    while (count != 12) {
        get_filtered_AHRS(ahrs_sensor);
        IMU_quaternion_update(ahrs_sensor); 
        q = get_quaternion();
        
#ifdef DEBUG_IMU_READ
        Serial.print(String(q.w, 4));
        Serial.print(", ");
        Serial.print(String(q.x, 4));
        Serial.print(", ");
        Serial.print(String(q.y, 4));
        Serial.print(", ");
        Serial.println(String(q.z, 4));
#endif
        
        if (D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f)) && !(raw_data->z_minus_0)) {
            
			get_raw_compass(ahrs_sensor);
			raw_data->Z_minus_0[0] = ahrs_sensor->raw_compass[0];
			raw_data->Z_minus_0[1] = ahrs_sensor->raw_compass[1];
			raw_data->Z_minus_0[2] = ahrs_sensor->raw_compass[2];

			raw_data->Z_minus_0[0] = -59;
			raw_data->Z_minus_0[1] = 217;
			raw_data->Z_minus_0[2] = -885;

			raw_data->z_minus_0 = true;
			count++;

			Serial.println("Считана координата Z_minus_0");
			Serial.println();

        }  if ((((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 1.0f, 2.0f))) || 
        ((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z + 1.0f, 2.0f)))) && 
        !(raw_data->z_minus_180)) {
            
			get_raw_compass(ahrs_sensor);
			raw_data->Z_minus_180[0] = ahrs_sensor->raw_compass[0];
			raw_data->Z_minus_180[1] = ahrs_sensor->raw_compass[1];
			raw_data->Z_minus_180[2] = ahrs_sensor->raw_compass[2];

			raw_data->z_minus_180 = true;
			count++;

			Serial.println("Считана координата Z_minus_180");
			Serial.println();

        }  if ((((180.0f - ANGLE) < to_degrees(2.0f * acos(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 1.0f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f))) || 
        ((180.0f - ANGLE) < to_degrees(2.0f * acos(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 1.0f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f)))) && 
        !(raw_data->z_plus_0)) {
          
			get_raw_compass(ahrs_sensor);
			raw_data->Z_plus_0[0] = ahrs_sensor->raw_compass[0];
			raw_data->Z_plus_0[1] = ahrs_sensor->raw_compass[1];
			raw_data->Z_plus_0[2] = ahrs_sensor->raw_compass[2];

			raw_data->z_plus_0 = true;
			count++;

			Serial.println("Считана координата Z_plus_0");
			Serial.println();

        }  if ((((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y - 1.0f, 2.0f) + pow(q.z - 0.0f, 2.0f))) ||
        ((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y + 1.0f, 2.0f) + pow(q.z - 0.0f, 2.0f)))) && 
        !(raw_data->z_plus_180)) {
          
			get_raw_compass(ahrs_sensor);
			raw_data->Z_plus_180[0] = ahrs_sensor->raw_compass[0];
			raw_data->Z_plus_180[1] = ahrs_sensor->raw_compass[1];
			raw_data->Z_plus_180[2] = ahrs_sensor->raw_compass[2];

			raw_data->z_plus_180 = true;
			count++;

			Serial.println("Считана координата Z_plus_180");
			Serial.println();

        }  if ((((90.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (90.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.707f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f))) ||
        ((270.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (270.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 0.707f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f)))) &&
        !(raw_data->y_minus_0)) {
          
			get_raw_compass(ahrs_sensor);
			raw_data->Y_minus_0[0] = ahrs_sensor->raw_compass[0];
			raw_data->Y_minus_0[1] = ahrs_sensor->raw_compass[1];
			raw_data->Y_minus_0[2] = ahrs_sensor->raw_compass[2];

			raw_data->y_minus_0 = true;
			count++;

			Serial.println("Считана координата Y_minus_0");
			Serial.println();

        }  if ((((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y + 0.707f, 2.0f) + pow(q.z + 0.707f, 2.0f))) ||  
        ((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y - 0.707f, 2.0f) + pow(q.z - 0.707f, 2.0f)))) && 
        !(raw_data->y_minus_180)) {
            
			get_raw_compass(ahrs_sensor);
			raw_data->Y_minus_180[0] = ahrs_sensor->raw_compass[0];
			raw_data->Y_minus_180[1] = ahrs_sensor->raw_compass[1];
			raw_data->Y_minus_180[2] = ahrs_sensor->raw_compass[2];

			raw_data->y_minus_180 = true;
			count++;

			Serial.println("Считана координата Y_minus_180");
			Serial.println();

        }  if ((((90.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (90.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 0.707f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f))) ||
        ((270.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (270.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.707f, 2.0f) + pow(q.y - 0.0f, 2.0f) + pow(q.z - 0.0f, 2.0f)))) && 
        !(raw_data->y_plus_0)) {
          
			get_raw_compass(ahrs_sensor);
			raw_data->Y_plus_0[0] = ahrs_sensor->raw_compass[0];
			raw_data->Y_plus_0[1] = ahrs_sensor->raw_compass[1];
			raw_data->Y_plus_0[2] = ahrs_sensor->raw_compass[2];

			raw_data->y_plus_0 = true;
			count++;

			Serial.println("Считана координата Y_plus_0");
			Serial.println();

        }  if ((((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y + 0.707f, 2.0f) + pow(q.z - 0.707f, 2.0f))) ||
        ((180.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (180.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.0f, 2.0f) + pow(q.y - 0.707f, 2.0f) + pow(q.z + 0.707f, 2.0f)))) && 
        !(raw_data->y_plus_180)) {
			
			get_raw_compass(ahrs_sensor);
			raw_data->Y_plus_180[0] = ahrs_sensor->raw_compass[0];
			raw_data->Y_plus_180[1] = ahrs_sensor->raw_compass[1];
			raw_data->Y_plus_180[2] = ahrs_sensor->raw_compass[2];

			raw_data->y_plus_180 = true;
			count++;

			Serial.println("Считана координата Y_plus_180");
			Serial.println();

        }  if ((((120.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (120.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 0.5f, 2.0f) + pow(q.y - 0.5f, 2.0f) + pow(q.z - 0.5f, 2.0f))) ||
        ((240.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (240.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.5f, 2.0f) + pow(q.y + 0.5f, 2.0f) + pow(q.z + 0.5f, 2.0f)))) && 
        !(raw_data->x_minus_0)) {
			
			get_raw_compass(ahrs_sensor);
			raw_data->X_minus_0[0] = ahrs_sensor->raw_compass[0];
			raw_data->X_minus_0[1] = ahrs_sensor->raw_compass[1];
			raw_data->X_minus_0[2] = ahrs_sensor->raw_compass[2];

			raw_data->x_minus_0 = true;
			count++;

			Serial.println("Считана координата X_minus_0");
			Serial.println();

        }  if ((((120.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (120.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.5f, 2.0f) + pow(q.y - 0.5f, 2.0f) + pow(q.z + 0.5f, 2.0f))) ||
        ((240.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (240.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 0.5f, 2.0f) + pow(q.y + 0.5f, 2.0f) + pow(q.z - 0.5f, 2.0f)))) && 
        !(raw_data->x_minus_180)) {
			get_raw_compass(ahrs_sensor);
			raw_data->X_minus_180[0] = ahrs_sensor->raw_compass[0];
			raw_data->X_minus_180[1] = ahrs_sensor->raw_compass[1];
			raw_data->X_minus_180[2] = ahrs_sensor->raw_compass[2];

			raw_data->x_minus_180 = true;
			count++;

			Serial.println("Считана координата X_minus_180");
			Serial.println();
            
        }  if ((((120.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (120.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.5f, 2.0f) + pow(q.y + 0.5f, 2.0f) + pow(q.z - 0.5f, 2.0f))) ||
        ((240.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (240.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 0.5f, 2.0f) + pow(q.y - 0.5f, 2.0f) + pow(q.z + 0.5f, 2.0f)))) && 
        !(raw_data->x_plus_0)) {
			get_raw_compass(ahrs_sensor);
			raw_data->X_plus_0[0] = ahrs_sensor->raw_compass[0];
			raw_data->X_plus_0[1] = ahrs_sensor->raw_compass[1];
			raw_data->X_plus_0[2] = ahrs_sensor->raw_compass[2];

			raw_data->x_plus_0 = true;
			count++;

			Serial.println("Считана координата X_plus_0");
			Serial.println();

        }  if ((((120.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (120.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x + 0.5f, 2.0f) + pow(q.y + 0.5f, 2.0f) + pow(q.z + 0.5f, 2.0f))) ||
        ((240.0f - ANGLE) < to_degrees(2.0f * acosf(q.w)) && (240.0f + ANGLE) > to_degrees(2.0f * acosf(q.w)) && D > sqrt(pow(q.x - 0.5f, 2.0f) + pow(q.y - 0.5f, 2.0f) + pow(q.z - 0.5f, 2.0f)))) && 
        !(raw_data->x_plus_180)) {
			get_raw_compass(ahrs_sensor);
			raw_data->X_plus_180[0] = ahrs_sensor->raw_compass[0];
			raw_data->X_plus_180[1] = ahrs_sensor->raw_compass[1];
			raw_data->X_plus_180[2] = ahrs_sensor->raw_compass[2];

			raw_data->x_plus_180 = true;
			count++;

			Serial.println("Считана координата X_plus_180");
			Serial.println();
        }
    }
}
