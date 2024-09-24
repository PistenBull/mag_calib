#include "lib_ahrs.h"

#define VECTOR_DEVIATION 10.0 // 10%
#define button_pin 5 // PULL_UP

ADXL345 accel;
ITG3200 gyro;
HMC5883L mag;

AHRS_sensor ahrs_sensor;
Raw_data raw_data;
Quaternion quaternion;

void setup() {
  Serial.begin(1000000);
  
  pinMode(button_pin, INPUT_PULLUP);

  Serial.println("Initializing I2C devices...");
  accel.initialize();
  gyro.initialize();
  mag.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
  Serial.println(gyro.testConnection() ? "ITG3200 connection successful" : "ITG3200 connection failed");
  Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");
  delay(500);

  EEPROM.get(OFFSETS_ADDR, ahrs_sensor.offsets);
  
  float old_vector = ahrs_sensor.offsets.compass_vector; 

  Serial.println("Click on the button to start mems calibration ...");
  while(!(digitalRead(button_pin) == LOW)) {}
  delay(500); 

  Serial.print("old compass vector: "); Serial.println(String(ahrs_sensor.offsets.compass_vector, 4));
  
  IMU_calibration(&ahrs_sensor);

  float new_vector = ahrs_sensor.offsets.compass_vector;

  if ((fabs(new_vector - old_vector) / old_vector) * 100 <= VECTOR_DEVIATION) {
    Serial.println("Перекалибровка не потребовалась ...");
    ahrs_sensor.offsets.compass_vector = new_vector;
  }
  else {
    Serial.println("Считывание данных для калибровки магнитометра (перекалибровка) ...");
    reading_raw_compass_for_calibration(&raw_data, &ahrs_sensor);

    Serial.println("Click on the button to start compass calibration ...");
    while(!(digitalRead(button_pin) == LOW)) {}
    delay(500);

    compass_calibration(&raw_data, &ahrs_sensor);
  }

  Serial.println("Click on the button to get data ...");  
  while(!(digitalRead(button_pin) == LOW)) {}
  delay(500);
}


void loop() {
	get_scaled_AHRS(&ahrs_sensor);
	AHRS_quaternion_update(&ahrs_sensor);
  //IMU_quaternion_update(&ahrs_sensor);

  Serial.print(get_heading());
  Serial.print(",");
  Serial.print(get_pitch());
  Serial.print(",");
	Serial.println(get_roll());
}
