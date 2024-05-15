#define DEG2ANG 57.29577951307855

Adafruit_ICM20948 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

// double icm_pitch;
// double icm_roll;
// double icm_yaw;
// float  icm_temp;
// unsigned long last_imu_update = 0;




// SimpleKalmanFilter	kf_ax(0.5, 1, 0.1);
// SimpleKalmanFilter	kf_ay(0.5, 1, 0.1);
// SimpleKalmanFilter	kf_az(0.5, 1, 0.1);

// SimpleKalmanFilter	kf_mx(0.5, 1, 0.1);
// SimpleKalmanFilter	kf_my(0.5, 1, 0.1);
// SimpleKalmanFilter	kf_mz(0.5, 1, 0.1);

// SimpleKalmanFilter	kf_gx(0.5, 1, 0.1);
// SimpleKalmanFilter	kf_gy(0.5, 1, 0.1);
// SimpleKalmanFilter	kf_gz(0.5, 1, 0.1);





// SimpleKalmanFilter	kf_yaw(1, 1, 0.01);

// double qw;
// double qx;
// double qy;
// double qz;

// double ax;
// double ay;
// double az;

// double mx;
// double my;
// double mz;

// double gx;
// double gy;
// double gz;

void imu_init() {
  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C(0x68)) {
    // if (!icm.begin_SPI(ICM_CS)) {
    // if (!icm.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {

    Serial.println("Failed to find ICM20948 chip");
    // while (1) {
    //   delay(10);
    // }
    return;
  }
  Serial.println("ICM20948 Found!");
  // icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm.getAccelRange()) {
  case ICM20948_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case ICM20948_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20948_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20948_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  Serial.println("OK");

  // icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm.getGyroRange()) {
  case ICM20948_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20948_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  }

  //  icm.setAccelRateDivisor(4095);
  uint16_t accel_divisor = icm.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  //  icm.setGyroRateDivisor(255);
  uint8_t gyro_divisor = icm.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);

  // icm.setMagDataRate(AK09916_MAG_DATARATE_10_HZ);
  Serial.print("Magnetometer data rate set to: ");
  switch (icm.getMagDataRate()) {
  case AK09916_MAG_DATARATE_SHUTDOWN:
    Serial.println("Shutdown");
    break;
  case AK09916_MAG_DATARATE_SINGLE:
    Serial.println("Single/One shot");
    break;
  case AK09916_MAG_DATARATE_10_HZ:
    Serial.println("10 Hz");
    break;
  case AK09916_MAG_DATARATE_20_HZ:
    Serial.println("20 Hz");
    break;
  case AK09916_MAG_DATARATE_50_HZ:
    Serial.println("50 Hz");
    break;
  case AK09916_MAG_DATARATE_100_HZ:
    Serial.println("100 Hz");
    break;
  }
  Serial.println();
}


void updateIMUData() {
	icm.getEvent(&accel, &gyro, &temp, &mag);

	ax = accel.acceleration.x;
	ay = accel.acceleration.y;
	az = accel.acceleration.z;

	mx = mag.magnetic.x;
	my = mag.magnetic.y;
	mz = mag.magnetic.z;

	gx = gyro.gyro.x;
	gy = gyro.gyro.y;
	gz = gyro.gyro.z;

	double dt = (millis() - last_imu_update) / 1000.0;
	last_imu_update = millis();

	icm_pitch = icm_pitch - gy * dt;
	icm_roll = icm_roll + gx * dt;
	icm_yaw = icm_yaw + gz * dt;
}


void imuUpdate_threading( void * parameter) {
	while (true) {
		updateIMUData();
		delay(1);
	}
}


void imuCalibration() {
	// jsonInfoHttp.clear();
	// switch (inputStep) {
	// case 0:
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "keep 10dof-imu device horizontal and then calibrate next step.";
	// 	}
	// 	break;
	// case 1:
	// 	calibrateStepA();
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "rotate z axis 180 degrees and then calibrate next step.";
	// 	}
	// 	break;
	// case 2:
	// 	calibrateStepB();
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "flip 10dof-imu device and keep it horizontal and then calibrate next step.";
	// 	}
	// 	break;
	// case 3:
	// 	calibrateStepC();
	// 	if (InfoPrint == 1){
	// 		jsonInfoHttp["info"] = "calibration done.";
	// 	}
	// 	break;
	// }
	// String getInfoJsonString;
	// serializeJson(jsonInfoHttp, getInfoJsonString);
	// Serial.println(getInfoJsonString);
}


void getIMUData() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

	jsonInfoHttp["r"] = icm_roll;
	jsonInfoHttp["p"] = icm_pitch;
	// jsonInfoHttp["y"] = icm_yaw;

	// jsonInfoHttp["q0"] = qw;
	// jsonInfoHttp["q1"] = qx;
	// jsonInfoHttp["q2"] = qy;
	// jsonInfoHttp["q3"] = qz;

	jsonInfoHttp["ax"] = ax;
	jsonInfoHttp["ay"] = ay;
	jsonInfoHttp["az"] = az;

	jsonInfoHttp["gx"] = gx;
	jsonInfoHttp["gy"] = gy;
	jsonInfoHttp["gz"] = gz;

	jsonInfoHttp["mx"] = mx;
	jsonInfoHttp["my"] = my;
	jsonInfoHttp["mz"] = mz;

	jsonInfoHttp["temp"] = temp.temperature;

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}

void getIMUOffset() {
	// getIMUOffsetData(&offsetData);
	// jsonInfoHttp.clear();
	// jsonInfoHttp["T"] = 101;

	// jsonInfoHttp["x"] = offsetData.X;
	// jsonInfoHttp["y"] = offsetData.Y;
	// jsonInfoHttp["z"] = offsetData.Z;

	// String getInfoJsonString;
	// serializeJson(jsonInfoHttp, getInfoJsonString);
	// Serial.println(getInfoJsonString);
}

void setIMUOffset(int16_t inputX, int16_t inputY, int16_t inputZ) {
	// setOffset(inputX, inputY, inputZ);
}