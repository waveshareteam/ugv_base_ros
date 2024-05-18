#define DEG2ANG 57.29577951307855

Adafruit_ICM20948 icm;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t mag;
sensors_event_t temp;

SimpleKalmanFilter	kf_ax(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter	kf_ay(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter	kf_az(kf_accel_q, kf_accel_r, kf_accel_p);

SimpleKalmanFilter	kf_mx(0.5, 1, 0.1);
SimpleKalmanFilter	kf_my(0.5, 1, 0.1);
SimpleKalmanFilter	kf_mz(0.5, 1, 0.1);

SimpleKalmanFilter	kf_gx(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter	kf_gy(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter	kf_gz(kf_accel_q, kf_accel_r, kf_accel_p);


void imu_init() {
  Serial.println("Adafruit ICM20948 test!");

  // Try to initialize!
  if (!icm.begin_I2C(0x68)) {
    Serial.println("Failed to find ICM20948 chip");
    return;
  }
  Serial.println("ICM20948 Found!");
  icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
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

  icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
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

	ax = - kf_ax.updateEstimate(accel.acceleration.x) + ax_offset;
	if (abs(ax) < accel_h) {
		ax = 0;
	}
	ay = - kf_ay.updateEstimate(accel.acceleration.y) + ay_offset;
	if (abs(ay) < accel_h) {
		ay = 0;
	}
	az = - kf_az.updateEstimate(accel.acceleration.z) + az_offset;
	if (abs(az) < accel_h * 6) {
		az = 0;
	}

	mx = kf_mx.updateEstimate(mag.magnetic.x);
	my = kf_my.updateEstimate(mag.magnetic.y);
	mz = kf_mz.updateEstimate(mag.magnetic.z);

	gx = kf_gx.updateEstimate(gyro.gyro.x) - gx_offset;
	if (abs(gx) < gyro_h) {
		gx = 0;
	}
	gy = kf_gy.updateEstimate(gyro.gyro.y) - gy_offset;
	if (abs(gy) < gyro_h) {
		gy = 0;
	}
	gz = kf_gz.updateEstimate(gyro.gyro.z) - gz_offset;
	if (abs(gz) < gyro_h) {
		gz = 0;
	}
}


void imuUpdate_threading( void * parameter) {
	while (true) {
		updateIMUData();
		delay(1);
	}
}


void imuCalibration() {
	for (int i=0;i<sample_count;i++){
		icm.getEvent(&accel, &gyro, &temp, &mag);

		gx_offset += kf_gx.updateEstimate(gyro.gyro.x);
		gy_offset += kf_gy.updateEstimate(gyro.gyro.y);
		gz_offset += kf_gz.updateEstimate(gyro.gyro.z);

		ax_offset += kf_ax.updateEstimate(accel.acceleration.x);
		ay_offset += kf_ay.updateEstimate(accel.acceleration.y);
		az_offset += kf_az.updateEstimate(accel.acceleration.z);

		delay(10);
	}
	gx_offset = gx_offset / sample_count;
	gy_offset = gy_offset / sample_count;
	gz_offset = gz_offset / sample_count;

	ax_offset = ax_offset / sample_count;
	ay_offset = ay_offset / sample_count;
	az_offset = az_offset / sample_count;
  
}


void getIMUData() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

	jsonInfoHttp["temp"] = temp.temperature;

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}


void getIMUOffset() {

}


void setIMUOffset(int16_t inputX, int16_t inputY, int16_t inputZ) {
	// setOffset(inputX, inputY, inputZ);
}