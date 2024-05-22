#define AD0_VAL 0
ICM_20948_I2C myICM;
icm_20948_DMP_data_t data;

SimpleKalmanFilter  kf_ax(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter  kf_ay(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter  kf_az(kf_accel_q, kf_accel_r, kf_accel_p);

SimpleKalmanFilter  kf_gx(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter  kf_gy(kf_accel_q, kf_accel_r, kf_accel_p);
SimpleKalmanFilter  kf_gz(kf_accel_q, kf_accel_r, kf_accel_p);

void imu_init() {
  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.println("Initialization of the sensor returned: ");
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else {
      initialized = true;
    }
  }
  Serial.println(F("Device connected!"));
  bool success = true;
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // myICM.setDMPstartAddress();
  // myICM.setDMPendAddress();
  // myICM.setDMPrate(20);

  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 1) == ICM_20948_Stat_Ok);
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  delay(2);
  ICM_20948_fss_t myFSS;
  // myFSS.a = gpm4;
  // myFSS.g = dps2000;
  myFSS.a = gpm16;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  myICM.lowPower(false);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 2) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 2) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 2) == ICM_20948_Stat_Ok);

  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  if (success) {
    Serial.println(F("DMP enabled!"));
  } else {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
  }
}


void updateIMUData() {
  myICM.readDMPdataFromFIFO(&data);
  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) {
      q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      q2sqr = q2 * q2;

      // roll (x-axis rotation)
      t0 = +2.0 * (q0 * q1 + q2 * q3);
      t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      icm_roll = atan2(t0, t1);

      // pitch (y-axis rotation)
      t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      icm_pitch = asin(t2);

      // yaw (z-axis rotation)
      t3 = +2.0 * (q0 * q3 + q1 * q2);
      t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      icm_yaw = atan2(t3, t4);
    }
  } else {
    // Serial.println("1");
  }

  if ((data.header & DMP_header_bitmap_Accel) > 0) {
    ax = kf_ax.updateEstimate((data.Raw_Accel.Data.X/2048.0));
    ay = kf_ay.updateEstimate((data.Raw_Accel.Data.Y/2048.0));
    az = kf_az.updateEstimate((data.Raw_Accel.Data.Z/2048.0));
    // ax = data.Raw_Accel.Data.X/2048.0;
    // ay = data.Raw_Accel.Data.Y/2048.0;
    // az = data.Raw_Accel.Data.Z/2048.0;
  }
  if ((data.header & DMP_header_bitmap_Gyro) > 0) {
    gx = kf_gx.updateEstimate((data.Raw_Gyro.Data.X/16.4) * PI / 180.0);
    gy = kf_gy.updateEstimate((data.Raw_Gyro.Data.Y/16.4) * PI / 180.0);
    gz = kf_gz.updateEstimate((data.Raw_Gyro.Data.Z/16.4) * PI / 180.0);
    // gx = (data.Raw_Gyro.Data.X/16.4) * PI / 180.0;
    // gy = (data.Raw_Gyro.Data.Y/16.4) * PI / 180.0;
    // gz = (data.Raw_Gyro.Data.Z/16.4) * PI / 180.0;
  }
  // delay(10);
}


void imuCalibration() {
  
}


void getIMUData() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_IMU_DATA;

  jsonInfoHttp["r"] = icm_roll;
  jsonInfoHttp["p"] = icm_pitch;
  jsonInfoHttp["y"] = icm_yaw;

  jsonInfoHttp["q0"] = q0;
  jsonInfoHttp["q1"] = q1;
  jsonInfoHttp["q2"] = q2;
  jsonInfoHttp["q3"] = q3;

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}


void getIMUOffset() {

}


void setIMUOffset(int16_t inputX, int16_t inputY, int16_t inputZ) {

}