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

  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 3) == ICM_20948_Stat_Ok);
  myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Continuous);
  delay(2);
  ICM_20948_fss_t myFSS;
  // myFSS.a = gpm4;
  // myFSS.g = dps2000;
  myFSS.a = gpm16;   // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
  myFSS.g = dps2000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  myICM.lowPower(false);

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 3) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 3) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 3) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 3) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 3) == ICM_20948_Stat_Ok);

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
  }
  if ((data.header & DMP_header_bitmap_Gyro) > 0) {
    gx = kf_gx.updateEstimate((data.Raw_Gyro.Data.X/16.4) * PI / 180.0);
    gy = kf_gy.updateEstimate((data.Raw_Gyro.Data.Y/16.4) * PI / 180.0);
    gz = kf_gz.updateEstimate((data.Raw_Gyro.Data.Z/16.4) * PI / 180.0);
  }
  if ((data.header & DMP_header_bitmap_Compass) > 0) {
    mx = data.Compass.Data.X;
    my = data.Compass.Data.Y;
    mz = data.Compass.Data.Z;
  }
}


// {"T":127}
void imuCalibration() {
  bool bias_success  = (myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

  if (!bias_success) {
    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

    jsonInfoHttp["status"] = 0;

    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
    return;
  }

  myICM.setBiasGyroX(store.biasGyroX);
  myICM.setBiasGyroY(store.biasGyroY);
  myICM.setBiasGyroZ(store.biasGyroZ);
  myICM.setBiasAccelX(store.biasAccelX);
  myICM.setBiasAccelY(store.biasAccelY);
  myICM.setBiasAccelZ(store.biasAccelZ);
  myICM.setBiasCPassX(store.biasCPassX);
  myICM.setBiasCPassY(store.biasCPassY);
  myICM.setBiasCPassZ(store.biasCPassZ);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["status"] = 1;

  jsonInfoHttp["gx"] = store.biasGyroX;
  jsonInfoHttp["gy"] = store.biasGyroY;
  jsonInfoHttp["gz"] = store.biasGyroZ;

  jsonInfoHttp["ax"] = store.biasAccelX;
  jsonInfoHttp["ay"] = store.biasAccelY;
  jsonInfoHttp["az"] = store.biasAccelZ;

  jsonInfoHttp["cx"] = store.biasCPassX;
  jsonInfoHttp["cy"] = store.biasCPassY;
  jsonInfoHttp["cz"] = store.biasCPassZ;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
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
  bool bias_success = (myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
       bias_success &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

  if (!bias_success) {
    return;
  }

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["gx"] = store.biasGyroX;
  jsonInfoHttp["gy"] = store.biasGyroY;
  jsonInfoHttp["gz"] = store.biasGyroZ;

  jsonInfoHttp["ax"] = store.biasAccelX;
  jsonInfoHttp["ay"] = store.biasAccelY;
  jsonInfoHttp["az"] = store.biasAccelZ;

  jsonInfoHttp["cx"] = store.biasCPassX;
  jsonInfoHttp["cy"] = store.biasCPassY;
  jsonInfoHttp["cz"] = store.biasCPassZ;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}


void setIMUOffset(int32_t inGX, int32_t inGY, int32_t inGZ, int32_t inAX, int32_t inAY, int32_t inAZ, int32_t inCX, int32_t inCY, int32_t inCZ) {
  store.biasGyroX = inGX;
  store.biasGyroY = inGY;
  store.biasGyroZ = inGZ;

  store.biasAccelX = inAX;
  store.biasAccelY = inAY;
  store.biasAccelZ = inAZ;

  store.biasCPassX = inCX;
  store.biasCPassY = inCY;
  store.biasCPassZ = inCZ;

  myICM.setBiasGyroX(store.biasGyroX);
  myICM.setBiasGyroX(store.biasGyroY);
  myICM.setBiasGyroX(store.biasGyroZ);
  myICM.setBiasAccelX(store.biasAccelX);
  myICM.setBiasAccelX(store.biasAccelY);
  myICM.setBiasAccelX(store.biasAccelZ);
  myICM.setBiasCPassX(store.biasCPassX);
  myICM.setBiasCPassX(store.biasCPassY);
  myICM.setBiasCPassX(store.biasCPassZ);

  jsonInfoHttp.clear();
  jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;

  jsonInfoHttp["status"] = 1;

  jsonInfoHttp["gx"] = store.biasGyroX;
  jsonInfoHttp["gy"] = store.biasGyroY;
  jsonInfoHttp["gz"] = store.biasGyroZ;

  jsonInfoHttp["ax"] = store.biasAccelX;
  jsonInfoHttp["ay"] = store.biasAccelY;
  jsonInfoHttp["az"] = store.biasAccelZ;

  jsonInfoHttp["cx"] = store.biasCPassX;
  jsonInfoHttp["cy"] = store.biasCPassY;
  jsonInfoHttp["cz"] = store.biasCPassZ;

  String getInfoJsonString;
  serializeJson(jsonInfoHttp, getInfoJsonString);
  Serial.println(getInfoJsonString);
}