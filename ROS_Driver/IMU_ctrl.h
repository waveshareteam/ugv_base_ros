#define AD0_VAL 0
ICM_20948_I2C myICM;

// SimpleKalmanFilter  kf_ax(kf_accel_q, kf_accel_r, kf_accel_p);
// SimpleKalmanFilter  kf_ay(kf_accel_q, kf_accel_r, kf_accel_p);
// SimpleKalmanFilter  kf_az(kf_accel_q, kf_accel_r, kf_accel_p);

// SimpleKalmanFilter  kf_gx(kf_accel_q, kf_accel_r, kf_accel_p);
// SimpleKalmanFilter  kf_gy(kf_accel_q, kf_accel_r, kf_accel_p);
// SimpleKalmanFilter  kf_gz(kf_accel_q, kf_accel_r, kf_accel_p);

void imu_init() {
}


void updateIMUData() {
}

// {"T":127}
// reset qc0 ~ q3
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
        return;
    }

    myICM.setBiasGyroX(-store.biasGyroX);
    myICM.setBiasGyroX(-store.biasGyroY);
    myICM.setBiasGyroX(-store.biasGyroZ);
    myICM.setBiasAccelX(-store.biasAccelX);
    myICM.setBiasAccelX(-store.biasAccelY);
    myICM.setBiasAccelX(-store.biasAccelZ);
    myICM.setBiasCPassX(-store.biasCPassX);
    myICM.setBiasCPassX(-store.biasCPassY);
    myICM.setBiasCPassX(-store.biasCPassZ);

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

    qc0 = 1.0;
    qc1 = 0.0;
    qc2 = 0.0;
    qc3 = 0.0;
}

// {"T":127}
void imuCalibration_bk() {
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

// {"T":126}
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

// {"T":128}
// get and set qc0 ~ qc3 compensation quaternion
void getIMUOffset() {
    double halfRoll = -icm_roll / 2.0;
    double qr0 = cos(halfRoll);
    double qr1 = sin(halfRoll);
    double qr2 = 0.0;
    double qr3 = 0.0;

    double halfPitch = -icm_pitch / 2.0;
    double qp0 = cos(halfPitch);
    double qp1 = 0.0;
    double qp2 = sin(halfPitch);
    double qp3 = 0.0;

    qc0 = qr0 * qp0 - qr1 * qp1 - qr2 * qp2 - qr3 * qp3;
    qc1 = qr0 * qp1 + qr1 * qp0 + qr2 * qp3 - qr3 * qp2;
    qc2 = qr0 * qp2 - qr1 * qp3 + qr2 * qp0 + qr3 * qp1;
    qc3 = qr0 * qp3 + qr1 * qp2 - qr2 * qp1 + qr3 * qp0;

    jsonInfoHttp.clear();
    jsonInfoHttp["T"] = FEEDBACK_IMU_OFFSET;
    jsonInfoHttp["qc0"] = qc0;
    jsonInfoHttp["qc1"] = qc1;
    jsonInfoHttp["qc2"] = qc2;
    jsonInfoHttp["qc3"] = qc3;
    String getInfoJsonString;
    serializeJson(jsonInfoHttp, getInfoJsonString);
    Serial.println(getInfoJsonString);
}

// {"T":128}
void getIMUOffset_bk() {
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

// {"T":129,"gx":0,"gy":0,"gz":0,"ax":0,"ay":0,"az":0,"cx":0,"cy":0,"cz":0}
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