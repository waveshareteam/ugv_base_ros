#include <ArduinoJson.h>
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<256> jsonInfoSend;
StaticJsonDocument<1024> jsonInfoHttp;

#include <SCServo.h>
#include <nvs_flash.h>
#include <esp_system.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_now.h>
#include <nvs_flash.h>
#include <Adafruit_SSD1306.h>
#include <INA219_WE.h>
#include <ESP32Encoder.h>
#include <PID_v2.h>
#include <SimpleKalmanFilter.h>
#include <math.h>
#include "ICM_20948.h"

// functions for barrery info.
#include "battery_ctrl.h"

// config for ugv.
#include "ugv_config.h"

// functions for oled.
#include "oled_ctrl.h"

// functions for the leds of UGV.
#include "ugv_led_ctrl.h"

// functions for RoArm-M2 ctrl.
#include "RoArm-M2_module.h"

// functions for gimbal ctrl.
#include "gimbal_module.h"

// define json cmd.
#include "json_cmd.h"

// functions for IMU ctrl.
#include "IMU_ctrl.h"

// functions for movtion ctrl. 
#include "movtion_module.h"

// functions for editing the files in flash.
#include "files_ctrl.h"

// advance functions for ugv ctrl.
#include "ugv_advance.h"

// functions for wifi ctrl.
#include "wifi_ctrl.h"

// functions for esp-now.
#include "esp_now_ctrl.h"

// functions for uart json ctrl.
#include "uart_ctrl.h"

// functions for http & web server.
#include "http_server.h"


void moduleType_RoArmM2() {
  unsigned long curr_time = millis();
  if (curr_time - prev_time >= 10){
    constantHandle();
    prev_time = curr_time;
  }

  RoArmM2_getPosByServoFeedback();
  
  // esp-now flow ctrl as a flow-leader.
  switch(espNowMode) {
  case 1: espNowGroupDevsFlowCtrl();break;
  case 2: espNowSingleDevFlowCtrl();break;
  }

  if (InfoPrint == 2) {
    RoArmM2_infoFeedback();
  }
}


void moduleType_Gimbal() {
  getGimbalFeedback();
  gimbalSteady(steadyGoalY);
}


void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Wire.begin(S_SDA, S_SCL, 400000);

  bool initialized = false;
  while (!initialized) {
    myICM.begin(Wire, AD0_VAL);
    Serial.print(F("Initialization of the sensor returned: "));
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

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  // +
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.

  // success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 3) == ICM_20948_Stat_Ok); // Set to the maximum

  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 3) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 3) == ICM_20948_Stat_Ok); // Set to the maximum

  // +
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 3) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 3) == ICM_20948_Stat_Ok);

  myICM.lowPower(false);

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
  }

  ina219_init();
  inaDataUpdate();

  // set mainType & moduleType.
  // mainType: 1.RaspRover, 2.UGV Rover, 3.UGV Beast
  // moduleType: 0.Null, 1.RoArm, 2.PT
  mm_settings(mainType, moduleType);

  init_oled();
  screenLine_0 = "ugv_base_ros.git";
  screenLine_1 = "version: 0.93";
  screenLine_2 = "starting...";
  screenLine_3 = "";
  oled_update();

  delay(1200);

  // functions for IMU.
  // imu_init();

  // functions for the leds on ugv.
  led_pin_init();

  // init the littleFS funcs in files_ctrl.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize LittleFS";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize LittleFS for Flash files ctrl.");}
  initFS();

  // init the funcs in switch_module.h
  screenLine_2 = screenLine_3;
  screenLine_3 = "Initialize 12V-switch ctrl";
  oled_update();
  if(InfoPrint == 1){Serial.println("Initialize the pins used for 12V-switch ctrl.");}
  movtionPinInit();

  // servos power up
  screenLine_2 = screenLine_3;
  screenLine_3 = "Power up the servos";
  oled_update();
  if(InfoPrint == 1){Serial.println("Power up the servos.");}
  delay(500);
  
  // init servo ctrl functions.
  screenLine_2 = screenLine_3;
  screenLine_3 = "ServoCtrl init UART2TTL...";
  oled_update();
  if(InfoPrint == 1){Serial.println("ServoCtrl init UART2TTL...");}
  RoArmM2_servoInit();

  // check the status of the servos.
  screenLine_2 = screenLine_3;
  screenLine_3 = "Bus servos status check...";
  oled_update();
  if(InfoPrint == 1){Serial.println("Bus servos status check...");}
  RoArmM2_initCheck(false);

  if(InfoPrint == 1 && RoArmM2_initCheckSucceed){
    Serial.println("All bus servos status checked.");
  }
  if(RoArmM2_initCheckSucceed) {
    screenLine_2 = "Bus servos: succeed";
  } else {
    screenLine_2 = "Bus servos: " + 
    servoFeedback[BASE_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].status +
    servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].status +
    servoFeedback[ELBOW_SERVO_ID - 11].status +
    servoFeedback[GRIPPER_SERVO_ID - 11].status;
  }
  screenLine_3 = ">>> Moving to init pos...";
  oled_update();
  RoArmM2_resetPID();
  RoArmM2_moveInit();

  screenLine_3 = "Reset joint torque to ST_TORQUE_MAX";
  oled_update();
  if(InfoPrint == 1){Serial.println("Reset joint torque to ST_TORQUE_MAX.");}
  RoArmM2_dynamicAdaptation(0, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX, ST_TORQUE_MAX);

  screenLine_3 = "WiFi init";
  oled_update();
  if(InfoPrint == 1){Serial.println("WiFi init.");}
  initWifi();

  screenLine_3 = "http & web init";
  oled_update();
  if(InfoPrint == 1){Serial.println("http & web init.");}
  initHttpWebServer();

  screenLine_3 = "ESP-NOW init";
  oled_update();
  if(InfoPrint == 1){Serial.println("ESP-NOW init.");}
  initEspNow();

  screenLine_3 = "IMU Calibrating";
  oled_update();
  if(InfoPrint == 1){Serial.println("IMU Calibrating");}
  imuCalibration();

  screenLine_3 = "UGV started";
  oled_update();
  if(InfoPrint == 1){Serial.println("UGV started.");}

  getThisDevMacAddress();

  updateOledWifiInfo();

  initEncoders();

  pidControllerInit();

  screenLine_2 = String("MAC:") + macToString(thisDevMac);
  oled_update();

  led_pwm_ctrl(0, 0);

  if(InfoPrint == 1){Serial.println("Application initialization settings.");}
  createMission("boot", "these cmds run automatically at boot.");
  missionPlay("boot", 1);
}


void loop() {
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    // if ((data.header & DMP_header_bitmap_Quat9) > 0) {
    //   // Scale to +/- 1
    //   q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    //   q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    //   q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
    //   q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

    //   q2sqr = q2 * q2;

    //   // roll (x-axis rotation)
    //   t0 = +2.0 * (q0 * q1 + q2 * q3);
    //   t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
    //   icm_roll = atan2(t0, t1);

    //   // pitch (y-axis rotation)
    //   t2 = +2.0 * (q0 * q2 - q3 * q1);
    //   t2 = t2 > 1.0 ? 1.0 : t2;
    //   t2 = t2 < -1.0 ? -1.0 : t2;
    //   icm_pitch = asin(t2);

    //   // yaw (z-axis rotation)
    //   t3 = +2.0 * (q0 * q3 + q1 * q2);
    //   t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
    //   icm_yaw = atan2(t3, t4);

    //   // Serial.print(F("r:"));
    //   // Serial.print(icm_roll, 2);
    //   // Serial.print(F(" p:"));
    //   // Serial.print(icm_pitch, 2);
    //   // Serial.print(F(" y:"));
    //   // Serial.println(icm_yaw, 2);
    // }

    if ((data.header & DMP_header_bitmap_Accel) > 0) {
      ax = data.Raw_Accel.Data.X;
      ay = data.Raw_Accel.Data.Y;
      az = data.Raw_Accel.Data.Z;
    }
    if ((data.header & DMP_header_bitmap_Gyro) > 0) {
      gx = data.Raw_Gyro.Data.X;
      gy = data.Raw_Gyro.Data.Y;
      gz = data.Raw_Gyro.Data.Z;
    }
    if ((data.header & DMP_header_bitmap_Compass) > 0) {
      mx = data.Compass.Data.X;
      my = data.Compass.Data.Y;
      mz = data.Compass.Data.Z;
    }
  }

  serialCtrl();
  server.handleClient();

  // read and compute the info of joints.
  switch (moduleType) {
  case 1: moduleType_RoArmM2();break;
  case 2: moduleType_Gimbal();break;
  }

  // recv esp-now json cmd.
  if(runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }

  getLeftSpeed();

  LeftPidControllerCompute();
  
  getRightSpeed();
  
  RightPidControllerCompute();
  
  oledInfoUpdate();

  if (baseFeedbackFlow) {
    baseInfoFeedback();
  }

  heartBeatCtrl();
}