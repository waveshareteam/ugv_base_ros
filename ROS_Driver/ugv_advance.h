// advance funcs for RoArm-M2 ctrl
// place holder.
void jsonCmdReceiveHandler();
bool moveToStep(String inputName, int inputStepNum);


// mission abort after serial received anything.
bool serialMissionAbort() {
	if (Serial.available()) {
		if (InfoPrint == 1) {Serial.println("[missionPlay abort.]");}
		return true;
	} else {
		return false;
	}
}


// input the mission name and the intro to create a mission file.
bool createMission(String inputName, String inputIntro) {
	jsonInfoSend.clear();
	jsonInfoSend["name"] = inputName;
	jsonInfoSend["intro"] = inputIntro;

	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	
	jsonInfoSend.clear();
	return createFile(inputName + ".mission", contentBuffer);
}


// input the mission name and get the total content
int missionContent(String inputName) {
	File file = LittleFS.open("/" + inputName + ".mission", "r");
	if (!file) {
		Serial.println("file not found.");
		return -1;
	}

	Serial.println("---=== File Content ===---");
	Serial.println("reading file: [" + inputName + "] starts:\n");
	String mission_intro = file.readStringUntil('\n');
	Serial.println(mission_intro);
	
	jsonInfoHttp.clear();
	jsonInfoHttp["info"] = "reading mission.";
	jsonInfoHttp["first_line"] = mission_intro;

	int _LineNum = 0;
	while (file.available()) {
		_LineNum++;
		String line = file.readStringUntil('\n');
		Serial.print("[StepNum: ");Serial.print(_LineNum);Serial.print(" ] - ");
		Serial.println(line);

		jsonInfoHttp["StepNum_"+String(_LineNum)] = line;
	}

	Serial.println("^^^ ^^^ ^^^ reading file: " + inputName + ".mission ends. ^^^ ^^^ ^^^");
	file.close();

	return _LineNum;
}




// input the mission name and the step to append 
// a new step at the end of the mission.
// using inputStep(String)
bool appendStepJson(String inputName, String inputStep) {
	DeserializationError err = deserializeJson(jsonInfoSend, inputStep);
	if (err == DeserializationError::Ok) {
		if (InfoPrint == 1) {
			Serial.println("[json parsing succeed.]");
		}
		appendLine(inputName + ".mission", inputStep);
		jsonInfoSend.clear();
		return true;
	} else {
		jsonInfoSend.clear();
		if (InfoPrint == 1) {
			Serial.println("[deserializeJson err]");
		}
		return false;
	}
}

// input the mission name and the step to append
// a new step at the end of the mission.
// using feedback.
void appendStepFB(String inputName, float inputSpd) {
	RoArmM2_infoFeedback();
	jsonInfoSend.clear();
	jsonInfoSend["T"] = 104;
	jsonInfoSend["x"] = lastX;
	jsonInfoSend["y"] = lastY;
	jsonInfoSend["z"] = lastZ;
	jsonInfoSend["t"] = lastT;
	jsonInfoSend["spd"] = inputSpd;
	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	appendLine(inputName + ".mission", contentBuffer);
}

// append a new delay(ms) at the end of the mission.
void appendDelayCmd(String inputName, int delayTime) {
	jsonInfoSend.clear();
	jsonInfoSend["T"] = 111;
	jsonInfoSend["cmd"] = delayTime;
	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	appendLine(inputName + ".mission", contentBuffer);
}




// insert a new step as the stepNum
// using the json string input.
bool insertStepJson(String inputName, int inputStepNum, String inputStep) {
	DeserializationError err = deserializeJson(jsonInfoSend, inputStep);
	if (err == DeserializationError::Ok) {
		if (InfoPrint == 1) {
			Serial.println("[json parsing succeed.]");
		}
		insertLine(inputName + ".mission", inputStepNum + 1, inputStep);
		jsonInfoSend.clear();
		return true;
	} else {
		jsonInfoSend.clear();
		if (InfoPrint == 1) {
			Serial.println("[deserializeJson err]");
		}
		return false;
	}
}

// insert a new step as the stepNum
// using the feedback.
void insertStepFB(String inputName, int inputStepNum, float inputSpd) {
	RoArmM2_infoFeedback();
	jsonInfoSend.clear();
	jsonInfoSend["T"] = 104;
	jsonInfoSend["x"] = lastX;
	jsonInfoSend["y"] = lastY;
	jsonInfoSend["z"] = lastZ;
	jsonInfoSend["t"] = lastT;
	jsonInfoSend["spd"] = inputSpd;
	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	insertLine(inputName + ".mission", inputStepNum + 1, contentBuffer);
}

// insert a new delayCmd as the stepNum
void insertDelayCmd(String inputName, int inputStepNum, int delayTime) {
	jsonInfoSend.clear();
	jsonInfoSend["T"] = 111;
	jsonInfoSend["cmd"] = delayTime;
	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	insertLine(inputName + ".mission", inputStepNum + 1, contentBuffer);
}




// replace the cmd at stepNum.
// using the step json string input.
bool replaceStepJson(String inputName, int inputStepNum, String inputStep) {
	DeserializationError err = deserializeJson(jsonInfoSend, inputStep);
	if (err == DeserializationError::Ok) {
		if (InfoPrint == 1) {
			Serial.println("[json parsing succeed.]");
		}
		replaceLine(inputName + ".mission", inputStepNum + 1, inputStep);
		jsonInfoSend.clear();
		return true;
	} else {
		jsonInfoSend.clear();
		if (InfoPrint == 1) {
			Serial.println("[deserializeJson err]");
		}
		return false;
	}
}

// replace the cmd at stepNum.
// using feedback.
void replaceStepFB(String inputName, int inputStepNum, float inputSpd) {
	RoArmM2_infoFeedback();
	jsonInfoSend.clear();
	jsonInfoSend["T"] = 104;
	jsonInfoSend["x"] = lastX;
	jsonInfoSend["y"] = lastY;
	jsonInfoSend["z"] = lastZ;
	jsonInfoSend["t"] = lastT;
	jsonInfoSend["spd"] = inputSpd;
	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	replaceLine(inputName + ".mission", inputStepNum + 1, contentBuffer);
}

// replace the cmd at stepNum with delay cmd.
void replaceDelayCmd(String inputName, int inputStepNum, int delayTime) {
	jsonInfoSend.clear();
	jsonInfoSend["T"] = 111;
	jsonInfoSend["cmd"] = delayTime;
	String contentBuffer;
	serializeJson(jsonInfoSend, contentBuffer);
	replaceLine(inputName + ".mission", inputStepNum + 1, contentBuffer);
}


// delete a step
void deleteStep(String inputName, int inputStepNum) {
	deleteSingleLine(inputName + ".mission", inputStepNum + 1);
}


// input the mission name and the stepNum.
// it will process the cmd.
bool moveToStep(String inputName, int inputStepNum) {
	String stepStringBuffer = readSingleLine(inputName + ".mission", inputStepNum + 1);
	DeserializationError err = deserializeJson(jsonCmdReceive, stepStringBuffer);
	if (err == DeserializationError::Ok) {
		if (InfoPrint == 1) {
			Serial.println("[json parsing succeed.]");
			Serial.println("[import a step]");
			Serial.print("[mission name]: ");Serial.println(inputName);
			Serial.print("[stepNum]: ");Serial.println(inputStepNum);
			Serial.print("[cmd]: ");Serial.println(stepStringBuffer);
		}
		jsonCmdReceiveHandler();
		if (InfoPrint == 1) {
			Serial.println("[step finished]");
		}
		jsonInfoSend.clear();
		return true;
	} else {
		jsonInfoSend.clear();
		if (InfoPrint == 1) {
			Serial.println("[deserializeJson err]");
		}
		return false;
	}
}


// input the mission name and the repeat times.
// when repeatTimes = -1, it will loop forever.
// play a mission file.
void missionPlay(String inputName, int repeatTimes) {
	int _LineNum = missionContent(inputName);
	int currentTimes = 0;
	while (1) {
		currentTimes++;
		if (currentTimes > repeatTimes && repeatTimes != -1) {
			if (InfoPrint == 1) {Serial.println("[missionPlay finished.]");}
			return;
		}
		if (InfoPrint == 1) {
			Serial.print("---\n[currentTimes: ");Serial.print(currentTimes);
			Serial.println(" ]");
		}

		for (int i = 1; i<=_LineNum; i++) {
			if (serialMissionAbort()) {
				return;
			}
			moveToStep(inputName, i);
		}
	}
}


// change EEmode.
void configEEmodeType(byte inputMode) {
	EEMode = inputMode;
	if (inputMode == 0){
		l3A = ARM_L3_LENGTH_MM_A_0;
		l3B = ARM_L3_LENGTH_MM_B_0;
		l3  = sqrt(l3A * l3A + l3B * l3B);
		t3rad = atan2(l3B, l3A);

		initX = l3A + l2B;
		initY = 0;
		initZ = l2A - l3B;
		initT = M_PI;
	}
	else if (inputMode == 1){
		l3A = ARM_L3_LENGTH_MM_A_1;
		l3B = ARM_L3_LENGTH_MM_B_1;
		l3  = sqrt(l3A * l3A + l3B * l3B);
		t3rad = atan2(l3B, l3A);

		EoAT_A = EoAT_A;
		EoAT_B = EoAT_B;
		l4A = ARM_L4_LENGTH_MM_A;
		l4B = ARM_L4_LENGTH_MM_B;
		lEA = EoAT_A + ARM_L4_LENGTH_MM_A;
		lEB = EoAT_B + ARM_L4_LENGTH_MM_B;
		lE  = sqrt(lEA * lEA + lEB * lEB);
		tErad = atan2(lEB, lEA);

		initX = l3A + l2B + l4A + EoAT_A;
		initY = 0;
		initZ = l2A - l3B - l4B - EoAT_B;
		initT = M_PI;
	}
	goalX = initX;
	goalY = initY;
	goalZ = initZ;
	goalT = initT;

	lastX = goalX;
	lastY = goalY;
	lastZ = goalZ;
	lastT = goalT;
	RoArmM2_baseCoordinateCtrl(initX, initY, initZ, initT);
	RoArmM2_goalPosMove();
}


// config the siza of EoAT.
void configEoAT(byte mountPos, double inputEA, double inputEB) {
	switch (mountPos) {
	case 0: ARM_L4_LENGTH_MM_A = 67.85;break;
	case 1: ARM_L4_LENGTH_MM_A = 64.16;break;
	case 2: ARM_L4_LENGTH_MM_A = 59.07;break;
	case 3: ARM_L4_LENGTH_MM_A = 51.07;break;
	}

	EoAT_A = inputEA;
	EoAT_B = inputEB;

	l4A = ARM_L4_LENGTH_MM_A;
	l4B = ARM_L4_LENGTH_MM_B;
	lEA = EoAT_A + ARM_L4_LENGTH_MM_A;
	lEB = EoAT_B + ARM_L4_LENGTH_MM_B;
	lE  = sqrt(lEA * lEA + lEB * lEB);
	tErad = atan2(lEB, lEA);

	initX = l3A + l2B + l4A + EoAT_A;
	initY = 0;
	initZ = l2A - l3B - l4B - EoAT_B;
	initT = M_PI;
}


// set the InfoPrint.
void configInfoPrint(byte inputCmd) {
	switch (inputCmd) {
	case 0: InfoPrint = 0;
			break;
	case 1: InfoPrint = 1;
			break;
	case 2: InfoPrint = 2;
			break;
	}
}


// set the baseInfoFeedback.
void setBaseInfoFeedbackMode(bool inputCmd) {
	if (inputCmd == 1) {
		baseFeedbackFlow = 1;
	} else if (inputCmd == 0) {
		baseFeedbackFlow = 0;
	}
}


// baseInfoFeedback.
void baseInfoFeedback() {
	static unsigned long last_feedback_time;
	if (millis() - last_feedback_time < feedbackFlowExtraDelay) {
		return;
	}
	
	last_feedback_time = millis();

	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = FEEDBACK_BASE_INFO;

	jsonInfoHttp["L"] = speedGetA;
	jsonInfoHttp["R"] = speedGetB;

	jsonInfoHttp["r"] = icm_roll;
	jsonInfoHttp["p"] = icm_pitch;
	jsonInfoHttp["y"] = icm_yaw;

	jsonInfoHttp["q0"] = q0;
	jsonInfoHttp["q1"] = q1;
	jsonInfoHttp["q2"] = q2;
	jsonInfoHttp["q3"] = q3;

	jsonInfoHttp["ax"] = ax;
	jsonInfoHttp["ay"] = ay;
	jsonInfoHttp["az"] = az;

	jsonInfoHttp["gx"] = gx;
	jsonInfoHttp["gy"] = gy;
	jsonInfoHttp["gz"] = gz;

	jsonInfoHttp["odl"] = en_odom_l;
	jsonInfoHttp["odr"] = en_odom_r;

	jsonInfoHttp["v"] = loadVoltage_V;

	switch(moduleType) {
	case 1:
		jsonInfoHttp["ax"] = lastX;
		jsonInfoHttp["ay"] = lastY;
		jsonInfoHttp["az"] = lastZ;
		jsonInfoHttp["ab"] = radB;
		jsonInfoHttp["as"] = radS;
		jsonInfoHttp["ae"] = radE;
		jsonInfoHttp["at"] = lastT;
		jsonInfoHttp["torB"] = servoFeedback[BASE_SERVO_ID - 11].load;
		jsonInfoHttp["torS"] = servoFeedback[SHOULDER_DRIVING_SERVO_ID - 11].load - servoFeedback[SHOULDER_DRIVEN_SERVO_ID - 11].load;
		jsonInfoHttp["torE"] = servoFeedback[ELBOW_SERVO_ID - 11].load;
		jsonInfoHttp["torH"] = servoFeedback[GRIPPER_SERVO_ID - 11].load;
		break;
	case 2:
		jsonInfoHttp["pan"]  = panAngleCompute(gimbalFeedback[0].pos);
		jsonInfoHttp["tilt"] = tiltAngleCompute(gimbalFeedback[1].pos);
		break;
	}

	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	Serial.println(getInfoJsonString);
}


// change module type.
void changeModuleType(byte inputCmd) {
	moduleType = inputCmd;
}


void setFeedbackFlowInterval(int inputCmd) {
	feedbackFlowExtraDelay = abs(inputCmd);
}


void setCmdEcho(bool inputCmd) {
	uartCmdEcho = inputCmd;
}


void saveSpdRate() {
	jsonInfoHttp.clear();
	jsonInfoHttp["T"] = CMD_SET_SPD_RATE;
	jsonInfoHttp["L"] = spd_rate_A;
	jsonInfoHttp["R"] = spd_rate_B;
	String getInfoJsonString;
	serializeJson(jsonInfoHttp, getInfoJsonString);
	appendStepJson("boot", getInfoJsonString);
}


// check the main & module type.
void saveMainTypeModuleTpye(byte inputMain, byte inputModule) {
	int _LineNum = missionContent("boot");
	bool sameAsSaved = false;
	int mm_line_num = -1;
	String stepStringBuffer;
	for (int i = 1; i<=_LineNum+1; i++) {
		stepStringBuffer = readSingleLine("boot.mission", i);
		DeserializationError err = deserializeJson(jsonCmdReceive, stepStringBuffer);
		if (err == DeserializationError::Ok) {
			int cmdType = jsonCmdReceive["T"].as<int>();
			if (cmdType == CMD_MM_TYPE_SET) {
				mm_line_num = i;
				int jsonMain = jsonCmdReceive["main"];
				int jsonModule = jsonCmdReceive["module"];
				if (inputMain == jsonMain && inputModule == jsonModule) {
					sameAsSaved = true;
				}
			}
		}
	}
	if (!sameAsSaved) {
		jsonInfoSend.clear();
		jsonInfoSend["T"] = CMD_MM_TYPE_SET;
		jsonInfoSend["main"] = inputMain;
		jsonInfoSend["module"] = inputModule;
		String contentBuffer;
		serializeJson(jsonInfoSend, contentBuffer);
		if (mm_line_num == -1) {
			appendStepJson("boot", contentBuffer);
			Serial.println("new mm_json appended.");
		} else {
			replaceStepJson("boot", mm_line_num-1, contentBuffer);
			Serial.println("new mm_json replaced.");
		}
	} else {
		Serial.println("same mm_json already saved.");
	}
}