#include <utility.h>
#include "mbed.h"
#include "BtuLinear.h"

#define NUM_FLOATS 5
#define TIMESTEP 0.05
#define DEPTH_THRESHOLD 0.3
#define MIN_MISSION_DEPTH 0.2
#define MISSION_TIMEOUT 60.0
#define ERROR_THRESHOLD 0.125
#define SUCCESS_TIME 8.0
#define DEBRIEF_TIME_LIMIT 20.0
#define UNWOUND_POS 1.0

#define DEF_V_KC 0.5
#define DEF_V_TAUI 0
#define DEF_V_TAUD 0

#define DEF_P_KC 1
#define DEF_P_TAUI 0
#define DEF_P_TAUD 0


#include "MODSERIAL/MODSERIAL.h"
#include "SerialComm/SerialComm.h"

MODSERIAL pcSerial(USBTX, USBRX);
// AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);
DigitalOut inMission(LED3);
DigitalOut missionSuccess(LED4);

LocalFileSystem local("local");

// bool clk = true;
BtuLinear btu = BtuLinear();
int counter = 0;
int mode = 2;
float Kc = 1.0;
float TauI = 0.0;
float TauD = 0.0;
float floorVal;
float jumpVal;
float missionFloor;
float missionStep;
float setVal = 0.0;             // meters
Ticker Mission;
float timeout = 0.0;
float missionTime = 0.0;
float successTime = 0.0;
float missionDepth = 0.0;
bool missionStarted = false;
float debriefTime = 0.0;
bool debriefMode = false;
FILE *fp;
bool returnTrip = false;

void terminateMission() {
    Mission.detach();
    fclose(fp);
    counter = 0;
    TestLED = 0;
    inMission = 0;
    timeout = 0.0;
    successTime = 0.0;
    missionTime = 0.0;
    missionStarted = false;
    debriefMode = false;
    debriefTime = 0.0;
    returnTrip = false;

    btu.updateAndRunCycle(VELOCITY_CTRL_MODE, UNWOUND_POS);
}

bool checkThreshold() {
    float error = btu.getDepth() - missionDepth;
    // float error = btu.getServoPos() - missionDepth;
    float absError = (error > 0) ? error : (-1 * error);
    return (absError <= ERROR_THRESHOLD);
}

void runMission() {
	if(debriefMode) {
		inMission = 0;
		missionDepth = 0;
		btu.updateAndRunCycle(VELOCITY_CTRL_MODE, UNWOUND_POS);
		if(debriefTime >= DEBRIEF_TIME_LIMIT) {
			terminateMission();
		}
		debriefTime += TIMESTEP;
		return;
	}
    counter = (counter + 1) % 20;
    if(!counter) {
        TestLED = !TestLED;
    }
    if(btu.getDepth() >= DEPTH_THRESHOLD) {
        inMission = 1;
        missionStarted = true;
        btu.updateMode(DEPTH_CTRL_MODE);
    }
    if(!missionStarted) {
        return;
    }
    btu.runCycle(missionDepth);
    if(checkThreshold()) {
    	successTime += TIMESTEP;
    } else {
    	successTime = 0.0;
    }
    if (successTime >= SUCCESS_TIME) {
        if((missionDepth <= MIN_MISSION_DEPTH) && (returnTrip == true)) {
            missionSuccess = 1;
            debriefMode = true;
            return;
        } else if(missionDepth == missionFloor) {
            returnTrip = true;
            successTime = 0.0;
            missionDepth = utility::clip(missionDepth - missionStep, MIN_MISSION_DEPTH, missionFloor);
            timeout = 0.0;
        } else {
        	successTime = 0.0;
            if(returnTrip) {
                missionDepth = utility::clip(missionDepth - missionStep, MIN_MISSION_DEPTH, missionFloor);
            } else {
                missionDepth = utility::clip(missionDepth + missionStep, MIN_MISSION_DEPTH, missionFloor);
            }
            timeout = 0.0;
        }
    }
    if (timeout >= MISSION_TIMEOUT) {
        missionSuccess = 0;
        debriefMode = true;
        return;
    }
    missionTime += TIMESTEP;
    timeout += TIMESTEP;
}

void startMission(float kc, float taui, float taud, float setDepth, float stepDist) {
    terminateMission();
    fp = fopen("/local/log", "w");
    fprintf(fp, "MISSION START, TARGET: %.2f, STEP DISTANCE: %.2f\r\n", setDepth, stepDist);

    missionSuccess = 0;
    missionFloor = setDepth;
    missionDepth = utility::clip(stepDist, MIN_MISSION_DEPTH, missionFloor);
    missionStep = stepDist;
    returnTrip = false;
    btu.updateDepthTunings(kc, taui, taud);
    btu.updateVelTunings(DEF_V_KC, DEF_V_TAUI, DEF_V_TAUD);
    btu.updatePosTunings(DEF_P_KC, DEF_P_TAUI, DEF_P_TAUD);
    // btu.update(SPEC_POSITION_CTRL_MODE, kc, taui, taud);
    Mission.attach(&runMission, TIMESTEP);
}


int main() {
  pcSerial.printf("Start!\n");

  SerialComm serialComm(&pcSerial);

  btu.init();
  pcSerial.printf("pressure at start: %.6f\r\n", btu.getPressure());
  TestLED = 0;
  float valueFloats[NUM_FLOATS];

  while(1) {
      // depth = btu.getDepth();
      // if (mode == DEPTH_CTRL_MODE) {
      //     pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, de:%.2f, depth_er:%.4f\r\n",
      //                 btu.getMode(), btu.getKc(), btu.getTauI(), btu.getTauD(), setVal, btu.getServoPos(), depth, setVal - depth);
      // } else if (mode == SPEC_POSITION_CTRL_MODE) {
      //     pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, de:%.2f, pos_er:%.4f\r\n",
      //                           btu.getMode(), btu.getKc(), btu.getTauI(), btu.getTauD(), setVal, btu.getServoPos(), depth, setVal - btu.getServoPos());
      // } else {
      //     pcSerial.printf("m:%d, s:%.2f, cu:%.2f, de:%.2f\r\n", btu.getMode(), setVal, btu.getServoPos(), depth);
      // }
      // pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, pos_er:%.4f, th:%d, to:%.2f, st:%.2f\r\n", btu.getMode(), btu.getKc(), btu.getTauI(), btu.getTauD(), setVal, btu.getServoPos(), setVal - btu.getServoPos(), checkThreshold(), timeout, successTime);
      if(inMission || debriefMode) {
          float depth = btu.getDepth();
          fprintf(fp, "m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, depth_er:%.4f, time: %.2f, to:%.2f, rt:%d\r\n",
                  btu.getMode(), btu.getKc(), btu.getTauI(), btu.getTauD(), missionDepth, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, missionDepth - depth, missionTime, timeout, returnTrip);
      } else {
    	  btu.updateAndRunCycle(VELOCITY_CTRL_MODE, UNWOUND_POS);
      }
      //float depth = btu.getDepth();
      //pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, de:%.2f, depth_er:%.4f, time: %.2f, to:%.2f\r\n",
      //                  btu.getMode(), btu.getKc(), btu.getTauI(), btu.getTauD(), missionDepth, btu.getServoPos(), depth, missionDepth - depth, missionTime, timeout);
      if(serialComm.checkIfNewMessage()) {
          serialComm.getFloats(valueFloats, NUM_FLOATS);

          // mode = (int) valueFloats[0];
          Kc = valueFloats[0];
          TauI = valueFloats[1];
          TauD = valueFloats[2];
          jumpVal = valueFloats[3];
          floorVal = valueFloats[4];
          startMission(Kc, TauI, TauD, floorVal, jumpVal);
      }
      wait_ms(500);
      TestLED2 = !TestLED2;
  }
}
