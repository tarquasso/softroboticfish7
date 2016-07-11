#include "BTU/btu_PID_lite.cpp"
#include "mbed.h"

#define NUM_FLOATS 4
#define TIMESTEP 0.05
#define DEPTH_THRESHOLD 0.1
#define MISSION_TIMEOUT 10.0
#define ERROR_THRESHOLD 0.15
#define SUCCESS_TIME 5.0
#define UNWOUND_POS 91.0


#include "MODSERIAL.h"
#include "SerialComm.h"

MODSERIAL pcSerial(USBTX, USBRX);
// AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);
DigitalOut inMission(LED3);
DigitalOut missionSuccess(LED4);

// LocalFileSystem local("local");

// bool clk = true;
BTU btu = BTU();
int counter = 0;
int mode = 2;
float Kc = 1.0;
float TauI = 0.0;
float TauD = 0.0;
float setVal = 0.0;             // meters
Ticker Mission;
float timeout = 0.0;
float successTime = 0.0;
int missionDepth = 0.0;
bool missionStarted = false;


void terminateMission() {
    Mission.detach();
    counter = 0;
    inMission = 0;
    timeout = 0.0;
    successTime = 0.0;
    missionStarted = false;
    btu.updateAndRunCycle(POSITION_CTRL_MODE, UNWOUND_POS);
}

bool checkThreshold() {
    float error = btu.getDepth() - missionDepth;
    // float error = btu.getServoPos() - missionDepth;
    float absError = (error > 0) ? error : (-1 * error);
    return (absError <= ERROR_THRESHOLD);
}

void runMission() {
    counter = (counter + 1) % 20;
    if(!counter) {
        TestLED = !TestLED;
    }
    if(btu.getDepth() >= DEPTH_THRESHOLD) {
        inMission = 1;
        missionStarted = true;
    }
    if(!missionStarted) {
        return;
    }
    btu.runCycle(missionDepth);
    successTime += ((int) checkThreshold() * TIMESTEP);
    if (successTime >= SUCCESS_TIME) {
        missionSuccess = 1;
        terminateMission();
        return;
    }
    if (timeout >= MISSION_TIMEOUT) {
        missionSuccess = 0;
        terminateMission();
        return;
    }
    timeout += TIMESTEP;
}

void startMission(float kc, float taui, float taud, float setDepth) {
    terminateMission();
    missionSuccess = 0;
    missionDepth = setDepth;
    btu.update(DEPTH_CTRL_MODE, kc, taui, taud);
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
      if(serialComm.checkIfNewMessage()) {
          serialComm.getFloats(valueFloats, NUM_FLOATS);

          // mode = (int) valueFloats[0];
          Kc = valueFloats[0];
          TauI = valueFloats[1];
          TauD = valueFloats[2];
          setVal = valueFloats[3];

          startMission(Kc, TauI, TauD, setVal);
      }
      wait_ms(500);
      TestLED2 = !TestLED2;
  }
}
