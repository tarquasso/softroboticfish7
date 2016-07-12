#include "BTU_Actuate/btu_PID_lite.cpp"
#include "mbed.h"

#define NUM_FLOATS 5
#define TIMESTEP 0.05


#include "MODSERIAL.h"
#include "SerialComm.h"

MODSERIAL pcSerial(USBTX, USBRX);
//AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);

// bool clk = true;
BTU btu = BTU();
int counter = 0;
int mode = 2;
float Kc = 1.0;
float TauI = 0.0;
float TauD = 0.0;
float setVal = 0.0;             // meters

void runControl() {
  counter = (counter + 1) % 20;
  if(!counter) {
    TestLED = !TestLED;
  }

  //setVal = pot1;
  //setVal = (b1-a1)*setVal+a1;

  if(mode == VELOCITY_CTRL_MODE) {
      btu.updateMode(mode);
      btu.updateVelTunings(Kc, TauI, TauD);
  } else if (mode == SPEC_POSITION_CTRL_MODE){
	  btu.updateMode(mode);
	  btu.updatePosTunings(Kc, TauI, TauD);
  } else {
      btu.update(mode, Kc, TauI, TauD);
  }
  btu.updateAndRunCycle(mode, setVal);
}

int main() {
  pcSerial.printf("Start!\n");

  SerialComm serialComm(&pcSerial);

  btu.init();
  pcSerial.printf("pressure at start: %.6f\r\n", btu.getPressure());
  Ticker timer;
  timer.attach(&runControl, TIMESTEP);
  TestLED = 0;
  float valueFloats[NUM_FLOATS];

  while(1) {
	  float depth = 0;
      //float depth = btu.getDepth();
      if (mode == DEPTH_CTRL_MODE) {
    	  pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, de:%.2f, depth_er:%.4f\r\n",
                      btu.getMode(), btu.getKc(), btu.getTauI(), btu.getTauD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - depth);
      } else if (mode == SPEC_POSITION_CTRL_MODE) {
    	  pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, pos_er:%.4f %.4f\r\n",
    	                        btu.getMode(), btu.getPKc(), btu.getPTauI(), btu.getPTauD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - btu.getActPosition(ACT_A), setVal - btu.getActPosition(ACT_B));
      } else {
    	  pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, de:%.2f\r\n", btu.getMode(), Kc, TauI, TauD, setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth);
      }

      if(serialComm.checkIfNewMessage()) {
          serialComm.getFloats(valueFloats, NUM_FLOATS);

          mode = (int) valueFloats[0];
          Kc = valueFloats[1];
          TauI = valueFloats[2];
          TauD = valueFloats[3];
          setVal = valueFloats[4];
      }
      wait_ms(500);
      TestLED2 = !TestLED2;
  }
}
