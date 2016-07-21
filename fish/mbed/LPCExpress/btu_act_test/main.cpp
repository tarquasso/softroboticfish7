#include "mbed.h"
#include "BtuLinear.h"

#define NUM_FLOATS 5
#define FREQUENCY 20
#define TIMESTEP (1.0/FREQUENCY)
#define PRINTF_WAIT_MS 500

#define SERIAL_MESSAGE_NUM_MODE 0
#define SERIAL_MESSAGE_NUM_KC 1
#define SERIAL_MESSAGE_NUM_TAUI 2
#define SERIAL_MESSAGE_NUM_TAUD 3
#define SERIAL_MESSAGE_NUM_SETVAL 4

#define DRY_RUN 1

#include "MODSERIAL.h"
#include "SerialComm.h"

MODSERIAL pcSerial(USBTX, USBRX);
//AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);

// bool clk = true;
BtuLinear btu = BtuLinear(DRY_RUN);
int counter = 0;
int mode = 2;
float Kc = 1.0;
float TauI = 0.0;
float TauD = 0.0;
float setVal = 0.0;             // meters

void runControl() {
  counter = (counter + 1) % FREQUENCY;
  if(!counter) {
	// blink every second
    TestLED = !TestLED;
  }

  //setVal = pot1;
  //setVal = (b1-a1)*setVal+a1;

  // Based on Mode, update Tunings
  if(mode == VELOCITY_CTRL_MODE) {
      btu.updateVelTunings(Kc, TauI, TauD);
  } else if (mode == POSITION_CTRL_MODE){
	  btu.updatePosTunings(Kc, TauI, TauD);
  } else {
      btu.updateDepthTunings(Kc, TauI, TauD);
  }
  // Update Mode and Run Cycle
  btu.updateAndRunCycle(mode, setVal);
}

int main() {
  pcSerial.printf("Start!\n");

  SerialComm serialComm(&pcSerial);

  btu.init();
  pcSerial.printf("pressure at start: %.6f\r\n", btu.getPressure());

  // start test leds at off
  TestLED = 0;
  TestLED2 = 0;

  // attach control ticker every time step
  Ticker controlTicker;
  controlTicker.attach(&runControl, TIMESTEP);

  float valueFloats[NUM_FLOATS];

  while(1) {
	  float depth = 0;
      depth = btu.getDepth();
	  if (mode == DEPTH_CTRL_MODE) {
    	  pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, depth_er:%.4f\r\n",
                      btu.getMode(), btu.getDKc(), btu.getDkI(), btu.getDkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - depth);
      } else if (mode == POSITION_CTRL_MODE) {
    	  pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, pos_er:%.4f %.4f\r\n",
    	                        btu.getMode(), btu.getPKc(), btu.getPkI(), btu.getPkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - btu.getActPosition(ACT_A), setVal - btu.getActPosition(ACT_B));
      } else {
    	  pcSerial.printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f %.2f, de:%.2f\r\n", btu.getMode(), Kc, TauI, TauD, setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth);
      }

      if(serialComm.checkIfNewMessage()) {
          serialComm.getFloats(valueFloats, NUM_FLOATS);

          mode = (int) valueFloats[SERIAL_MESSAGE_NUM_MODE];
          Kc = valueFloats[SERIAL_MESSAGE_NUM_KC];
          TauI = valueFloats[SERIAL_MESSAGE_NUM_TAUI];
          TauD = valueFloats[SERIAL_MESSAGE_NUM_TAUD];
          setVal = valueFloats[SERIAL_MESSAGE_NUM_SETVAL];
      }
      wait_ms(PRINTF_WAIT_MS);
      TestLED2 = !TestLED2;
  }
}
