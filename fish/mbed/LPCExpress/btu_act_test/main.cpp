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

#define DRY_RUN 0

#include "MODSERIAL.h"
#include "SerialComm.h"

MODSERIAL pcSerial(USBTX, USBRX);
//AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);

// bool clk = true;
BtuLinear btu = BtuLinear(DRY_RUN);
int counter = 0;
float setVal = 0.4;             // meters

void runControl() {
  counter = (counter + 1) % FREQUENCY;
  if(!counter) {
	// blink every second
    TestLED = !TestLED;
  }

  //setVal = pot1;
  //setVal = (b1-a1)*setVal+a1;

  // Update Mode and Run Cycle
  btu.runCycle(setVal);
}

int main() {
  pcSerial.printf("Start!\n");

  SerialComm serialComm(&pcSerial);

  btu.init();
  //float DkC, PkC, VkC;
  //float DkI, PkI, VkI;
  //float DkD, PkD, VkD;
  float kC, kI, kD;
  int mode = btu.getMode();
  if(mode == VELOCITY_CTRL_MODE) {
	  kC = btu.getVkC(); kI = btu.getVkI(); kD = btu.getVkD();
  } else if (mode == POSITION_CTRL_MODE){
	  kC = btu.getPkC(); kI = btu.getPkI(); kD = btu.getPkD();
  } else {
	  kC = btu.getDkC(); kI = btu.getDkI(); kD = btu.getDkD();
  }

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
    	  pcSerial.printf("m:%d, kc:%f, ki:%f, kd:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, depth_er:%.4f\r\n",
                      btu.getMode(), btu.getDkC(), btu.getDkI(), btu.getDkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - depth);
      } else if (mode == POSITION_CTRL_MODE) {
    	  pcSerial.printf("m:%d, kc:%f, ki:%f, kd:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, pos_er:%.4f %.4f\r\n",
    	              btu.getMode(), btu.getPkC(), btu.getPkI(), btu.getPkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - btu.getActPosition(ACT_A), setVal - btu.getActPosition(ACT_B));
      } else {
    	  pcSerial.printf("m:%d, kc:%f, ki:%f, kd:%f, s:%.2f, cu:%.2f %.2f, de:%.2f\r\n",
    			  	  btu.getMode(), btu.getVkC(), btu.getVkI(), btu.getVkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth);
      }

      if(serialComm.checkIfNewMessage()) {
          serialComm.getFloats(valueFloats, NUM_FLOATS);

          mode = (int) valueFloats[SERIAL_MESSAGE_NUM_MODE];
          kC = valueFloats[SERIAL_MESSAGE_NUM_KC];
          kI = valueFloats[SERIAL_MESSAGE_NUM_TAUI];
          kD = valueFloats[SERIAL_MESSAGE_NUM_TAUD];
          setVal = valueFloats[SERIAL_MESSAGE_NUM_SETVAL];

          // Based on Mode, update Tunings
          if(mode == VELOCITY_CTRL_MODE) {
              btu.updateVelTunings(kC, kI, kD);
          } else if (mode == POSITION_CTRL_MODE){
        	  btu.updatePosTunings(kC, kI, kD);
          } else {
              btu.updateDepthTunings(kC, kI, kD);
          }

          btu.updateMode(mode);

      }
      wait_ms(PRINTF_WAIT_MS);
      TestLED2 = !TestLED2;
  }
}
