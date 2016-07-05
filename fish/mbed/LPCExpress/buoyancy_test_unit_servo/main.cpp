#include "BTU/btu.h"
#include "mbed.h"

#define NUM_FLOATS 5
#define TIMESTEP 1.0

#include "MODSERIAL.h"
#include "SerialComm.h"

// MODSERIAL pcSerial(USBTX,USBRX); //serial device
AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);



bool clk = true;
BTU m_BTU = BTU();
int counter = 0;

int mode = 3; //default
float Kc = 1.0;
float TauI = 0.0;
float TauD = 0.0;
float setVal = 0.0; //meters

void runControl() {
  counter = (counter + 1) % 20;
  if(counter == 0) {
    TestLED = !TestLED;
  }

	setVal = pot1;
	float a1 = -91; 		// scale for range in [a,b]
	float b1 = 91;
	setVal = (b1-a1)*setVal+a1;
  m_BTU.update(mode, setVal, Kc, TauI, TauD);
  m_BTU.runCycle();
}

int main() {
	MODSERIAL* pcSerial = new MODSERIAL(USBTX,USBRX); //dynamic allocation of the serial device
	pcSerial->printf("Start!\n");

	//Set up a serial communication object
    SerialComm serialComm(pcSerial);

	//BTU m_BTU single instance is made in .h .cpp files;
    m_BTU.init(TIMESTEP);
    pcSerial->printf("pressure at start: %.6f\r\n",m_BTU.m_pvDepth);
    Ticker timer;
    timer.attach(&runControl, TIMESTEP);
    TestLED = 0;
    float valueFloats[NUM_FLOATS];

	while (1) {
<<<<<<< 4f84e71cb2f70c35fa422ec84eb98b78e8124533
		wait(0.5);
		pcSerial->printf(
				"m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, cm:%.2f, pm:%.2f, de:%.4f, er:%.4f\r\n",
				m_BTU.m_mode, m_BTU.m_kc, m_BTU.m_taui, m_BTU.m_taud,
				m_BTU.m_setval, m_BTU.m_currentval, m_BTU.m_cmdPosDeg,
				m_BTU.m_pvDepthMeters, m_BTU.m_setval - m_BTU.m_pvDepthMeters,
				m_BTU.m_errorPIDUnScaled);
=======
//		pcSerial->printf("m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f, cm:%.2f, pm:%.2f, de:%.4f\r\n",
//				m_BTU.m_mode, m_BTU.m_kc, m_BTU.m_taui, m_BTU.m_taud, m_BTU.m_setval,
//				m_BTU.m_currentval, m_BTU.m_cmdPosDeg, m_BTU.m_pvDepthMeters, m_BTU.m_setval-m_BTU.m_pvDepthMeters);

		pcSerial->printf("m:%d, set:%.2f, current:%.2f, Volt:%.2f, DepthM:%.2f, de:%.4f\r\n",
				m_BTU.m_mode, m_BTU.m_setval,
				m_BTU.m_currentval, m_BTU.m_cmdVoltage, m_BTU.m_pvDepthMeters, m_BTU.m_setval-m_BTU.m_currentval);
>>>>>>> f87177676e31eee27daee6849151cb42de328a39
		if (serialComm.checkIfNewMessage()) {

			serialComm.getFloats(valueFloats, NUM_FLOATS);
			pcSerial->printf("New Values\n");
			// printing out for debugging
//			for (int i = 0; i < NUM_FLOATS; i++) {
//				pcSerial->printf("Value#%d: %f\n", i, valueFloats[i]);
//			}


			mode = (int) valueFloats[0];
			Kc = valueFloats[1];
			TauI = valueFloats[2];
			TauD = valueFloats[3];
			//setVal = valueFloats[4];
		}
		wait_ms(1000);
		TestLED2 = !TestLED2;
	}
}
