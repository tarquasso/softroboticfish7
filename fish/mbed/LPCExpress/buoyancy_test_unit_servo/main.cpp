#include "BTU/btu.h"
#include "mbed.h"

#define NUM_FLOATS 5
#define TIMESTEP 0.05

#include "MODSERIAL.h"
#include "SerialComm.h"

// MODSERIAL pcSerial(USBTX,USBRX); //serial device

AnalogIn pot1(p15);
AnalogIn pot2(p19);
AnalogIn pot3(p20);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);

bool clk = true;
BTU m_BTU = BTU();
int counter = 0;

int mode = 2; //default
float Kc = 0.0005;
float TauI = 0.00003;
float TauD = 0.000000000;
float setVal = 0.885;

void runLED() {
  counter = (counter + 1) % 20;
  if(counter == 0) {
    TestLED = !TestLED;
  }
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

    Ticker timer;
    timer.attach(&runLED, TIMESTEP);
    TestLED = 0;
    float valueFloats[NUM_FLOATS];

	while (1) {
		pcSerial->printf("m:%d, Kc:%f, TI:%f, TD:%f, S:%.2f, C:%.2f, d: %.2f, pM:%.3f\r\n",
				m_BTU.m_mode, m_BTU.m_kc, m_BTU.m_taui, m_BTU.m_taud, m_BTU.m_setval,
				m_BTU.m_currentval, m_BTU.m_cmdPosDeg, m_BTU.m_pvDepthMeters);
		if (serialComm.checkIfNewMessage()) {

			serialComm.getFloats(valueFloats, NUM_FLOATS);
			// printing out for debugging
			for (int i = 0; i < NUM_FLOATS; i++) {
				pcSerial->printf("Value#%d: %f\n", i, valueFloats[i]);
			}

			mode = (int) valueFloats[0];
			Kc = valueFloats[1];
			TauI = valueFloats[2];
			TauD = valueFloats[3];
			setVal = valueFloats[4];
		}
		wait_ms(1000);
		TestLED2 = !TestLED2;
	}

}
