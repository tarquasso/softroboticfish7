#include "BTU/btu.h"
#include "mbed.h"

#define NUM_FLOATS 4
#define TIMESTEP 0.05
// MODSERIAL pcSerial(USBTX,USBRX); //serial device

Serial pc(USBTX, USBRX);

AnalogIn pot1(p15);
AnalogIn pot2(p19);
AnalogIn pot3(p20);
DigitalOut TestLED(LED1);
bool clk = true;
BTU m_BTU = BTU();
int counter = 0;

int mode = 2; //default
float Kc = 0.0005;
float TauI = 0.00003;
float TauD = 0.000000000;
float setVal = 90;

void runLED() {
  counter = (counter + 1) % 20;
  if(counter == 0) {
    TestLED = !TestLED;
  }
  m_BTU.update(mode, setVal, Kc, TauI, TauD);
  m_BTU.runCycle();
}

int main() {
	// MODSERIAL* pcSerial = new MODSERIAL(USBTX,USBRX); //dynamic allocation of the serial device
	// pcSerial->printf("Start!\n");

	//Set up a serial communication object
    // SerialComm serialComm(pcSerial);
    pc.printf("Start!\r\n");

	//BTU m_BTU single instance is made in .h .cpp files;
	int mode = 2; //default
	float Kc = 0.0005;
	float TauI = 0.00003;
	float TauD = 0.000000000;
	float setVal = 90;
    Ticker timer;
    timer.attach(&runLED, TIMESTEP);
    TestLED = 0;

    while(1) {
        pc.printf("mode: %d, Kc: %f, TauI: %f, TauD: %f, SetVal: %.2f, CurrentVal: %2f, duty cycle: %.2f\r\n", m_BTU.m_mode, m_BTU.m_kc, m_BTU.m_taud, m_BTU.m_setval, m_BTU.m_currentval, m_BTU.m_output*100);
        wait_ms(100);
    }

}
