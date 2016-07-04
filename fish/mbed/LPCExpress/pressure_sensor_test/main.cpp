
#include "mbed.h"
#include "MS5837.h"
//#include "MODSERIAL.h"

#define PIN_IMU_SDA p28
#define PIN_IMU_SCL p27

#define TIMESTEP 0.3

Serial pcSerial(USBTX,USBRX); //serial device
Timer t;

MS5837 pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL);

DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);

bool clk = true;

int counter = 0;
float pressure;

void runControl() {
  counter = (counter + 1) % 3;
  if(counter == 0) {
    TestLED = !TestLED;
  }

}

int main() {


	pcSerial.printf("Start!\n");
	pressureSensor.MS5837Init();

	//Set up a serial communication object
    //SerialComm serialComm(pcSerial);

	//BTU m_BTU single instance is made in .h .cpp files;
    //m_BTU.init(TIMESTEP);

    Ticker timer;
    timer.attach(&runControl, TIMESTEP);
    TestLED = 0;
    int count = 1;
    float tAvgSum = 0.0;
    float tAvg = 0.0;
    float tNow = 0.0;
	pressureSensor.MS5837Start();
	t.start();

	while (1) {
		if (pressureSensor.MS5837Done()) {
			pressure = pressureSensor.MS5837_Pressure();

			tNow = t.read();
			tAvgSum = (tAvgSum + tNow);
			tAvg = tAvgSum / (float) count;
			t.reset();
			count++;
			if (count % 50 == 1) {
				printf("t_now:%f t_avg:%f\n", tNow, tAvg);
				pcSerial.printf("pressure:%.3f\r\n", pressure);
				TestLED2 = !TestLED2;
			}

		}

	}

}
