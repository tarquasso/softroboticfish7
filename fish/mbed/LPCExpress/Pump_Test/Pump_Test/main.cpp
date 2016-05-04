// test file for PumpWithValve class functions
// NOTE look at the below h files to define whether that control mode is enabled

#include "mbed.h"
#include "PumpWithValve.h"
//#include "QEI.h"

#define fishMinYaw       ((float)(-1.0))
#define fishMaxYaw       ((float)(1.0))
#define fishMinThrust    ((float)(0.0))
#define fishMaxThrust    ((float)(1.0))
#define fishMinFrequency ((float)(0.0000009))
#define fishMaxFrequency ((float)(0.0000016))

Serial pc(USBTX, USBRX);
DigitalOut valveSideLED(LED2);
DigitalOut valveRunningLED(LED3);
Ticker run_timer;
PwmOut bcu(p22);

float freq = 0;
float yaw = 0;
float thrust = 0;

void run_func() {
	pumpWithValve.set(freq, yaw, thrust);

	pc.printf("Valve Vset: %f\t Time since hall sensor: %f\n", pumpWithValve.getVset(), pumpWithValve.getSensorTime());
	pc.printf("Set Freq: %f\t Cur Freq: %f\n", pumpWithValve.getSetFreq(), pumpWithValve.getCurFreq());
	pc.printf("Period 1: %d\t Period 2: %d\n\n", pumpWithValve.getPeriod1(), pumpWithValve.getPeriod2());
	valveSideLED = pumpWithValve.getVside();
	valveRunningLED = pumpWithValve.getRunState();
}

int main() {
	pc.baud(19200);
	bcu = 0.0;
	int input_freq;
	int input_yaw;
//	int input_thrust;

	freq = fishMinFrequency;
	yaw = 0;

	pc.printf("starting up...\n\n");

	while (1) {
		// inputs are unsigned ints from 0 to 255
//		pc.scanf("%d %d %d", &input_freq, &input_yaw, &input_thrust);
		pc.scanf("%d %d", &input_freq, &input_yaw); // if just testing freq & yaw (valve)
		//pc.scanf("%d %d %d", &input_freq); // if just testing hall sensor interrupt

		if(input_freq == 999) {
			pc.printf("\nstopping system\n");
			pumpWithValve.stop();
			run_timer.detach();
			valveRunningLED = pumpWithValve.getRunState();
		} else if(input_freq == 888){
			pc.printf("\nstarting system\n");
			pumpWithValve.start();
			run_timer.attach(&run_func, 0.2);
		} else if(0 <= input_freq && input_freq <= 255){
			// figure out why these interpolation functions are different in actual FishController
			freq = (input_freq * (fishMaxFrequency - fishMinFrequency) / 255.0) + fishMinFrequency;
			//freq = input_freq/256.0;
			yaw = (input_yaw * (fishMaxYaw - fishMinYaw) / 255.0) + fishMinYaw;
	//		thrust = (input_thrust * (fishMaxThrust - fishMinThrust) / 255.0) + fishMinThrust;
		}
	}	
}
