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
//DigitalOut valveSideLED(LED2);
Ticker run_timer;

float freq = 0;
float yaw = 0;
float thrust = 0;

void run_func() {
	pumpWithValve.set(freq, yaw, thrust);
	pc.printf("Valve side: %d\n", pumpWithValve.getVside());
	pc.printf("Valve Vset: %f\n\n", pumpWithValve.getVset());
//	valveSideLED = pumpWithValve.getVside();
}

int main() {
	pc.baud(19200);
	pumpWithValve.start();
	int input_freq;
	int input_yaw;
//	int input_thrust;

	run_timer.attach(&run_func, 0.05);

	while (1) {
		// inputs are unsigned ints from 0 to 255
//		pc.scanf("%d %d %d", &input_freq, &input_yaw, &input_thrust);
		pc.scanf("%d %d", &input_freq, &input_yaw); // if just testing freq & yaw (valve)
		//pc.scanf("%d %d %d", &input_freq); // if just testing hall sensor interrupt
		
		// figure out why these interpolation functions are different in actual FishController
		//freq = (input_freq * (fishMaxFrequency - fishMinFrequency) / 255.0) + fishMinFrequency;
		freq = input_freq/256.0;
		yaw = (input_yaw * (fishMaxYaw - fishMinYaw) / 256.0) + fishMinYaw;
//		thrust = (input_thrust * (fishMaxThrust - fishMinThrust) / 255.0) + fishMinThrust;
	}	
}
