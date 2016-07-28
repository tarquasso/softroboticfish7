// test file for CyclicActuator class functions
// NOTE look at the below h files to define whether that control mode is enabled

#include "mbed.h"
#include "CyclicActuator.h"

#define fishMinYaw       ((float)(-1.0))
#define fishMaxYaw       ((float)(1.0))
#define fishMinThrust    ((float)(1.0))
#define fishMaxThrust    ((float)(2.0))
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
	cyclicActuator.set(freq, yaw, thrust);

	pc.printf("Valve Vset: %f\t Time since hall sensor: %f\n", cyclicActuator.getVset(), cyclicActuator.getSensorTime());
	pc.printf("Set Freq: %f\t Cur Freq: %f\n", cyclicActuator.getSetFreq(), cyclicActuator.getCurFreq());
	pc.printf("Period 1: %d\t Period 2: %d\n\n", cyclicActuator.getPeriod1(), cyclicActuator.getPeriod2());
	pc.printf("Valve PWM: %f\t Pump PWM: %f\n\n", cyclicActuator.getValvePwm(), cyclicActuator.getPumpPwm());
	valveSideLED = cyclicActuator.getVside();
	valveRunningLED = cyclicActuator.getRunState();
}

int main() {
	//pc.baud(19200);
	bcu = 0.0;
	int input_freq;
	int input_yaw;
//	int input_thrust;

	freq = fishMinFrequency;
	yaw = 0;

	pc.printf("starting up cyclic actuator test...\n\n");

	while (1) {
		// inputs are unsigned ints from 0 to 255
//		pc.scanf("%d %d %d", &input_freq, &input_yaw, &input_thrust);
		pc.scanf("%d %d", &input_freq, &input_yaw); // if just testing freq & yaw (valve)
		//pc.scanf("%d %d %d", &input_freq); // if just testing hall sensor interrupt

		if(input_freq == 999) {
			pc.printf("\nstopping system\n");
			cyclicActuator.stop();
			run_timer.detach();
			valveRunningLED = cyclicActuator.getRunState();
		} else if(input_freq == 888){
			pc.printf("\nstarting system\n");
			cyclicActuator.start();
			run_timer.attach(&run_func, 1); //prev 0.2
		} else if(0 <= input_freq && input_freq <= 255){
			// figure out why these interpolation functions are different in actual FishController
			freq = (input_freq * (fishMaxFrequency - fishMinFrequency) / 255.0) + fishMinFrequency;
			//freq = input_freq/256.0;
			yaw = (input_yaw * (fishMaxYaw - fishMinYaw) / 255.0) + fishMinYaw;
	//		thrust = (input_thrust * (fishMaxThrust - fishMinThrust) / 255.0) + fishMinThrust;
		}
	}
}
