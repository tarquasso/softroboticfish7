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

float freq = 0.1;
float yaw = 0.1;
float thrust = 0.1;

void run_func() {
	cyclicActuator.set(freq, yaw, thrust);

	//pc.printf("Valve Vset: %f\t Time since hall sensor: %f\n", cyclicActuator.getVset(), cyclicActuator.getSensorTime());
	//pc.printf("Set Freq: %f\t Cur Freq: %f\n", cyclicActuator.getSetFreq(), cyclicActuator.getCurFreq());
	//pc.printf("Period 1: %d\t Period 2: %d\n\n", cyclicActuator.getPeriod1(), cyclicActuator.getPeriod2());
	pc.printf("Valve PWM: %f\t Pump PWM: %f\n\n", cyclicActuator.getValvePwm(), cyclicActuator.getPumpPwm());
	//valveSideLED = cyclicActuator.getVside();
	//valveRunningLED = cyclicActuator.getRunState();
}

int main() {
	//pc.baud(19200);
	bcu = 0.0;
	float input_valvePwm;
	float input_pumpPwm;
//	int input_thrust;

	freq = fishMinFrequency;
	yaw = 0;

	pc.printf("starting up cyclic actuator test...\n\n");

	while (1) {
		pc.scanf("%d %d", &input_valvePwm, &input_pumpPwm);

		if(input_valvePwm == 999) {
			pc.printf("\nstopping system\n");
			cyclicActuator.stop();
			run_timer.detach();
			valveRunningLED = cyclicActuator.getRunState();
		} else {
			pc.printf("\nstarting system\n");
			freq = input_valvePwm;
			yaw = 1;
			thrust = input_pumpPwm;
			//cyclicActuator.start();
			//run_timer.attach(&run_func, 0.2); //prev 0.2
			pc.printf("input valve pwm : %f input pump pwm: %f\n\n", input_valvePwm, input_pumpPwm);
		}
	}
}
