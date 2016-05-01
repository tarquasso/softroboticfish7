// test file for BuoyancyControlUnit class functions
// NOTE look at the below h files to define whether that control mode is enabled

#include "mbed.h"
#include "BuoyancyControlUnit.h"

#define fishMinPitch     ((float)(1000)) // need to redefine, probably should not be negative
										 // TODO: may have to base on startup pressure
#define fishMaxPitch     ((float)(1400)) // then this would have to be based on a diff from min depth

Serial pc(USBTX, USBRX);
DigitalOut bcuDirectionLED(LED3);
Ticker run_timer;

#define valvePwmPin p21
PwmOut valvePWM(valvePwmPin);


float depth_in;
// float depth_meas = 0;

void run_func() {
	//buoyancyControlUnit.setEncoderPosition(depth_in);

	pc.printf("Pressure: %d \tEncoder: %d\n", buoyancyControlUnit.pressureReadings, buoyancyControlUnit.encoderLoops);

	pc.printf("Depth cmd: %f \t", depth_in);
	pc.printf("BCU Vset: %f\n", buoyancyControlUnit.getVset());

	pc.printf("Depth set: %f\t", buoyancyControlUnit.getSetDepth());
	pc.printf("Depth act: %f\n", buoyancyControlUnit.getCurDepth());
	pc.printf("Pos set: %f\t", buoyancyControlUnit.getSetPos());
	pc.printf("Pos act: %f\n\n", buoyancyControlUnit.getCurPos());

	bcuDirectionLED = buoyancyControlUnit.getBCUdir();
}

int main() {
	pc.baud(19200);
	pc.printf("\n\nstarting up...\n\n");
//	buoyancyControlUnit.start();
	valvePWM = 0;

	int input_cmd = 0;
	depth_in = (input_cmd * (fishMaxPitch - fishMinPitch) / 255.0) + fishMinPitch;
	//depth_in = 0;

//	run_timer.attach(&run_func, .3);
/*
	while (1) {
		pc.printf("getting another input\n");
		// inputs are unsigned ints from 0 to 255
		pc.scanf("%d", &input_cmd);

		if(input_cmd == 9999){
			pc.printf("resetting bcu position\n");
			depth_in = fishMinPitch;

			if(buoyancyControlUnit.getCurPos() < 30) {
				buoyancyControlUnit.stop();
				run_timer.detach();
				pc.printf("motor stopped\n");
			}

		} else if(input_cmd == 9990){
			pc.printf("Current Position: %f\n", buoyancyControlUnit.getCurPos());
			buoyancyControlUnit.stop();
			run_timer.detach();
			pc.printf("motor stopped without position reset\n");
		} else if(input_cmd == 8888){
			buoyancyControlUnit.runBackwards();
		} else if(input_cmd == 7777){
			buoyancyControlUnit.runForwards();
		} else if(input_cmd == 1111){
			pc.printf("bcu started\n");
			run_timer.attach(&run_func, .5);
		} else if(input_cmd <= 255 && input_cmd >= 0){
			depth_in = (input_cmd * (fishMaxPitch - fishMinPitch) / 255.0) + fishMinPitch;
			buoyancyControlUnit.setDepth(depth_in);
			//depth_in = input_cmd * 10000.0 / 255.0;
		}
	}
*/

	while(1) {
		// inputs are unsigned ints from 0 to 255
		pc.scanf("%d", &input_cmd);

		if(input_cmd == 9999){
			pc.printf("resetting bcu position\n");
			buoyancyControlUnit.returnToZero();

		} else if(input_cmd == 9990){
			run_timer.detach();
			pc.printf("motor stopped without position reset\n");
			buoyancyControlUnit.stop();
			pc.printf("Current Position: %f\n", buoyancyControlUnit.getCurPos());

		} else if(input_cmd == 8888){
			buoyancyControlUnit.runBackwards();
		} else if(input_cmd == 7777){
			buoyancyControlUnit.runForwards();

		} else if(input_cmd == 1111){
			pc.printf("bcu started\n");
			buoyancyControlUnit.start();
			buoyancyControlUnit.set(depth_in);
			run_timer.attach(&run_func, .3);

		} else if(input_cmd <= 255 && input_cmd >= 0){
			depth_in = (input_cmd * (fishMaxPitch - fishMinPitch) / 255.0) + fishMinPitch;
			buoyancyControlUnit.set(depth_in);
		}
	}
}
