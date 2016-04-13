// test file for BuoyancyControlUnit class functions
// NOTE look at the below h files to define whether that control mode is enabled

#include "mbed.h"
#include "BuoyancyControlUnit.h"
#include "BNO055.h"
#include "MS5837.h"

#define fishMinPitch     ((float)(1100)) // need to redefine, probably should not be negative
#define fishMaxPitch     ((float)(1400))

Serial pc(USBTX, USBRX);
DigitalOut bcuDirectionLED(LED3);
Ticker run_timer;
MS5837 pressure_sensor(p28,p27); // where are the pin numbers initialized?

float depth_in = 0;
float depth_meas = 0;

void run_func() {
	pressure_sensor.Barometer_MS5837();
	depth_meas = pressure_sensor.MS5837_Pressure();
	pc.printf("Depth sensor: %f\n", depth_meas);

	buoyancyControlUnit.set(depth_in,depth_meas);
	pc.printf("Depth cmd: %f\n", depth_in);
	pc.printf("BCU Vset: %f\n\n", buoyancyControlUnit.getVset());
	bcuDirectionLED = buoyancyControlUnit.getBCUdir();
}

int main() {
	pc.baud(19200);
	buoyancyControlUnit.start();
	pressure_sensor.MS5837Init();
	int input_cmd = 0;
//	int input_meas = 0;

	run_timer.attach(&run_func, 1);

	while (1) {
		// inputs are unsigned ints from 0 to 255
//		pc.scanf("%d %d\n", &input_cmd, &input_meas); // manually tell it measured depth (make sure this won't cause problems)

		pc.scanf("%d", &input_cmd); // use pressure sensor

		depth_in = (input_cmd * (fishMaxPitch - fishMinPitch) / 255.0) + fishMinPitch;
//		pc.printf("\n\nDepth in: %f\n\n\n", depth_in);
//		depth_meas = (input_meas * (fishMaxPitch - fishMinPitch) / 255.0) + fishMinPitch;
	}
}
