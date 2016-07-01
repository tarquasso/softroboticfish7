#include "MS5837.h"
#include "mbed.h"

#define PIN_IMU_TX p28
#define PIN_IMU_RX p27

DigitalOut readPressure(LED2);
PwmOut pressure(LED3);
MS5837 pressureSensor(PIN_IMU_TX,PIN_IMU_RX);

float max_pressure = 0;
float min_pressure = 2000;
float curDepth = 0;

int main(){
	pressureSensor.MS5837Init();

	while(1){
		readPressure = 1;
		pressureSensor.Barometer_MS5837();
		curDepth = pressureSensor.MS5837_Pressure();
		readPressure = 0;

		if(curDepth > max_pressure){
			max_pressure = curDepth;
		}
		if(curDepth < min_pressure){
			min_pressure = curDepth;
		}

		if(max_pressure != min_pressure){
			pressure = (curDepth - min_pressure)/(max_pressure - min_pressure);
		}

		wait(1);
	}
	return 0;
}
