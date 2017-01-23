
#include "mbed.h"
#include "MS5837.h"
#include "SerialComm/SerialComm.h"

#define PIN_IMU_SDA p28
#define PIN_IMU_SCL p27

#define TIMESTEP 0.3

Serial pc(USBTX,USBRX); //serial device

I2CManager manager(PIN_IMU_SDA, PIN_IMU_SCL, &pc);
MS5837 pressureSensor(manager);

float pressure, temperature;

int main() {
	manager.start_comms();
	int res;
	pc.printf("Start!\r\n");
	res = pressureSensor.init();
	pc.printf("init: %d\r\n", res);

	pressureSensor.start();
	pc.printf("start\r\n");

	while (1) {
		if (pressureSensor.done()) {
			pressure = pressureSensor.get_pressure();
			temperature = pressureSensor.get_temperature();
			pc.printf("pressure:%.1f, temp:%.2f\r\n", pressure, temperature);
		}

		wait_ms(1000);
	}

}
