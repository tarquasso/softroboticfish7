#include <stdlib.h>
#include "MS5837.h"
#include "error.h"

/*
 * Sensor operating function according data sheet
 */

int MS5837::init(void) {
	int r;
	if((r = reset()) < 0) {
		return r;
	}
	wait_ms(1000);
	if((r = read_prom()) < 0) {
		return r;
	}
	return 0;
}

void MS5837::start(void) {
	if (!started) {
		runner = new Thread(calculation_helper, this);
	}
}

void MS5837::calculation_helper(const void *args) {
	((MS5837*)args)->calculate_values();
}

/* Send soft reset to the sensor */
int MS5837::reset(void) {
	if (runner != NULL) {
		runner->terminate();
	}
	started = false;
	/* transmit out 1 byte reset command */
	if (command(ms5837_reset)) {
		return -ERROR_NACK;
	}
	//printf("send soft reset");
	return 0;
}

bool MS5837::done(void) {
	if (m_finishedCalc) {
		m_finishedCalc = false;
		return true;
	}
	return false;
}

/* read the sensor calibration data from rom */
int MS5837::read_prom(void) {
	uint8_t i, j;
	for (i = 0; i < 7; i++) {
		j = i;
		if (readLen(ms5837_PROMread + (j << 1), rx_data, 2)) {
			return -ERROR_NACK;
		}
		C[i] = rx_data[1] + (rx_data[0] << 8);
	}
	return 0;
}

/* Start the sensor pressure conversion */
int MS5837::convert_d1(void) {
	if (command(ms5837_convD1)){
		return -ERROR_NACK;
	}
	return 0;
}

/* Start the sensor temperature conversion */
int MS5837::convert_d2(void) {
	if (command(ms5837_convD2)){
		return -ERROR_NACK;
	}
	return 0;
}

/* Read the previous started conversion results */
int MS5837::read_adc(int *res) {
	if (res == NULL) {
		return -ERROR_NULL_POINTER;
	}
	if (readLen(ms5837_ADCread, rx_data, 3)) {
		return -ERROR_NACK;
	}
	*res = rx_data[2] + (rx_data[1] << 8) + (rx_data[0] << 16);
	return 0;
}

/* return the results */
float MS5837::get_pressure(void) {
	return pressure;
}
float MS5837::get_temperature(void) {
	return temperature;
}

/* Sensor reading and calculation procedure */
void MS5837::calculate_values(void) {
	started = true;
	while (true) {
		if(convert_d1() < 0) {
			continue;
		}
		wait_us(2*ms5837_conv_time_us);
		if(read_adc(&D1) < 0) {
			continue;
		}
		if(convert_d2()< 0) {
			continue;
		}
		wait_us(2*ms5837_conv_time_us);
		if(read_adc(&D2) < 0) {
			continue;
		}
		dT = D2 - (C[5] * 256);
		OFF = (int64_t) C[2] * (1 << 16)
				+ ((int64_t) dT * (int64_t) C[4]) / (1 << 7);
		SENS = (int64_t) C[1] * (1 << 15)
				+ ((int64_t) dT * (int64_t) C[3]) / (1 << 8);

		temp = 2000 + (dT * C[6]) / (1 << 23);
		temperature = (float) temp / 100.0f; // result of temperature in deg C in this var
		press = (((int64_t) D1 * SENS) / (1 << 21) - OFF) / (1 << 13);
		pressure = (float) press / 10.0f; // result of pressure in mBar in this var
		m_finishedCalc = true;
		wait_us(ms5837_wait_time_us);
	}
}

bool MS5837::command(char loc) {
  struct i2creq *req = _manager.transaction_queue.alloc();
  req->req_type = I2CREQ_COMMAND;
  req->req_buf.command.addr = device_address;
  req->req_buf.command.loc = loc;
  bool success;
  req->req_buf.command.success = &success;
  req->req_buf.command.thread_id = Thread::gettid();
  _manager.transaction_queue.put(req);
  Thread::signal_wait(0x1);
  return success;
}

bool MS5837::readLen(char loc, char * buffer, char len) {
  struct i2creq *req = _manager.transaction_queue.alloc();
  req->req_type = I2CREQ_READLEN;
  req->req_buf.readLen.addr = device_address;
  req->req_buf.readLen.loc = loc;
  req->req_buf.readLen.buffer = buffer;
  req->req_buf.readLen.len = len;
  bool success;
  req->req_buf.readLen.success = &success;
  req->req_buf.readLen.thread_id = Thread::gettid();
  _manager.transaction_queue.put(req);
  Thread::signal_wait(0x1);
  return success;
}
