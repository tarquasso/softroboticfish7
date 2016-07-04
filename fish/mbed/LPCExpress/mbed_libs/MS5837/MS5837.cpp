#include <stdlib.h>
#include "MS5837.h"


/*
 * Sensor operating function according data sheet
 */

void MS5837::MS5837Init(void)
{
    MS5837Reset();
    MS5837ReadProm();
    step = 1;

    return;
}

void MS5837::MS5837Start(void)
{
	if(!started)
	{
		MS5837::Barometer_MS5837();
	}
	return;
}

/* Send soft reset to the sensor */
void MS5837::MS5837Reset(void)
{
	timeOut1.detach();
	started = false;
    /* transmit out 1 byte reset command */
    ms5837_tx_data[0] = ms5837_reset;
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    //printf("send soft reset");
    wait_ms(20);
    timeOut1.detach(); // to be sure it was detached
}

/* read the sensor calibration data from rom */
void MS5837::MS5837ReadProm(void)
{
    uint8_t i,j;
    for (i=0; i<8; i++) {
        j = i;
        ms5837_tx_data[0] = ms5837_PROMread + (j<<1);
        if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
        if ( i2c.read( device_address,  ms5837_rx_data, 2 ) );
        C[i]   = ms5837_rx_data[1] + (ms5837_rx_data[0]<<8);
    }
}

/* Start the sensor pressure conversion */
void MS5837::MS5837ConvertD1(void)
{
    ms5837_tx_data[0] = ms5837_convD1;
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
}

/* Start the sensor temperature conversion */
void MS5837:: MS5837ConvertD2(void)
{
    ms5837_tx_data[0] = ms5837_convD2;
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
}

/* Read the previous started conversion results */
int32_t MS5837::MS5837ReadADC(void)
{
    ms5837_tx_data[0] = ms5837_ADCread;
    if ( i2c.write( device_address,  ms5837_tx_data, 1 ) );
    if ( i2c.read( device_address,  ms5837_rx_data, 3 ) );
    adc = ms5837_rx_data[2] + (ms5837_rx_data[1]<<8) + (ms5837_rx_data[0]<<16);
    return (adc);
}

/* return the results */
float MS5837::MS5837_Pressure (void)
{
    return P_MS5837;
}
float MS5837::MS5837_Temperature (void)
{
    return T_MS5837;
}

/* Sensor reading and calculation procedure */
void MS5837::Barometer_MS5837(void) {
	started = true;
	switch (step) {
	case 1:
		MS5837ConvertD1();             // start pressure conversion
		step++;
		timeOut1.attach_us(this, &MS5837::Barometer_MS5837, ms5837_conv_time_us);
		break;
	case 2:
		D1 = MS5837ReadADC(); // read the pressure value
		MS5837ConvertD2(); // start temperature conversion
		step++;
		timeOut1.attach_us(this, &MS5837::Barometer_MS5837, ms5837_conv_time_us);
		break;
	case 3:
		D2 = MS5837ReadADC();         // read the temperature value
		/* calculation according MS5837-01BA data sheet DA5837-01BA_006 */
		dT = D2 - (C[5] * 256);
		OFF = (int64_t) C[2] * (1 << 16)
				+ ((int64_t) dT * (int64_t) C[4]) / (1 << 7);
		SENS = (int64_t) C[1] * (1 << 15)
				+ ((int64_t) dT * (int64_t) C[3]) / (1 << 8);

		temp = 2000 + (dT * C[6]) / (1 << 23);
		T_MS5837 = (float) temp / 100.0f; // result of temperature in deg C in this var
		press = (((int64_t) D1 * SENS) / (1 << 21) - OFF) / (1 << 13);
		P_MS5837 = (float) press / 10.0f; // result of pressure in mBar in this var

		step = 1; //reset to step 1
		//return to step 1
		timeOut1.attach_us(this, &MS5837::Barometer_MS5837, ms5837_wait_time_us);
		break;
	default:
		// if nothing else matches, do the default
		// default is optional
		break;
	}
}
