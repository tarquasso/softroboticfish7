#include "mbed.h"

#ifndef MS5837_H
#define MS5837_H

#define MS5837_RX_DEPTH 3 //
#define MS5837_TX_DEPTH 2 //

#define ms5837_wait_time_us 200
// choose your connection here
#define ms5837_addr_no_CS  0x76 //0b1110110

#define ms5837_reset       0x1E // Sensor Reset

#define ms5837_convD1_256  0x40 // Convert D1 OSR 256
#define ms5837_convD1_512  0x42 // Convert D1 OSR 512
#define ms5837_convD1_1024 0x44 // Convert D1 OSR 1024
#define ms5837_convD1_2048 0x46 // Convert D1 OSR 2048
#define ms5837_convD1_4096 0x48 // Convert D1 OSR 4096
#define ms5837_convD1_8192 0x4A // Convert D1 OSR 8192

#define ms5837_convD1 ms5837_convD1_4096 // choose your sampling rate here

#define ms5837_convD2_256  0x50 // Convert D2 OSR  256
#define ms5837_convD2_512  0x52 // Convert D2 OSR  512
#define ms5837_convD2_1024 0x54 // Convert D2 OSR 1024
#define ms5837_convD2_2048 0x56 // Convert D2 OSR 2048
#define ms5837_convD2_4096 0x58 // Convert D2 OSR 4096
#define ms5837_convD2_8192 0x5A // Convert D2 OSR 8192

#define ms5837_convD2 ms5837_convD2_4096 // choose your sampling rate here

#define ms5837_conv_time_us_256  600 // Conversion Time OSR  256
#define ms5837_conv_time_us_512  1170 // Conversion Time OSR  512
#define ms5837_conv_time_us_1024 2280 // Conversion Time OSR 1024
#define ms5837_conv_time_us_2048 4540 // Conversion Time OSR 2048
#define ms5837_conv_time_us_4096 9040 // Conversion Time OSR 4096
#define ms5837_conv_time_us_8192 18080 // Conversion Time OSR 8192

#define ms5837_conv_time_us ms5837_conv_time_us_4096

#define ms5837_ADCread     0x00 // read ADC command
#define ms5837_PROMread    0xA0 // read PROM command base address

class MS5837{
private:
    int D1, D2, Temp, C[8];
    float T_MS5837, P_MS5837;
    int32_t dT, temp;
    int64_t OFF, SENS, press;
    int32_t adc;
    int step;
    bool started;
    bool m_finishedCalc;
    /* Data buffers */
    char ms5837_rx_data[MS5837_RX_DEPTH];
    char ms5837_tx_data[MS5837_TX_DEPTH];
    Timeout timeOut1;

public:
    MS5837 (PinName sda, PinName scl,
            char ms5837_addr = ms5837_addr_no_CS  )
            : step(1), started(false), m_finishedCalc(false), i2c( sda, scl ), device_address( ms5837_addr << 1 ) {
    }
    void MS5837Init(void);
    void MS5837Reset(void);
    void MS5837Start(void);
    bool MS5837Done(void);

    float MS5837_Pressure (void);
    float MS5837_Temperature (void);
protected:
    void Barometer_MS5837(void);
    void MS5837ReadProm(void);
    void MS5837ConvertD1(void);
    void MS5837ConvertD2(void);
    int32_t MS5837ReadADC(void);



private:
    I2C     i2c;
    char    device_address;

};
#endif
