#include "mbed.h"
#include "I2CManager/I2CManager.h"

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

public:
    MS5837 (I2CManager const &manager, char ms5837_addr = ms5837_addr_no_CS)
        : _manager(manager)
        , device_address(ms5837_addr << 1)
        , started(false)
        , m_finishedCalc(false)
        {}
    int init(void);
    int reset(void);
    void start(void);
    bool done(void);

    float get_pressure (void);
    float get_temperature (void);
protected:
    void calculate_values(void);
    int read_prom();
    int convert_d1(void);
    int convert_d2(void);
    int read_adc(int *res);
private:
    I2CManager _manager;
    Thread *runner;
    char device_address;
    int D1, D2, Temp, C[8];
    float temperature, pressure;
    int32_t dT, temp;
    int64_t OFF, SENS, press;
    int32_t adc;
    bool started;
    bool m_finishedCalc;
    /* Data buffers */
    char rx_data[MS5837_RX_DEPTH];
    Timeout timeOut1;

    static void calculation_helper(const void *args);
    bool command(char loc);
    bool readLen(char loc, char* buffer, char len);

};
#endif
