#ifndef BTU_H
#define BTU_H

#include "mbed.h"
#include "QEI.h"
#include "PID.h"
#include "MS5837.h" // pressure sensor

// Encorder number of pulses per revolution
#define PULSEPERREV 1786
// Max pressure to be experienced (mbar); currently at ~5m under water
#define MAXDEPTH 1516
// Voltage PID parameters
#define vKc 0.1
#define vTaul 0.1
#define vTauD 0.1
// Depth PID parameters
#define dKc 0.1
#define dTaul 0.1
#define dTauD 0.1
#define NA 0.1 // holceholder; rate at which the PID is run is not used here but should be used when BTU::Run() is called. BTU::Run() only computes PID once


// Define pinouts from mbed
#define analogInA p16
#define analogInB p17
#define pwmOutA p21
#define pwmOutB p22
#define imuTXPin p28
#define imuRXPin p27

/**
* This class sets the motor position to the desired state
* It includes instances of the classes PwmOut, QEI, PID, and MS5837
*/
class BTU
{
private:
    PwmOut PWM1;
    PwmOut PWM2;
    QEI     motor;
    PID     voltagePID;
    PID     depthPID;
    MS5837  pressureSensor;

public:
    BTU();
    ~BTU();
    void Run(int mode, float setValue);
    void voltageControl(float setDuty, float T = 0);
    void positionControl(float setPos);
    void depthControl(float setDepth);
    void voltageDefault();
};

#endif /* BTU_H */