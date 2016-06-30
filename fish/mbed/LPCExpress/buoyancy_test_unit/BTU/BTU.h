#ifndef BTU_H
#define BTU_H

#include "mbed.h"
#include "QEI.h"
#include "PID.h"
#include "MS5837.h" // pressure sensor

// Encoder number of pulses per revolution
#define PULSEPERREV 1786
// Max pressure to be experienced (mbar); currently at ~5m under water
#define MAXDEPTH 1516
#define T 0.00345 //period of PWM
// Position PID parameters
#define pKc 1
#define pTauI 1
#define pTauD 1
// Depth PID parameters
#define dKc 0.1
#define dTauI 0.1
#define dTauD 0.1
#define NA 0.1 // placeholder;rate at which the PID is run is not used here but should be used when BTU::Run() is called. BTU::Run() only computes PID once

#define VOLTAGE 	1
#define POSITION	2
#define DEPTH		3

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
	Serial	pc;
    PwmOut 	PWM1;
    PwmOut 	PWM2;
    QEI     motor;
    PID     posPID;
    PID     depthPID;
    MS5837  pressureSensor;
    Ticker  timer;
    int MODE;
    float SETVAL;
    float KC;
    float TAUI;
    float TAUD;

public:
    BTU();
    ~BTU();
    void run();
    void init();
    void stop();
    void voltageControl();
    void updateParam(int M, float A, float B, float C, float D);
    void positionControl();
    void depthControl();
    void voltageDefault();
};

#endif /* BTU_H */

// Create a static instance of BTU to be used by anyone controlling the Buoyancy Test Unit
extern BTU m_BTU;
