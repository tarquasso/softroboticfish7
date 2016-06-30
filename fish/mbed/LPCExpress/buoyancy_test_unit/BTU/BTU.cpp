#include "BTU.h"

// The static instance
BTU m_BTU;

BTU::BTU():
	pc(USBTX, USBRX),
	PWM1(pwmOutA),
	PWM2(pwmOutB),
	motor(analogInA, analogInB, NC, PULSEPERREV),
	posPID(pKc, pTauI, pTauD, NA),
    depthPID(dKc, dTauI, dTauD, NA),
	pressureSensor(imuTXPin, imuRXPin)
{};

BTU::~BTU(){}

/**
 * This function uses control loops to set the motor position to the desired state
 * @param mode         1 for voltage, 2 for position, or 3 for depth
 * @param SETVALue     Desired value of voltage, position, or depth
 * Example A: Run(1,0.75) will set PWM signal to be 75% of the max voltage
 * Example B: Run(2, 37) will rotate the motor to be 37deg CCW of starting pos
 * Example C: Run(2, -37) will rotate the motor to be 37deg CW of starting pos
 * Example D: Run(3,
 */
//void BTU::Run(int mode, float setValue)
//{
//    switch (mode)
//    {
//        case VOLTAGE:
//        voltageControl();
//        break;
//
//        case POSITION:
//        positionControl();
//        break;
//
//        case DEPTH:
//        depthControl();
//        break;
//    }
//}


void BTU::init()
{
	KC = 0;
	TAUI = 0;
	TAUD = 0;
	SETVAL = 0;
}


void BTU::run()
{
    voltageDefault();

    switch (MODE)
    {
        case VOLTAGE:
        timer.attach(&m_BTU, &BTU::voltageControl,0.05);
        break;

        case POSITION:
		posPID.setMode(1); // AUTO != 0
		posPID.setInputLimits(-360, 360); // analog input of position to be scaled 0-100%
		posPID.setOutputLimits(-1,1); // PWM output from -1 (CW) to 1 (CCW)
        timer.attach(&m_BTU, &BTU::positionControl,0.05);
        break;

        case DEPTH:
		depthPID.setMode(1); // nonzero: AUTO
		depthPID.setInputLimits(0.0, MAXDEPTH); // analog input of position to be scaled 0-100%
		depthPID.setOutputLimits(-360, 360); // position output from -360 to 360 deg
		pressureSensor.MS5837Init();
        timer.attach(&m_BTU, &BTU::depthControl,0.05);
        break;
    }
}


void BTU::stop()
{
	timer.detach();
	PWM1 = 0;
	PWM2 = 0;
}


/**
 * This function sets the voltage provided to the motor to a desired voltage
 * by changing the PWM
 * @param duty     Desired pwm duty cycle in range [-1,1]; negative for CW, positive for CCW rotation
 * @param T        (Optional arg) change period
 */
void BTU::voltageControl()
{
	float setDuty = SETVAL;

    if (setDuty > 0) //CCW
    {
        PWM1 = setDuty;
        PWM2 = 0;
    }
    else //CW
    {
        PWM1 = 0; // set duty cycle
        PWM2 = -setDuty;
    }
}

/**
 * Update global variables
 */
void BTU::updateParam(int M, float A, float B, float C, float D)
{
	MODE = M;
	SETVAL = A;
	KC = B;
	TAUI = C;
	TAUD = D;
}

/** 
 * This function receives a desired position and tries to rotate the motor to that position
 * setDeg is in range [-360, 360] where negative value is CW and positive value is CCW
 * Note: make sure to set the motor to its rest position before starting
 */
void BTU::positionControl()
{
	float setDeg = SETVAL;
    posPID.setSetPoint(setDeg); // we want the process variable to be the desired value

    // Detect motor position
    float pvPos = motor.getPulses() % PULSEPERREV;
    float pvDeg = pvPos/PULSEPERREV*360;

    // Set motor voltage
    posPID.setProcessValue(pvDeg); // update the process variable
    SETVAL = posPID.compute(); // set the new value of voltage
    pc.printf("setPos: %.1f deg, pvPos: %.1f deg, duty: %.2f \n",setDeg, pvDeg, SETVAL*100);
    voltageControl(); // change voltage provided to motor
}


/** 
 * This function changes the motor position to a desired value based on
 * the readings on the pressure sensor
 * setDepth is the pressure reading
 */
void BTU::depthControl()
{
    // Run PID
	float setDepth = SETVAL;
    depthPID.setSetPoint(setDepth); // we want the process variable to be the desired value
    
    // Detect depth
    float pvDepth = pressureSensor.MS5837_Pressure();
    
    // Set motor position
    depthPID.setProcessValue(pvDepth); // update the process variable
    SETVAL = depthPID.compute(); // set the new output of position
    pc.printf("setDepth: %.1f mbar, pvPos: %.1f mbar, setPos: %.2f deg\n",setDepth, pvDepth, SETVAL);
    positionControl(); // change position of motor
}


/**
 * This function sets the PWM to the default values
 * to rotate the motor CCW at half the supplied voltage
 */
void BTU::voltageDefault()
{
    PWM1.period(0.00345); // 3.45ms period
    PWM2.period(0.00345); // 3.45ms period
    PWM1 = 0.5;           // duty cycle of 50%
    PWM2 = 0;
}
