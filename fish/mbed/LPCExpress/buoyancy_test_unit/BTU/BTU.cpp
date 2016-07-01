#include "BTU.h"

// The static instance
BTU m_BTU;

BTU::BTU():
	PWM1(pwmOutA),
	PWM2(pwmOutB),
	motor(analogInA, analogInB, NC, PULSEPERREV),
	posPID(pKc, pTauI, pTauD, NA),
    depthPID(dKc, dTauI, dTauD, NA),
	pressureSensor(imuTXPin, imuRXPin)
{};

BTU::~BTU(){}


void BTU::init()
{
	counter = 0;
	CURRENTVAL = 555;
	OUTPUT = 444;
	MODE = 0;
	KC = 0;
	TAUI = 0;
	TAUD = 0;
	SETVAL = 0;
    voltageDefault();

	posPID.setMode(1); // AUTO != 0
	posPID.setInputLimits(-360, 360); // analog input of position to be scaled 0-100%
	posPID.setOutputLimits(-1,1); // PWM output from -1 (CW) to 1 (CCW)

	depthPID.setMode(1); // nonzero: AUTO
	depthPID.setInputLimits(0.0, MAXDEPTH); // analog input of position to be scaled 0-100%
	depthPID.setOutputLimits(-360, 360); // position output from -360 to 360 deg
	pressureSensor.MS5837Init();
}


/**
 * Update global variables
 */
void BTU::update(int M, float A, float B, float C, float D)
{
	if (MODE != M)
	{
		updateMode(M);
	}
	MODE = M;
	SETVAL = A;
	KC = B;
	TAUI = C;
	TAUD = D;
}


void BTU::updateMode(int mode)
{
	stop();
    switch (mode)
    {
        case VOLTAGE:
        timer.attach(this, &BTU::voltageControl,0.05);
        break;

        case POSITION:
		timer.attach(this, &BTU::positionControl,0.05);
        break;

        case DEPTH:
        timer.attach(this, &BTU::depthControl,0.05);
        break;
    }
}


void BTU::stop()
{
	timer.detach();
	PWM1 = 0;
	PWM2 = 0;
}


void BTU::printGlobal()
{
	pc.printf("GLOBAL::: counter: %d, mode: %d, Kc:%f, TauI:%f, TauD:%f, SETVAL: %.2f, CURRENTVAL: %.2f, DUTY CYCLE: %.2f \n",counter, MODE, KC, TAUI, TAUD, SETVAL, CURRENTVAL, OUTPUT*100);
}


/**
 * This function sets the voltage provided to the motor to a desired voltage
 * by changing the PWM
 * @param duty     Desired pwm duty cycle in range [-1,1]; negative for CW, positive for CCW rotation
 * @param T        (Optional arg) change period
 */
void BTU::voltageControlHelper(float setDuty)
{
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

void BTU::voltageControl()
{
	voltageControlHelper(SETVAL);
}

/** 
 * This function receives a desired position and tries to rotate the motor to that position
 * setDeg is in range [-360, 360] where negative value is CW and positive value is CCW
 * Note: make sure to set the motor to its rest position before starting
 */
void BTU::positionControl()
{
	counter++;
	posPID.setTunings(KC, TAUI, TAUD);
    posPID.setSetPoint(SETVAL); // we want the process variable to be the desired value

    // Detect motor position
    float pvPos = motor.getPulses() % PULSEPERREV;
    float pvDeg = (pvPos/PULSEPERREV)*360;
    CURRENTVAL = pvDeg;
    // Set motor voltage
    posPID.setProcessValue(CURRENTVAL); // update the process variable
    OUTPUT = posPID.compute();
    voltageControlHelper(OUTPUT); // change voltage provided to motor
}


/** 
 * This function changes the motor position to a desired value based on
 * the readings on the pressure sensor
 * setDepth is the pressure reading
 */
void BTU::depthControl()
{
	depthPID.setTunings(KC, TAUI, TAUD);
    // Run PID
	float setDepth = SETVAL;
    depthPID.setSetPoint(setDepth); // we want the process variable to be the desired value
    
    // Detect depth
    float pvDepth = pressureSensor.MS5837_Pressure();
    
    // Set motor position
    depthPID.setProcessValue(pvDepth); // update the process variable
    SETVAL = depthPID.compute(); // set the new output of position
    //pc.printf("setDepth: %.1f mbar, pvPos: %.1f mbar, setPos: %.2f deg\n",setDepth, pvDepth, SETVAL);
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
