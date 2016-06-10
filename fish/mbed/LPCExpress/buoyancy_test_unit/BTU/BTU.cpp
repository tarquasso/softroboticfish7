#include "BTU.h"

BTU::BTU():
	PWM1(pwmOutA),
	PWM2(pwmOutB),
	motor(analogInA, analogInB, NC, PULSEPERREV),
	voltagePID(vKc, vTaul, vTauD, NA),
    depthPID(dKc, dTaul, dTauD, NA),
	pressureSensor(imuTXPin, imuRXPin)
{

};

BTU::~BTU(){}


/**
  * This function uses control loops to set the motor position to the desired state
  * @param mode         1 for voltage, 2 for position, or 3 for depth
  * @param setValue     Desired value of voltage, position, or depth
  */  
void BTU::Run(int mode, float setValue)
{
    voltageDefault();

    switch (mode)
    {
        case 1:
        voltageControl(setValue);
        break;

        case 2:
        positionControl(setValue);
        break;

        case 3:
        depthControl(setValue);
        break;
    }
}


/**
  * This function sets the voltage provided to the motor to a desired voltage 
  * by changing the PWM
  * @param duty     Desired pwm duty cycle in range [-1,1]; negative for CW, positive for CCW rotation
  * @param T        (Optional arg) change period
  */  
void BTU::voltageControl(float setDuty, float T)
{
    if (T != 0)
    {
        PWM1.period(T);
    }
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


// Only work with positive pulse values. When starting, set the initial position to when the BCU is fully untwisted (or even reverse twisted. either way, don't let the pulse count go negative)
/** 
  * This function receives a desired position iand tries to go to that position
  * @param value Desired position in range [0,1)
  */

void BTU::positionControl(float setPos)
{
    // Run PID
    voltagePID.setMode(1); // AUTO != 0
    voltagePID.setInputLimits(0.0, PULSEPERREV); // analog input of position to be scaled 0-100%
    voltagePID.setOutputLimits(0,1); // PWM output from 0 to 1
    voltagePID.setBias(0.3); // if there is bias
    voltagePID.setSetPoint(setPos); // we want the process variable to be the desired value
    
    float pvPos = 0;

    /**
      *  PID TO BE ITERATED
      */ 
    // Detect the position of motor
    pvPos = motor.getPulses() % PULSEPERREV;
    
    // Provide appropriate voltage to motor
    voltagePID.setProcessValue(pvPos); // update the process variable
    float co = voltagePID.compute(); // set the new output
    
    voltageControl(co); // change voltage provided to motor
}


/** 
  * This function changes the position of the motor to a desired value based on
  * the readings on the pressure sensor
  * @param value 
  */
void BTU::depthControl(float setDepth)
{
    // Run PID
    depthPID.setMode(1); // nonzero: AUTO
    depthPID.setInputLimits(0.0, MAXDEPTH); // analog input of position to be scaled 0-100%
    depthPID.setOutputLimits(0,1); // position output from 0 to 1
    depthPID.setBias(0.3); // if there is bias
    depthPID.setSetPoint(setDepth); // we want the process variable to be the desired value
    
    float pvDepth = 0;
    pressureSensor.MS5837Init();

    /**
      *  PID TO BE ITERATED
      */     
    // Detect depth
    pvDepth = pressureSensor.MS5837_Pressure();
    
    // Set position of motor
    depthPID.setProcessValue(pvDepth); // update the process variable
    float co = depthPID.compute(); // set the new output

    positionControl(co); // change position of motor

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
