#include "BTU.h"

BTU::BTU():
	pc(USBTX, USBRX),
	PWM1(pwmOutA),
	PWM2(pwmOutB),
	motor(analogInA, analogInB, NC, PULSEPERREV),
	posPID(pKc, pTauI, pTauD, NA),
    depthPID(dKc, dTauI, dTauD, NA),
	pressureSensor(imuTXPin, imuRXPin)
{

};

BTU::~BTU(){}


/**
  * This function uses control loops to set the motor position to the desired state
  * @param mode         1 for voltage, 2 for position, or 3 for depth
  * @param setValue     Desired value of voltage, position, or depth
  * Example A: Run(1,0.75) will set PWM signal to be 75% of the max voltage
  * Example B: Run(2, 37) will rotate the motor to be 37deg CCW of starting pos
  * Example C: Run(2, -37) will rotate the motor to be 37deg CW of starting pos
  * Example D: Run(3,
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
        PWM2.period(T);
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


/** 
  * This function receives a desired position and tries to rotate the motor to that position
  * @param value desired position, in degrees, in range [-360, 360]
  * where negative value is CW and positive value is CCW
  * Note: make sure to set the motor to its rest position before starting
  */
void BTU::positionControl(float setDeg)
{
    // Run PID
	//float setPos = setDeg/360; //convert into position in range [-1,1]
    posPID.setMode(1); // AUTO != 0
    posPID.setInputLimits(-360, 360); // analog input of position to be scaled 0-100%
    posPID.setOutputLimits(-1,1); // PWM output from -1 (CW) to 1 (CCW)
    //posPID.setBias(0.3); // if there is bias
    posPID.setSetPoint(setDeg); // we want the process variable to be the desired value
    posPID.setTunings(0.45,0.1,0.1); //Kc, TauI, TauD (0.5,0.1,0.1) is ok
    posPID.setInterval(0.05);
    float pvPos = 0;
    float pvDeg = 0;
    /**
      *  PID TO BE ITERATED
      */ 
    // Detect the position of motor

    while(1){
    pvPos = motor.getPulses() % PULSEPERREV;
    pvDeg = pvPos/PULSEPERREV*360;

    // Provide appropriate voltage to motor
    posPID.setProcessValue(pvDeg); // update the process variable

    float co = posPID.compute(); // set the new output
    pc.printf("setPos: %.1f deg, pvPos: %.1f deg, duty: %.2f \n",setDeg, pvDeg, co*100);

    voltageControl(co); // change voltage provided to motor
    wait(0.1);
    }
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
