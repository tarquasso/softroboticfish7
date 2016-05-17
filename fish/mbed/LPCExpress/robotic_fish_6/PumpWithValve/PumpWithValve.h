
/*
 * PumpWithValve.h
 *
 * Author: Cyndia Cao, Robert Katzschmann
 */

#ifndef PUMPWITHVALVE_H_
#define PUMPWITHVALVE_H_

#include "mbed.h"

#define valvePwmPin 		p21
#define pumpPwmPin 			p23
#define hallInterruptPin 	p18
//#define encoderPinA 		p25
//#define encoderPinB 		p24
//#define valveCurrentPin 	p19

#define count2rev 			12
#define valveMotorGearRatio 297.92
#define KpFreq	 			10000.0 // frequency on the order of 10^-7
#define KdFreq	 			0.00
#define valveOffsetGain 	0.5

class PumpWithValve
{
public:
	// Initialization
	PumpWithValve();
	void start();
	void stop();

	void flipFlowUp();
	void flipFlowDown();
	void set(float freq_in, float yaw_in, float thrust_in);
	void setVoid();

	float getVset();
	bool getVside();

protected:
	void calculateYawMethod1();
	void calculateYawMethod2();

private:

	volatile float frequency;
	volatile float yaw;
	volatile float thrust;
	volatile float periodSide1;
	volatile float periodSide2;

	volatile bool 	valveSide;
	volatile float 	valveV1;
	volatile float 	valveV2;
	volatile float 	Vfreq;
	volatile float 	VfreqAdjusted;
	volatile float  Vset;

	volatile float freqAct;
	volatile float freqErr;
	volatile float prevFreqErr;
	volatile float dVFreq;

	PwmOut pumpPWM;
	PwmOut valvePWM;
	//QEI valveEncoder;
	//AnalogIn valveCurrent; // actually is a voltage value proportional to current
	InterruptIn hallSignal;
	Timer timer;
	DigitalOut valveLED;

	Ticker valveControl;
};

// Create a static instance of PumpWithValve to be used by anyone controlling the pump with valve
extern PumpWithValve pumpWithValve;

#endif /* PUMPWITHVALVE_H_ */
