
/*
 * PumpWithValve.h
 *
 * Author: Cyndia Cao, Robert Katzschmann
 */

#ifndef PUMPWITHVALVE_H_
#define PUMPWITHVALVE_H_

#include "mbed.h"
//#include "../QEI/QEI.h"

#define valvePwmPin 		p21
#define pumpPwmPin 			p23
#define hallInterruptPin 	p18
//#define encoderPinA 		p25
//#define encoderPinB 		p24
//#define valveCurrentPin 	p19

#define count2rev 			12
#define valveMotorGearRatio 297.92
#define freq_PGain 			0.05 // random dummy number
#define freq_DGain 			0.01 // dummy val
#define valveOffsetGain 	1.5

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

	float getVset();
	bool getVside();

protected:
	void calculateYawMethod1();
	void calculateYawMethod2();

private:

	volatile float frequency;
	volatile float yaw;
	volatile float thrust;

	volatile bool 	valve_side;
	volatile float 	valveV1;
	volatile float 	valveV2;
	volatile float 	Vfreq;
	volatile float 	VfreqAdjusted;
	volatile float  Vset;

//	volatile float dt;
//	volatile float rot;
//	volatile float freq_act;
//	volatile float freq_error;
//	volatile float prev_freq_error;
//	volatile float dV_freq;

	PwmOut pumpPWM;
	PwmOut valvePWM;
	//QEI valveEncoder;
	//AnalogIn valveCurrent; // actually is a voltage value proportional to current
	InterruptIn hallSignal;
	Timer timer;
	DigitalOut valveLED;
};

// Create a static instance of PumpWithValve to be used by anyone controlling the pump with valve
extern PumpWithValve pumpWithValve;

#endif /* PUMPWITHVALVE_H_ */
