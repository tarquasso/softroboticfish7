
/*
 * BuoyancyControlUnit.h
 *
 * Author: Cyndia Cao, Robert Katzschmann
 */

#ifndef BUOYANCYCONTROLUNIT_H_
#define BUOYANCYCONTROLUNIT_H_

#include "mbed.h"
//#include "MS5837.h"
#include "QEI/QEI.h"


#define bcuPwmPin p22
#define bcuDirAPin p11
#define bcuDirBPin p12
#define bcuCurrentPin p20

#define encoderPinA p25 // tbd
#define encoderPinB p24 // tbd
#define count2rev 12 //https://www.pololu.com/product/3081/blog
#define gearRatio 75 // need to check


//#define maxBCUCurrent 0.5 // need to measure
//#define minBCUCurrent 0.1 // need to measure

#define KpDepth 0.5 // random dummy val
#define KdDepth 0.0 // ignore for now; start with P control
#define KiDepth 0.0

#define KpEnc 0.1
#define KdEnc 0.01
#define KiEnc 0.0


class BuoyancyControlUnit
{
public:
	// Initialization
	BuoyancyControlUnit();
	void start();
	void stop();
	void set(float depthDesIn, float depthMeasIn);
	void setEncoderPosition(float setPosIn);
	bool getBCUdir();
	float getVset();

private:
	volatile float depthDes;
	volatile float depthMeas;
	volatile bool bcuDir;

	volatile float posRef;
	volatile float depthErr;
	volatile float prevDepthErr;
	volatile float depthDer;
	volatile float depthInt;
	volatile float depthLastTime;

	volatile float setPos;
	volatile float currPos;

	volatile float VRef;
	volatile float posErr;
	volatile float prevPosErr;
	volatile float posDer;
	volatile float posInt;
	volatile float posLastTime;


	Timer timer;
	PwmOut bcuPWM;
	QEI bcuEncoder;
	DigitalOut bcuDirA;
	DigitalOut bcuDirB;
	AnalogIn bcuCurrent; // actually is a voltage value proportional to current
	//MS5837 pressureSensor;
};

// Create a static instance of BuoyancyControlUnit to be used by anyone controlling the Buoyancy Control Unit
extern BuoyancyControlUnit buoyancyControlUnit;

#endif /* BUOYANCYCONTROLUNIT_H_ */
