
/*
 * BuoyancyControlUnit.h
 *
 * Author: Cyndia Cao, Robert Katzschmann
 */

#ifndef BUOYANCYCONTROLUNIT_H_
#define BUOYANCYCONTROLUNIT_H_

#include "mbed.h"
#include "MS5837/MS5837.h"
#include "QEI/QEI.h"


#define bcuPwmPin p22
#define bcuDirAPin p11
#define bcuDirBPin p12
//#define bcuCurrentPin p20

#define encoderPinA p16
#define encoderPinB p17
#define count2rev 12 //https://www.pololu.com/product/3081/blog
#define gearRatio 75 // need to check

#define imuTXPin p28
#define imuRXPin p27

//#define maxBCUCurrent 0.5 // need to measure
//#define minBCUCurrent 0.1 // need to measure

#define KpDepth 5 // random dummy val
#define KdDepth 0.0 // ignore for now; start with P control
#define KiDepth 0.0

#define KpEnc 0.0005
#define KdEnc 0.00003
#define KiEnc 0.000000000


class BuoyancyControlUnit
{
public:
	// Initialization
	BuoyancyControlUnit();
	void start();
	void stop();
	void set(float depthDesIn);
	void setDepthFunc(float depthDesIn);
	void setEncoderPosition(float setPosIn);
	void setV(float Vin);

	void setDepthFuncVoid();
	void setEncoderPosVoid();

	void returnToZero();
	void runBackwards();
	void runForwards();

	bool getBCUdir();
	float getVset();
	float getSetPos();
	float getCurPos();
	float getCurDepth();
	float getSetDepth();

//	int pressureReadings;
//	int encoderLoops;

private:
	volatile bool resetFlag;
	volatile bool inDepthLoop;
	volatile bool inPosLoop;

	volatile float setDepth;
	volatile float curDepth;
	volatile bool bcuDir;

	volatile float posRef;
	volatile float depthErr;
	volatile float prevDepthErr;
	volatile float depthDer;
	volatile float depthInt;
	volatile float depthLastTime;

	volatile float setPos;
	volatile float curPos;

	volatile float VRef;
	volatile float posErr;
	volatile float prevPosErr;
	volatile float posDer;
	volatile float posInt;
	volatile float posLastTime;

	Timer timer;
	PwmOut bcuPWM;
	DigitalOut bcuDirA;
	DigitalOut bcuDirB;
//	AnalogIn bcuCurrent; // actually is a voltage value proportional to current
	QEI bcuEncoder;
	MS5837 pressureSensor;

	Ticker depthControl;
	Ticker posControl;
};

// Create a static instance of BuoyancyControlUnit to be used by anyone controlling the Buoyancy Control Unit
extern BuoyancyControlUnit buoyancyControlUnit;

#endif /* BUOYANCYCONTROLUNIT_H_ */
