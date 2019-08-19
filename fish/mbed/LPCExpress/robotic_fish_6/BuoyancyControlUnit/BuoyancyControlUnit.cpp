#include <BuoyancyControlUnit/BuoyancyControlUnit.h>

// The static instance
BuoyancyControlUnit buoyancyControlUnit;

//============================================
// Initialization
//============================================

// Constructor
BuoyancyControlUnit::BuoyancyControlUnit() :
    				// Initialize variables
    				//bcuPWM(bcuPwmPin),
					bcuDirA(bcuDirAPin),
					bcuDirB(bcuDirBPin),
					//bcuCurrent(bcuCurrentPin),
					wiper(wiperPin),
					pressureSensor(PIN_IMU_TX,PIN_IMU_RX)
{
	setDepth = 0; // input command
	curDepth = 0; // actual depth based on sensor

	depthLastTime = 0.0;
	prevDepthErr = 0.0;
	depthInt = 0.0;

	posLastTime = 0.0;
	prevPosErr = 0.0;
	posInt = 0.0;
	VRef = 0;

	resetFlag = 0;
	inDepthLoop = 0;
	inPosLoop = 0;
}


void BuoyancyControlUnit::start()
{
	bcuDirA.write(0.0); // apply nothing to start
	bcuDirB.write(0.0); // apply nothing to start
	//bcuEncoder.reset(); // gives it a reference to return to
	
	pressureSensor.MS5837Init();
	timer.start();
	
	// pressure sensor takes just over 0.3 seconds to read
	depthControl.attach(&buoyancyControlUnit, &BuoyancyControlUnit::setDepthFuncVoid, 0.4);
	posControl.attach(&buoyancyControlUnit, &BuoyancyControlUnit::setEncoderPosVoid, 0.1);
}

void BuoyancyControlUnit::stop()
{
	//bcuPWM.write(0.0);
	// return bcu back to start position
	// have to rethink how to call this because we don't want the while loop stopping everything else
//	while(curPos > 15){
//		this->setEncoderPosition(0);
//	}

	depthControl.detach();
	posControl.detach();

	bcuDirA = 0.0;
	bcuDirB = 0.0;
}

void BuoyancyControlUnit::returnToZero()
{
	depthControl.detach();
	resetFlag = 1;
}

//============================================
// Processing
//============================================
void BuoyancyControlUnit::set(float depthDesIn)
{
	setDepth = depthDesIn;
}

void BuoyancyControlUnit::setDepthFuncVoid(){
	while(inPosLoop);
	inDepthLoop = 1;
	this->setDepthFunc(setDepth);
	inDepthLoop = 0;
}
void BuoyancyControlUnit::setEncoderPosVoid(){
	while(inDepthLoop);
	inPosLoop = 1;

	if(resetFlag){
		posRef = 0;
	} else {
		depthErr = setDepth - curDepth;
		if(depthErr < 5 && depthErr > -5){
			posRef = curPos;
		}
	}

	this->setEncoderPosition(posRef);
	inPosLoop = 0;
}

void BuoyancyControlUnit::setDepthFunc(float depthDesIn){
	setDepth = depthDesIn;
//	pressureSensor.Barometer_MS5837();
//	curDepth = pressureSensor.MS5837_Pressure();
/*
TODO: Map measured depth to 0 - 30 using h = P/rg. 
#define MIN_DEPTH 1019.0 //mbar
curDepth = curDepth * (fishMaxPitch - fishMinPitch)/(MAX_DEPTH - MIN_DEPTH)
*/
	curDepth = (curPos * 1.25); // fake depth readings

	depthErr = setDepth - curDepth;

/*	float depthdt = timer.read() - depthLastTime;

	depthInt += depthErr * depthdt;
	depthDer = (depthErr - prevDepthErr)/depthdt;

	posRef += KpDepth*depthErr + KiDepth*depthInt + KdDepth*depthDer;
*/
	posRef += KpDepth * depthErr;
	
	if (posRef > 30){
		posRef = 30;
	}
	else if (posRef < 0){
		posRef = 0;
	}

//	if(posRef < 0){
//		posRef = 0; // limit position to only values greater than 0
//		depthInt = 0; // reset these values so they don't build while the bcu can't reach this depth value
//		depthDer = 0;
//	}

//	depthLastTime = timer.read();
//	prevDepthErr = depthErr;
}


void BuoyancyControlUnit::setEncoderPosition(float setPosIn) {

	setPos = setPosIn;
	if(setPos < 0){ setPos = 0; }
	curPos = wiper * (30.0); //map ADC reading to system level voltage, try adjusting pitch max and min values (0 - 30)
	posErr = setPos - curPos;
	float posdt = timer.read() - posLastTime;

	posInt += posErr * posdt;
	posDer = (posErr - prevPosErr)/posdt;

	VRef = KpEnc * posErr + KiEnc * posInt + KdEnc * posDer;
//	VRef = KpEnc * posErr;

	/* Add constrain + map here */
	if (VRef > 6.0){
		VRef = 6.0;
	}
	else if (VRef < -6.0){
		VRef = -6.0;
	}
	
	// Map Vref to 0.7 - 1.0 duty cycle
	if (VRef < 0){
		VRef = VRef * (0.3/(-6.0)) + 0.7;
		bcuDir = 1;
	}
	else if (VRef > 0){
		VRef = VRef * (0.3/(6.0)) + 0.7;
		bcuDir = 0;
	}
	
	posLastTime = timer.read();
	prevPosErr = posErr;

	if(resetFlag){
		if(posErr > 10 || posErr < -10){
			this->setV(VRef);
		} else {
			resetFlag = 0;
			this->stop();
		}
	} else {
		if(posErr > 5 || posErr < -5) { // TODO: determine appropriate tolerance (final error from target pos?)
			this->setV(VRef);
			DigitalOut test(LED1);
			test = 0;
		} else {
			bcuDirA = 0.0;
			bcuDirB = 0;
			//Testing whether fishController is running
			DigitalOut test(LED1);
			test = 1;
		}
	}
}

void BuoyancyControlUnit::setV(float Vin){
	float setV;
	
	if(bcuDir == 1){
		bcuDirA.write(setV);
		bcuDirB.write(0.0);
	} else if(bcuDir == 0){
		bcuDirA.write(0.0);
		bcuDirB.write(setV);
	}
	
}

void BuoyancyControlUnit::runBackwards() {
	this->stop();
	this->setV(-0.2);
}

void BuoyancyControlUnit::runForwards() {
	this->stop();
	this->setV(0.2);
}

bool BuoyancyControlUnit::getBCUdir() { return bcuDir; }
float BuoyancyControlUnit::getVset() { return VRef; }
float BuoyancyControlUnit::getSetDepth() { return setDepth; }
float BuoyancyControlUnit::getCurDepth() { return curDepth; }
float BuoyancyControlUnit::getSetPos() { return setPos; }
float BuoyancyControlUnit::getCurPos() { return curPos; }
float BuoyancyControlUnit::readPressure()
{
	pressureSensor.Barometer_MS5837();
	return pressureSensor.MS5837_Pressure();
}
