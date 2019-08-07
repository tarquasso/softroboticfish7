#include <BuoyancyControlUnit/BuoyancyControlUnit.h>

// The static instance
BuoyancyControlUnit buoyancyControlUnit;

//============================================
// Initialization
//============================================

// Constructor
BuoyancyControlUnit::BuoyancyControlUnit() :
    				// Initialize variables
    				bcuPWM(bcuPwmPin),
					bcuDirA(bcuDirAPin),
					bcuDirB(bcuDirBPin),
					//bcuCurrent(bcuCurrentPin),
					bcuEncoder(encoderPinA, encoderPinB, NC, count2rev*gearRatio, QEI::X4_ENCODING),
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
	bcuEncoder.reset(); // gives it a reference to return to
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
		if(depthErr < 10 && depthErr > -10){
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

	curDepth = (curPos/10000)*(400)+1000; // fake depth readings

	depthErr = setDepth - curDepth;

/*	float depthdt = timer.read() - depthLastTime;

	depthInt += depthErr * depthdt;
	depthDer = (depthErr - prevDepthErr)/depthdt;

	posRef += KpDepth*depthErr + KiDepth*depthInt + KdDepth*depthDer;
*/
	posRef += KpDepth * depthErr;

	if(posRef < 0){
		posRef = 0; // limit position to only values greater than 0
//		depthInt = 0; // reset these values so they don't build while the bcu can't reach this depth value
//		depthDer = 0;
	}

//	depthLastTime = timer.read();
//	prevDepthErr = depthErr;
}


void BuoyancyControlUnit::setEncoderPosition(float setPosIn) {

	setPos = setPosIn;
	if(setPos < 0){ setPos = 0; }
	curPos = bcuEncoder.getPulses();
	posErr = setPos - curPos;
	float posdt = timer.read() - posLastTime;

	posInt += posErr * posdt;
	posDer = (posErr - prevPosErr)/posdt;

	VRef = KpEnc * posErr + KiEnc * posInt + KdEnc * posDer;
//	VRef = KpEnc * posErr;

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
		if(posErr > 30 || posErr < -30) { // TODO: determine appropriate tolerance
			this->setV(VRef);
		} else {
			bcuDirA = 0.0;
			bcuDirB = 0;
		}
	}
}

void BuoyancyControlUnit::setV(float Vin){
	float setV;

	if(Vin > 0) {
		bcuDir = 0;
		setV = Vin;
	} else if(Vin < 0) {
		bcuDir = 1;
		setV = -1.0*Vin;
	}

	//bcuDirA = bcuDir;
	//bcuDirB = !bcuDir;

	if(setV > 1) {
		setV = 1.0;
	} else if(setV < 0.08 ) {
		setV = 0.08;
	} 
	
	if(bcuDir == 1){
		bcuDirA = setV;
		bcuDirB = 0.0;
	} else if(bcuDir == 0){
		bcuDirA = 0.0;
		bcuDirB = setV;
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

