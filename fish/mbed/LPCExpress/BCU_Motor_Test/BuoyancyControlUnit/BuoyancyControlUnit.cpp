#include <BuoyancyControlUnit/BuoyancyControlUnit.h>

// The static instance
BuoyancyControlUnit buoyancyControlUnit;

//============================================
// Initialization
//============================================

// Constructor
BuoyancyControlUnit::BuoyancyControlUnit() :
    				// Initialize variables
    				// pressureSensor(p28,p27),
    				bcuPWM(bcuPwmPin),
					bcuDirA(bcuDirAPin),
					bcuDirB(bcuDirBPin),
					bcuCurrent(bcuCurrentPin),
					bcuEncoder(encoderPinA, encoderPinB, NC, count2rev*gearRatio, QEI::X4_ENCODING)
{
	depthDes = 0; // input command
	depthMeas = 0; // actual depth based on sensor

	depthLastTime = 0.0;
	prevDepthErr = 0.0;
	depthInt = 0.0;

	posLastTime = 0.0;
	prevPosErr = 0.0;
	posInt = 0.0;

	//pressureSensor.MS5837Init();

	VRef = 0;
//	dV_in = 0;
//	depth_error = depth_cmd - depth_act;
//	prev_depth_error = 0; // in case we do PD control

}


void BuoyancyControlUnit::start()
{
	bcuPWM.write(0.0); // apply nothing to start
	bcuEncoder.reset(); // gives it a reference to return to
	timer.start();
}

void BuoyancyControlUnit::stop()
{
	//bcuPWM.write(0.0);
	// return bcu back to start position
	this->setEncoderPosition(0);
}

//============================================
// Processing
//============================================
void BuoyancyControlUnit::set(float depthDesIn, float depthMeasIn)
{
	depthDes = depthDesIn;
	depthMeas = depthMeasIn;
	// pressureSensor.Barometer_MS5837();
	// depthMeas = pressureSensor.MS5837_Pressure();
	depthErr = depthDes - depthMeas;

/*	if (depthDiff < 15 && depthDiff > -15)
		VRef = 0;
	else
		VRef = depthDiff / 150; // arbitrary scaling factor...

	//Reverse the direction of the motor depending on the command
	if(VRef >= 0)
	{
		bcuDir = 0;
		bcuDirA = bcuDir;
		bcuDirB = !bcuDir;
		if(VRef > 1) {bcuPWM.write(1.0);}
		else {bcuPWM.write(VRef);}
	}
	else
	{
		bcuDir = 1;
		bcuDirA = bcuDir;
		bcuDirB = !bcuDir;
		if(VRef < -1){bcuPWM.write(1.0);}
		else {bcuPWM.write(-1 * VRef);}
	}
*/
	float depthdt = timer.read_ms() - depthLastTime;

	depthInt += depthErr * depthdt;
	depthDer = (depthErr - prevDepthErr)/depthdt;

	posRef += KpDepth*depthErr + KiDepth*depthInt + KdDepth*depthDer;

	depthLastTime = timer.read_ms();
	prevDepthErr = depthErr;

	this->setEncoderPosition(posRef);
}


void BuoyancyControlUnit::setEncoderPosition(float setPosIn) {
	setPos = setPosIn;
	currPos = bcuEncoder.getPulses();
	posErr = setPos - currPos;
	float posdt = timer.read_ms() - posLastTime;

	posInt += posErr * posdt;
	posDer = (posErr - prevPosErr)/posdt;

	VRef += KpEnc * posErr + KiEnc * posInt + KdEnc * posDer;

	posLastTime = timer.read_ms();
	prevPosErr = posErr;

	if(posErr < 15 && posErr > -15) { // TODO: determine appropriate tolerance
		if(VRef >= 0) {
			bcuDir = 0;
			bcuDirA = bcuDir;
			bcuDirB = !bcuDir;
			if(VRef > 1) {bcuPWM.write(1.0);}
			else {bcuPWM.write(VRef);}
		} else {
			bcuDir = 1;
			bcuDirA = bcuDir;
			bcuDirB = !bcuDir;
			if(VRef < -1){bcuPWM.write(1.0);}
			else {bcuPWM.write(-1 * VRef);}
		}
	}
}

bool BuoyancyControlUnit::getBCUdir() { return bcuDir; }
float BuoyancyControlUnit::getVset() { return VRef; }


