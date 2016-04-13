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
					bcuCurrent(bcuCurrentPin)
{
	depthDes = 0; // input command
	depthMeas = 0; // actual depth based on sensor
	Vref = 0;
//	dV_in = 0;
//	depth_error = depth_cmd - depth_act;
//	prev_depth_error = 0; // in case we do PD control

}


void BuoyancyControlUnit::start()
{
	bcuPWM.write(0.0); // apply nothing to start
}

void BuoyancyControlUnit::stop()
{
	bcuPWM.write(0.0);
}

//============================================
// Processing
//============================================
//void BuoyancyControlUnit::set(float depth_in)
//{
//	depth_cmd = depth_in;
//	depth_act = 0; // TODO update value from sensor
//	depth_error = depth_cmd - depth_act;
//
//	dV_in = depth_PGain * depth_error + depth_DGain * (depth_error - prev_depth_error);
//
//	if ((dV_in > 0 && bcuCurrent.read() < maxBCUCurrent) || (dV_in < 0 && bcuCurrent.read() > minBCUCurrent))
//	{
//		Vref += dV_in;
//	} // don't update Vref if it requires going over max or under min
//
//	bcuPWM.write(Vref);
//
//	prev_depth_error = depth_error;
//}

void BuoyancyControlUnit::set(float depthDesIn, float depthMeasIn)
{

	depthDes = depthDesIn;
	depthMeas = depthMeasIn;
	float depthDiff = depthDes - depthMeas;


	if (depthDiff < 15 && depthDiff > -15)
		Vref = 0;
	else
		Vref = depthDiff / 150; // arbitrary scaling factor...

	//Reverse the direction of the motor depending on the command
	if(Vref >= 0)
	{
		bcuDir = 0;
		bcuDirA = bcuDir;
		bcuDirB = !bcuDir;
		if(Vref > 1) {bcuPWM.write(1.0);}
		else {bcuPWM.write(Vref);}
	}
	else
	{
		bcuDir = 1;
		bcuDirA = bcuDir;
		bcuDirB = !bcuDir;
		if(Vref < -1){bcuPWM.write(1.0);}
		else {bcuPWM.write(-1 * Vref);}
	}


}

bool BuoyancyControlUnit::getBCUdir() { return bcuDir; }
float BuoyancyControlUnit::getVset() { return Vref; }


