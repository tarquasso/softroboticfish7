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
	depth_cmd = 0; // input command
	depth_act = 0; // actual depth based on sensor
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

void BuoyancyControlUnit::set(float depth_in)
{

	depth_cmd = depth_in;

	//ASSUMES that "pitch" signal in is from -1.0 to 1.0
	if (depth_in < 0.2 && depth_in > -0.2)
		Vref = 0;
	else
		Vref = depth_in;

	//Reverse the direction of the motor depending on the command
	if(Vref >= 0)
	{
		bcuDirA.write(0);
		bcuDirB.write(1);
		bcuPWM.write(Vref);
	}
	else
	{
		bcuDirA.write(1);
		bcuDirB.write(0);
		bcuPWM.write(-1 * Vref);
	}
}


