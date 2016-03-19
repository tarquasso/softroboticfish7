#include <PumpWithValve/PumpWithValve.h>

// The static instance
PumpWithValve pumpWithValve;

void flipFlowDirectionStatic()
{
	pumpWithValve.flipFlowDirection();
}

//============================================
// Initialization
//============================================

// Constructor
PumpWithValve::PumpWithValve() :
    		// Initialize variables
    		pumpPWM(pumpPwmPin),
			valvePWM(valvePwmPin),
			//valveEncoder(encoderPinA, encoderPinB, NC, count2rev), // use X2 encoding by default
			//valveCurrent(valveCurrentPin),
			hallSignal(hallInterruptPin)
{
	hallSignal.rise(&flipFlowDirectionStatic); //TODO: check if that is how the hall sensor is used by Alex

	frequency = 0;
	thrust = 0;
	yaw = 0;

	valve_side = true; //true is the first side
	valveV1 = 0;
	valveV2 = 0;
	Vfreq = 0;
	VfreqAdjusted = 0;
//	dV_freq = 0;
//	freq_error = 0;
//	prev_freq_error = 0;
}


void PumpWithValve::start()
{
	valvePWM.write(0.0); // apply nothing to start
	pumpPWM.write(0.0);
	timer.start();
}

void PumpWithValve::stop()
{
	valvePWM.write(0.0);
	pumpPWM.write(0.0);
}

void PumpWithValve::flipFlowDirection()
{
	// when the hall sensor sees a rising edge, we have rotated 180 degrees
	// --> want to adjust the applied voltage based on the side we are on
	valve_side = !valve_side; //change boolean state, keeps track of which half of the rotation we are on
}

//============================================
// Processing
//============================================
void PumpWithValve::set(float freq_in, float yaw_in, float thrust_in)
{
	//Centrifugal Pump
	thrust = thrust_in; //thrust comes in as float from 0.0-1.0
	pumpPWM.write(thrust);

	// set speed of the valve motor through the frequency value
	frequency = freq_in;
	//   TODO: NOT YET CONTROLLING ACTUAL FREQUENCY - JUST SET VOLTAGE
	//    // measure current tail frequency
	//    dt = timer.read_us();
	//    timer.reset();
	//    rot = valveEncoder.getPulses()/count2rev; // TODO also need to divide by gear ratio if encoder is attached directly to motor
	//	  valveEncoder.reset();
	//    freq_act = rot/dt;
	//    freq_error = frequency - freq_act;
	//    // update tail frequency
	//    dV_freq = freq_PGain * freq_error + freq_DGain * (freq_error - prev_freq_error);
	//    prev_freq_error = freq_error; //reset previous frequency error
	//    Vfreq += dV_freq;
	Vfreq = frequency; //just setting directly the voltage

	yaw = yaw_in; //update yaw state using input
	//this->calculateYawMethod1();
	this->calculateYawMethod2();
}


void PumpWithValve::calculateYawMethod1()
{
	// split tail frequency voltage into voltage on either side of the valve
	// TODO figure out ideal relationship between yaw and offset between V1 and V2
	// is it additive or multiplicative? start with super simple math

	valveV1 = (1 + valveOffsetGain * yaw) * Vfreq;
	valveV2 = (1 - valveOffsetGain * yaw) * Vfreq;

	// TODO need to decide whether to give up frequency or yaw when we are at input limits
	if (valveV1 > 1.0) {valveV1 = 1.0;}
	else if (valveV1 < 0.0) {valveV1 = 0.0;}
	if (valveV2 > 1.0) {valveV2 = 1.0;}
	else if (valveV2 < 0.0) {valveV2 = 0.0;}

	// write valve voltage based on which side the hall sensor says we are on
	// TODO: directions are arbitrary at the moment, yaw is depending on hall sensor initial input
	if (valve_side)
		valvePWM.write(valveV1);
	else
		valvePWM.write(valveV2);

}

void PumpWithValve::calculateYawMethod2()
{

		if(yaw < 0.0 && !valve_side)
			VfreqAdjusted = (1.0 + valveOffsetGain*yaw)*Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		else if(yaw > 0.0 && valve_side)
			VfreqAdjusted = (1.0 - valveOffsetGain*yaw)*Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		else
			VfreqAdjusted = Vfreq;
		valvePWM.write(VfreqAdjusted);

}
