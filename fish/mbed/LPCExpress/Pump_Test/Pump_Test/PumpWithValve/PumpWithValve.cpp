#include <PumpWithValve.h>

// The static instance
PumpWithValve pumpWithValve;

void flipFlowUpStatic()
{
	pumpWithValve.flipFlowUp();
}

void flipFlowDownStatic()
{
	pumpWithValve.flipFlowDown();
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
			hallSignal(hallInterruptPin),
			valveLED(LED2)
{
	running = false;

	hallSignal.rise(&flipFlowUpStatic);
	hallSignal.fall(&flipFlowDownStatic);

	frequency = 0;
	thrust = 0;
	yaw = 0;

	periodSide1 = 0;
	periodSide2 = 0;

	valveSide = true;
	valveV1 = 0;
	valveV2 = 0;
	Vfreq = 0;
	VfreqAdjusted = 0;
	Vset = 0;
	dVFreq = 0;
	freqErr = 0;
	prevFreqErr = 0;
}


void PumpWithValve::start()
{
	running = true;
	valvePWM.write(0.0); // apply nothing to start
	pumpPWM.write(0.0);
	timer.start();
}

void PumpWithValve::stop()
{
	running = false;
	valvePWM.write(0.0);
	pumpPWM.write(0.0);
}

void PumpWithValve::flipFlowUp()
{
	// when the hall sensor sees a rising edge, we have rotated 180 degrees
	// --> want to adjust the applied voltage based on the side we are on
	valveSide = true; //change boolean state, keeps track of which half of the rotation we are on
	valveLED = 1;
	periodSide1 = timer.read_us();
	timer.reset();
	freqAct = 1/(periodSide1 + periodSide2);
}

void PumpWithValve::flipFlowDown()
{
	valveSide = false;
	valveLED = 0;
	periodSide2 = timer.read_us();
	timer.reset();
	freqAct = 1/(periodSide1 + periodSide2);
}

//============================================
// Processing
//============================================
void PumpWithValve::set(float freq_in, float yaw_in, float thrust_in)
{
	if(running == true){
		// Failure mode - if it has been a full (desired) period since a hall sensor has been read
		if(timer.read_us() > 1 / frequency){
			pumpWithValve.stop(); // may have to add a condition that allows for sudden input changes
		}

		//Centrifugal Pump
		thrust = thrust_in; //thrust comes in as float from 0.0-1.0
		pumpPWM.write(thrust);

		// set speed of the valve motor through the frequency value
		frequency = freq_in;
		if(periodSide1 != 0 && periodSide2 != 0){ // don't be fooled by initialization values
			freqErr = frequency - freqAct;
			dVFreq = KpFreq * freqErr + KdFreq * (freqErr - prevFreqErr);
			prevFreqErr = freqErr; //reset previous frequency error
			Vfreq += dVFreq;
		} else {
			Vfreq = frequency * 300000; //just setting directly the voltage, scaled up; need to tune this value
		}

		yaw = yaw_in; //update yaw state using input
		this->calculateYawMethod1();
		//this->calculateYawMethod2();
	}
}


void PumpWithValve::calculateYawMethod1()
{
	// split tail frequency voltage into voltage on either side of the valve
	// TODO figure out ideal relationship between yaw and offset between V1 and V2
	// is it additive or multiplicative? start with super simple math

	valveV1 = (1.0 + valveOffsetGain * yaw) * Vfreq;
	valveV2 = (1.0 - valveOffsetGain * yaw) * Vfreq;

	// TODO need to decide whether to give up frequency or yaw when we are at input limits
	if (valveV1 > 1.0) {valveV1 = 1.0;}
	else if (valveV1 < 0.0) {valveV1 = 0.05;}
	if (valveV2 > 1.0) {valveV2 = 1.0;}
	else if (valveV2 < 0.0) {valveV2 = 0.05;}

	// write valve voltage based on which side the hall sensor says we are on
	if (valveSide) { Vset = valveV1; }
	else { Vset = valveV2; }

	valvePWM.write(Vset);
}

void PumpWithValve::calculateYawMethod2()
{

	if (yaw < 0.0 && !valve_side) {
		Vset = (1.0 + valveOffsetGain*yaw)*Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Vset > 1.0) { VfreqAdjusted = 1.0; }
	} 
	else if (yaw > 0.0 && valve_side) {
		Vset = (1.0 - valveOffsetGain*yaw)*Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Vset < 0.0) { VfreqAdjusted = 0.05; } // needs to keep turning
	}
	else {
		Vset = Vfreq;
		VfreqAdjusted = Vfreq;
	}
	valvePWM.write(VfreqAdjusted);

}

float PumpWithValve::getVset() { return Vset;}
bool PumpWithValve::getVside() { return valve_side; }
bool PumpWithValve::checkRunState() { return running; }
