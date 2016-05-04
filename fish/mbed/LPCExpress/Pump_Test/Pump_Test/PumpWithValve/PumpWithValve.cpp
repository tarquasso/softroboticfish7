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
	hallSignal.rise(&flipFlowUpStatic);
	hallSignal.fall(&flipFlowDownStatic);
	timer.start();

	frequency = 0.0000009;
	thrust = 0;
	yaw = 0;

	valveSide = true;
	valveV1 = 0;
	valveV2 = 0;
	Vfreq = 0;
	VfreqAdjusted = 0;
	Vset = 0;
	dVFreq = 0;
	freqErr = 0;
	prevFreqErr = 0;

	running = false;
}


void PumpWithValve::start()
{
	valvePWM.write(0.0); // apply nothing to start
	pumpPWM.write(0.0);
	periodSide1 = 0;
	periodSide2 = 0;

	timer.reset();
	valveControl.attach(&pumpWithValve, &PumpWithValve::setVoid, 0.08);

	running = true;
}

void PumpWithValve::stop()
{
	valveControl.detach();
	thrust = 0;
	Vfreq = 0;
	valvePWM.write(0.0);
	pumpPWM.write(0.0);

	running = false;
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
void PumpWithValve::set(float freq_in, float yaw_in, float thrust_in){
	thrust = thrust_in;
	yaw = yaw_in;
	frequency = freq_in;
}

void PumpWithValve::setVoid(){
	//Centrifugal Pump
	pumpPWM.write(thrust);

	// set speed of the valve motor through the frequency value
	if(periodSide1 == 0 || periodSide2 == 0) {
		Vfreq = frequency * 400000; //just setting directly the voltage, scaled up; need to tune this value
		this->calculateYawMethod1();
	} else { // don't be fooled by initialization values
		// Failure mode - if it has been a full (desired) period since a hall sensor has been read
		if(timer.read_us() > 1.0 / frequency){
			pumpWithValve.stop(); // may have to add a condition that allows for sudden input changes
		} else {
		freqErr = frequency - freqAct;
		dVFreq = KpFreq * freqErr + KdFreq * (freqErr - prevFreqErr);
		prevFreqErr = freqErr; //reset previous frequency error
		Vfreq += dVFreq;
		this->calculateYawMethod1();
		}
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

	if (yaw < 0.0 && !valveSide) {
		Vset = (1.0 + valveOffsetGain*yaw)*Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Vset > 1.0) { VfreqAdjusted = 1.0; }
	} 
	else if (yaw > 0.0 && valveSide) {
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
bool PumpWithValve::getVside() { return valveSide; }
bool PumpWithValve::getRunState() { return running; }
float PumpWithValve::getSensorTime() { return timer.read(); }
int PumpWithValve::getPeriod1() { return periodSide1; }
int PumpWithValve::getPeriod2() { return periodSide2; }
float PumpWithValve::getCurFreq() { return freqAct * 1000000; }
float PumpWithValve::getSetFreq() { return frequency * 1000000; }
