#include "CyclicActuator.h"

// The static instance
CyclicActuator cyclicActuator;

void flipFlowUpStatic() {
	cyclicActuator.flipFlowUp();
}

void flipFlowDownStatic() {
	cyclicActuator.flipFlowDown();
}

//============================================
// Initialization
//============================================

// Constructor
CyclicActuator::CyclicActuator() :
		// Initialize variables
		pumpPWM(pumpPwmPin), valvePWM(valvePwmPin),
		//valveEncoder(encoderPinA, encoderPinB, NC, count2rev), // use X2 encoding by default
		//valveCurrent(valveCurrentPin),
		hallSignal(hallInterruptPin), valveLED(LED2) {
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
	setValvePwm = 77;
	setPumpPwm = 77;

	running = false;
}

void CyclicActuator::start() {
	valvePWM.write(0.2); // apply nothing to start
	pumpPWM.write(0.2);
	periodSide1 = 0;
	periodSide2 = 0;

	timer.reset();
	valveControl.attach(&cyclicActuator, &CyclicActuator::setVoid, 0.2);

	running = true;
}

void CyclicActuator::stop() {
	valveControl.detach();
	thrust = 0;
	Vfreq = 0;
	valvePWM.write(0.0);
	pumpPWM.write(0.0);

	running = false;
}

void CyclicActuator::flipFlowUp() {
	// when the hall sensor sees a rising edge, we have rotated 180 degrees
	// --> want to adjust the applied voltage based on the side we are on
	valveSide = true; //change boolean state, keeps track of which half of the rotation we are on
	valveLED = 1;
	periodSide1 = timer.read_us();
	timer.reset();
	freqAct = 1 / (periodSide1 + periodSide2);
}

void CyclicActuator::flipFlowDown() {
	valveSide = false;
	valveLED = 0;
	periodSide2 = timer.read_us();
	timer.reset();
	freqAct = 1 / (periodSide1 + periodSide2);
}

//============================================
// Processing
//============================================
void CyclicActuator::set(float freq_in, float yaw_in, float thrust_in) {
	thrust = thrust_in;
	yaw = yaw_in;
	frequency = freq_in;
	thrust = 555;
}

void CyclicActuator::setVoid() {
	//Centrifugal Pump
	pumpPWM.write(thrust);
	setPumpPwm = thrust;

	this->calculateYawMethod1();
}

void CyclicActuator::calculateYawMethod1() {
	valvePWM.write(frequency);
	setValvePwm +=1;
}

void CyclicActuator::calculateYawMethod2() {

	if (yaw < 0.0 && !valveSide) {
		Vset = (1.0 + valveOffsetGain * yaw) * Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Vset > 1.0) {
			VfreqAdjusted = 1.0;
		}
	} else if (yaw > 0.0 && valveSide) {
		Vset = (1.0 - valveOffsetGain * yaw) * Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Vset < 0.0) {
			VfreqAdjusted = 0.05;
		} // needs to keep turning
	} else {
		Vset = Vfreq;
		VfreqAdjusted = Vfreq;
	}
	valvePWM.write(VfreqAdjusted);
	setValvePwm = VfreqAdjusted;

}

float CyclicActuator::getVset() {
	return Vset;
}
bool CyclicActuator::getVside() {
	return valveSide;
}
bool CyclicActuator::getRunState() {
	return running;
}
float CyclicActuator::getSensorTime() {
	return timer.read();
}
int CyclicActuator::getPeriod1() {
	return periodSide1;
}
int CyclicActuator::getPeriod2() {
	return periodSide2;
}
float CyclicActuator::getCurFreq() {
	return freqAct * 1000000;
}
float CyclicActuator::getSetFreq() {
	return frequency * 1000000;
}
float CyclicActuator::getValvePwm() {
	return setValvePwm;
}
float CyclicActuator::getPumpPwm() {
	return thrust;
}
