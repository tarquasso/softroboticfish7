#include "CyclicActuatorSync.h"

// The static instance
CyclicActuatorSync cyclicActuator;

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
CyclicActuatorSync::CyclicActuatorSync() :
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

void CyclicActuatorSync::start() {
	valvePWM.write(0.2); // apply nothing to start
	pumpPWM.write(0.2);
	periodSide1 = 0;
	periodSide2 = 0;

	timer.reset();
	// valveControl.attach(&cyclicActuator, &CyclicActuatorSync::setVoid, 0.2);

	running = true;
}

void CyclicActuatorSync::stop() {
	valveControl.detach();
	thrust = 0;
	Vfreq = 0;
	valvePWM.write(0.0);
	pumpPWM.write(0.0);

	running = false;
}

void CyclicActuatorSync::flipFlowUp() {
	// when the hall sensor sees a rising edge, we have rotated 180 degrees
	// --> want to adjust the applied voltage based on the side we are on
	valveSide = true; //change boolean state, keeps track of which half of the rotation we are on
	valveLED = 1;
	periodSide1 = timer.read_us();
	timer.reset();
	freqAct = 1 / (periodSide1 + periodSide2);
}

void CyclicActuatorSync::flipFlowDown() {
	valveSide = false;
	valveLED = 0;
	periodSide2 = timer.read_us();
	timer.reset();
	freqAct = 1 / (periodSide1 + periodSide2);
}

//============================================
// Processing
//============================================
void CyclicActuatorSync::set(float freq_in, float yaw_in, float thrust_in) {
	thrust = thrust_in;
	yaw = yaw_in;
	frequency = freq_in;
}


void CyclicActuatorSync::setVoid() {
	//Centrifugal Pump
	pumpPWM.write(thrust);
	setPumpPwm = thrust;

	calculateYawMethod2();
}

void CyclicActuatorSync::calculateYawMethod1() {
	// valvePWM.write(frequency);
	// setValvePwm = frequency;
  float Pset;
  float PfreqAdjusted;
  	if (yaw < 0.0 && !valveSide) {
		Pset = (1.0 + pumpOffsetGain * yaw) * Pfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Pset > max_P) {
			PfreqAdjusted = max_P;
		}
        if (Pset < min_P) {
          PfreqAdjusted = min_P;
        }
	} else if (yaw > 0.0 && valveSide) {
		Pset = (1.0 - pumpOffsetGain * yaw) * Pfreq; // 0.7 can be adjusted to a power of 2 if needed
        if (Pset > max_P) {
          PfreqAdjusted = max_P;
        }
		if (Pset < 0.05) {
			PfreqAdjusted = min_P;
		} // needs to keep turning
	} else {
		Pset = Pfreq;
		PfreqAdjusted = Pfreq;
	}
	pumpPWM.write(PfreqAdjusted);
	setPumpPwm = PfreqAdjusted;
}

void CyclicActuatorSync::calculateYawMethod2() {

	if (yaw < 0.0 && !valveSide) {
		Vset = (1.0 + valveOffsetGain * yaw) * Vfreq; // 0.7 can be adjusted to a power of 2 if needed
		if (Vset > 1.0) {
			VfreqAdjusted = 1.0;
		}
        if (Vset < 0.05) {
          VfreqAdjusted = 0.05;
        }
	} else if (yaw > 0.0 && valveSide) {
		Vset = (1.0 - valveOffsetGain * yaw) * Vfreq; // 0.7 can be adjusted to a power of 2 if needed
        if (Vset > 1.0) {
          VfreqAdjusted = 1.0;
        }
		if (Vset < 0.05) {
			VfreqAdjusted = 0.05;
		} // needs to keep turning
	} else {
		Vset = Vfreq;
		VfreqAdjusted = Vfreq;
	}
	valvePWM.write(VfreqAdjusted);
	setValvePwm = VfreqAdjusted;

}

float CyclicActuatorSync::getVset() {
	return Vset;
}
bool CyclicActuatorSync::getVside() {
	return valveSide;
}
bool CyclicActuatorSync::getRunState() {
	return running;
}
float CyclicActuatorSync::getSensorTime() {
	return timer.read();
}
int CyclicActuatorSync::getPeriod1() {
	return periodSide1;
}
int CyclicActuatorSync::getPeriod2() {
	return periodSide2;
}
float CyclicActuatorSync::getCurFreq() {
	return freqAct * 1000000;
}
float CyclicActuatorSync::getSetFreq() {
	return frequency * 1000000;
}
float CyclicActuatorSync::getValvePwm() {
	return setValvePwm;
}
float CyclicActuatorSync::getPumpPwm() {
	return thrust;
}
