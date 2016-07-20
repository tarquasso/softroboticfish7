#include "PidController.h"
#include "utility.h"

PidController::PidController(float Kc, float Ki, float Kd, float interval,
		float inMin, float inMax, float outMin, float outMax, float bias) {
	// Set Up PID Parameters
	this->setTunings(Kc, Ki, Kd);
	this->setInputLimits(inMin, inMax);
	this->setOutputLimits(outMin, outMax);
	this->setInterval(interval);
	this->setBias(bias);
	reset();
}

void PidController::reset() {
	setPoint_ = 0.0;
	processVar_ = 0.0;
	integral_ = 0.0;
	errorPrior_ = 0;
	prevControlOut_ = 0.0;
	error_ = 0.0;
	derivative_ = 0.0;
	controlOut_ = 0.0;
}

void PidController::setInputLimits(float inMin, float inMax) {

	//Make sure we haven't been given impossible values.
	if (inMin >= inMax) {
		return;
	}

	//Rescale the working variables to reflect the changes

	inMin_ = inMin;
	inMax_ = inMax;
}

void PidController::setOutputLimits(float outMin, float outMax) {

	//Make sure we haven't been given impossible values.
	if (outMin >= outMax) {
		return;
	}

	//Rescale the working variables to reflect the changes.
	controlOutMin_ = outMin;
	controlOutMax_ = outMax;
}

void PidController::setTunings(float Kc, float Ki, float Kd) {
	//Verify that the tunings make sense.
	if (Kc == 0.0 || Ki < 0.0 || Kd < 0.0) {
		return;
	}

	if ((Kc != Kc_) || (Ki != Ki_) || (Kd != Kd_)) {
		reset();
	}

	Kc_ = Kc;
	Ki_ = Ki;
	Kd_ = Kd;
}

void PidController::setInterval(float interval) {
	sampleTime_ = interval;
}

void PidController::setSetPoint(float sp) {
	setPoint_ = utility::clip(sp, inMin_, inMax_);
}

void PidController::setProcessValue(float pv) {
	processVar_ = utility::clip(pv, inMin_, inMax_);
}

void PidController::setBias(float b) {
	bias_ = b;
}

float PidController::compute() {
	// Calculate process error between the set point and the process variable
	error_ = setPoint_ - processVar_;

	// anti-windup, if at either min or max limit, while error is
	if (!(prevControlOut_ >= controlOutMax_ && error_ > 0)
			&& !(prevControlOut_ <= controlOutMin_ && error_ < 0)) {
		integral_ += (error_ * sampleTime_);
	}
	// Calculate Derivative of the process error
	derivative_ = (error_ - errorPrior_) / sampleTime_;

	// Store Error Value for Derivative Gain
	errorPrior_ = error_;

	// Calculate Commanded Output Value from the Controller
	controlOut_ = (Kc_ * error_) + (Ki_ * integral_) + (Kd_ * derivative_) + bias_;


	prevControlOut_ = utility::clip(controlOut_, controlOutMin_, controlOutMax_);

	return prevControlOut_;
}
