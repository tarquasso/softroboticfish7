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
	prevControllerOutput_ = 0.0;
	error_ = 0.0;
	derivative_ = 0.0;
	output_ = 0.0;
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
	outMin_ = outMin;
	outMax_ = outMax;
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
	// Calculate errotr between the set point and the process variable
	error_ = setPoint_ - processVar_;

	// anti-windup, if at either min or max limit, while error as in the old PID library, but rewritten to handle our bounds
	if (!(prevControllerOutput_ >= outMax_ && error_ > 0)
			&& !(prevControllerOutput_ <= outMin_ && error_ < 0)) {
		integral_ += (error_ * sampleTime_);
	}
	derivative_ = (error_ - errorPrior_) / sampleTime_;
	output_ = (Kc_ * error_) + (Ki_ * integral_) + (Kd_ * derivative_) + bias_;
	errorPrior_ = error_;

	prevControllerOutput_ = utility::clip(output_, outMin_, outMax_);

	return prevControllerOutput_;
}
