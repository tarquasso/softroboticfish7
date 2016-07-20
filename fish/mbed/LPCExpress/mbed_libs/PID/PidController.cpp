#include "PidController.h"
#include "utility.h"

PidController::PidController(float Kc, float tauI, float tauD, float interval,
		float inMin, float inMax, float outMin, float outMax, float b) {
	setInputLimits(inMin, inMax);
	setOutputLimits(outMin, outMax);

	sampleTime_ = interval;

	setTunings(Kc, tauI, tauD);

	reset();

}

void PidController::reset() {
	setPoint_ = 0.0;
	processVar_ = 0.0;
	integral_ = 0.0;
	errorPrior_ = 0;
	prevControllerOutput_ = 0.0;
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

void PidController::setTunings(float Kc, float tauI, float tauD) {
	//Verify that the tunings make sense.
	if (Kc == 0.0 || tauI < 0.0 || tauD < 0.0) {
		return;
	}

	if ((Kc != Kc_) || (tauI != tauI_) || (tauD != tauD_)) {
		reset();
	}

	Kc_ = Kc;
	tauI_ = tauI;
	tauD_ = tauD;
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
	reset();
}

float PidController::compute() {
	float error = setPoint_ - processVar_;
	// same anti-windup as in the old PID library, but rewritten to handle our bounds
	if (!(prevControllerOutput_ >= outMax_ && error > 0)
			&& !(prevControllerOutput_ <= outMin_ && error < 0)) {
		integral_ += (error * sampleTime_);
	}
	float derivative = (error - errorPrior_) / sampleTime_;
	float output = (Kc_ * error) + (tauI_ * integral_) + (tauD_ * derivative)
			+ bias_;
	errorPrior_ = error;

	prevControllerOutput_ = utility::clip(output, outMin_, outMax_);

	return prevControllerOutput_;
}
