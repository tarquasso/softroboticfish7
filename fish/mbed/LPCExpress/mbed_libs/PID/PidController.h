#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "mbed.h"

class PidController {
public:
	PidController(float Kc, float Ki, float Kd, float interval, float inMin,
			float inMax, float outMin, float outMax, float bias);
	void setTunings(float Kc, float Ki, float Kd);
	void reset();
	void setSetPoint(float sp);
	void setProcessValue(float pv);
	void setBias(float b);
	float compute();

private:
	void setInterval(float interval);
	void setInputLimits(float inMin, float inMax);
	void setOutputLimits(float outMin, float outMax);

	float Kc_, Ki_, Kd_, sampleTime_, setPoint_, processVar_, integral_,
			bias_, errorPrior_, inMin_, inMax_, outMin_, outMax_,
			prevControllerOutput_, error_, derivative_, output_;

};

#endif
