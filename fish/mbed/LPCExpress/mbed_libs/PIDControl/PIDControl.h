#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "mbed.h"

class PID {
 public:
  PID(float Kc, float tauI, float tauD, float interval, float inMin, float inMax, float outMin, float outMax, float bias);
  void setSetPoint(float sp);
  void setProcessValue(float pv);
  float compute();

 private:
  void setInterval(float interval);
  void setTunings(float Kc, float tauI, float tauD);
  void setInputLimits(float inMin, float inMax);
  void setOutputLimits(float outMin, float outMin);

  float Kc_, tauI_, tauD_, sampleTime_, setPoint_, processVar_, integral_, bias_, errorPrior_, inMin_, inMax_, outMin_, outMax_;

}

#endif
