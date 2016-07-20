//utility.cpp

#include "utility.h"

namespace utility {

float clip(float val, float min, float max) {
	float newVal = (val > max) ? max : val;
	return (newVal < min) ? min : newVal;
}

float pid_clip(float val, float min, float max) {
  float newval = val;
  newval = (newval > max) ? max : newval;
  newval = (newval < min) ? min : newval;
  return newval;
}

}
