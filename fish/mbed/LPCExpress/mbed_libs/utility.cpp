//utility.cpp

#include "utility.h"

namespace utility {

float clip(float val, float min, float max) {
	float newVal = (val > max) ? max : val;
	return (newVal < min) ? min : newVal;
}

float deadzone(float val, float threshold) {
	// get absolute value for dead zone
	float absVal = (val >= 0) ? val : -val;
	// check if within threshold
	if (absVal <= threshold) {
		return 0;
	} else {
		return val;
	}

}

}
