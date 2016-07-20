//utility.cpp

#include "utility.h"

namespace utility {

float clip(float val, float min, float max) {
	float newVal = (val > max) ? max : val;
	return (newVal < min) ? min : newVal;
}

}
