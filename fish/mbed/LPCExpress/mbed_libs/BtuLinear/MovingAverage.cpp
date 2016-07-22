#include "MovingAverage.h"

MovingAverage::MovingAverage() {
	this->reset();
}

MovingAverage::~MovingAverage() {};

void MovingAverage::reset() {
	// reset all the readings to 0:
	for (int i = 0; i < s_numReadings; i++) {
		m_readings[i] = 0;
	}
	m_readIndex = 0;
	m_total = 0;                  // the running total
	m_average = 0;
	bufferFilled = false;
}


float MovingAverage::computeMovingAverage(float newestValue) {
	// subtract the last reading:
	m_total = m_total - m_readings[m_readIndex];
	// read from the sensor:
	m_readings[m_readIndex] = newestValue;
	// add the reading to the total:
	m_total = m_total + m_readings[m_readIndex];
	// advance to the next position in the array:
	m_readIndex = m_readIndex + 1;
	// if we're at the end of the array...
	if (m_readIndex >= s_numReadings) {
		// ...wrap around to the beginning:
		m_readIndex = 0;
		if (!bufferFilled) {
			bufferFilled = true;
		}
	}

	// calculate the average:
	if (bufferFilled) {
		m_average = m_total / s_numReadings;
	} else {
		m_average = m_total / (m_readIndex + 1);
	}
	return m_average;

}
