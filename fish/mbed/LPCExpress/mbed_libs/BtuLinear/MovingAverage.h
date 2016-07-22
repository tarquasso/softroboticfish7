#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include "mbed.h"

#define MOVING_AVERAGE_SIZE 10

class MovingAverage {

public:
	MovingAverage();
	~MovingAverage();
	void reset();
	float computeMovingAverage(float newestValue);

private:

	static const int s_numReadings = MOVING_AVERAGE_SIZE;
	float m_readings[s_numReadings];      // the readings from the analog input
	int m_readIndex;              // the index of the current reading
	float m_total;                  // the running total
	float m_average;                // the average
	bool bufferFilled;

};

#endif
