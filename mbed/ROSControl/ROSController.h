
#ifndef ROSCONTROL_ROSCONTROLLER_H_
#define ROSCONTROL_ROSCONTROLLER_H_

#include "FishController.h"
#include "mbed.h"
#include <ros.h>
#include <fish_msgs/joystick_in.h>

#define defaultBaudUSB 115200
#define defaultBaud 115200
#define defaultTX p13
#define defaultRX p14
// note lowBatteryVoltagePin is defined in FishController

#define printStatusSerialController // whether to print what's going on (i.e. when it gets commands, etc.)
#define debugLEDsSerial // LED1: initialized LED2: running LED3: receiving a character LED4: done (others turn off)
#define runTime 10000 	   // how long to run for (in milliseconds) if inifiniteLoopSerial is undefined
#define infiniteLoopSerial // if defined, will run forever (or until stop() is called from another thread)

#define serialControllerControlFish // whether to start fishController to control the servos and motor

// Map bytes sent over serial (1-255) to ranges for each fish property
#define serialMinPitch     ((float)(0.2))
#define serialMaxPitch     ((float)(0.8))
#define serialMinYaw       ((float)(-1.0))
#define serialMaxYaw       ((float)(1.0))
#define serialMinThrust    ((float)(0.0))
#define serialMaxThrust    ((float)(0.75))
#define serialMinFrequency ((float)(0.0000009))
#define serialMaxFrequency ((float)(0.0000016))

class ROSController
{
public:
	// Initialization
	ROSController();
	void init();
	// Execution control
	void run();
	void stop();
	void lowBatteryCallback();
	// void processSerialWord(uint8_t* word);
	void processSerialWord(const fish_msgs::joystick_in& cmd_msg);

private:
	Timer programTimer;
	bool terminated;

	ros::NodeHandle cmd_in;
	ros::Subscriber<fish_msgs::joystick_in> serial_sub;

	// Low battery monitor
	DigitalIn* lowBatteryVoltageInput;
	InterruptIn* lowBatteryInterrupt;

	// Debug LEDs
	#ifdef debugLEDsSerial
	DigitalOut* serialLEDs[4];
	#endif
};

// Create a static instance of SerialController to be used by anyone doing serial control
extern ROSController rosController;

#endif /* ROSCONTROL_ROSCONTROLLER_H_ */

