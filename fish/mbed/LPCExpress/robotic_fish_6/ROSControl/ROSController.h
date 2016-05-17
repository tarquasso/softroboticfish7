/*
 * ROSController.h
 *
 * Author: Joseph DelPreto
 */

//#define rosControl

#ifdef rosControl

#ifndef ROSCONTROL_ROSCONTROLLER_H_
#define ROSCONTROL_ROSCONTROLLER_H_

#include "FishController.h"
#include "mbed.h"
#include <ros.h>
#include <std_msgs/Empty.h>
#include <fish_msgs/JoystickState.h>

// ROS setup
// TODO replace this with your topic name
#define rosTopicName "to_serial"

// Pins and comm
#define rosDefaultBaudUSB 115200
#define rosDefaultBaud 115200
#define rosDefaultTX p13
#define rosDefaultRX p14
// note lowBatteryVoltagePin is defined in FishController

//#define printStatusROSController // whether to print what's going on (i.e. when it gets commands, etc.)
#define debugLEDsROS      // LED1: initialized LED2: running LED3: receiving a message LED4: done (others turn off)
#define runTimeROS 10000  // how long to run for (in milliseconds) if inifiniteLoopROS is undefined
#define infiniteLoopROS   // if defined, will run forever (or until stop() is called from another thread)

#define rosControllerControlFish // whether to start fishController to control the servos and motor

// Map bytes sent over ROS serial to ranges for each fish property
#define rosMinPitch     fishMinPitch
#define rosMaxPitch     fishMaxPitch
#define rosMinYaw       fishMinYaw
#define rosMaxYaw       fishMaxYaw
#define rosMinThrust    fishMinThrust
#define rosMaxThrust    fishMaxThrust
#define rosMinFrequency fishMinFrequency
#define rosMaxFrequency fishMaxFrequency

class ROSController
{
	public:
		// Initialization
		ROSController(Serial* serialObject = NULL, Serial* usbSerialObject = NULL); // if objects are null, ones will be created
		void init(Serial* serialObject = NULL, Serial* usbSerialObject = NULL); // if serial objects are null, ones will be created
		// Execution control
		void run();
		void stop();
		void lowBatteryCallback();
		// ROS
		ros::NodeHandle nodeHandle;
		ros::Subscriber<fish_msgs::JoystickState> subscriber;
		void processROSMessage(const fish_msgs::JoystickState& msg);
	private:
		Timer programTimer;
		bool terminated;
		Serial* usbSerial;
		Serial* serial;

		// Low battery monitor
		DigitalIn* lowBatteryVoltageInput;
		Ticker lowBatteryTicker;
		bool detectedLowBattery;

		// Debug LEDs
		#ifdef debugLEDsROS
		DigitalOut* rosLEDs[4];
		#endif
};

// Create a static instance of ROSController to be used by anyone doing ROS control
extern ROSController rosController;

#endif /* ROSCONTROL_ROSCONTROLLER_H_ */

#endif // #ifdef rosControl
