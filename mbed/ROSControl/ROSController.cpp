

#include "ROSController.h"

// The static instance
ROSController rosController;

void lowBatteryCallbackSerialStatic()
{
	rosController.lowBatteryCallback();
}

void processSerialWordStatic(const fish_msgs::joystick_in& cmd_msg)
{
	rosController.processSerialWord(cmd_msg);
}

// Initialization
ROSController::ROSController():
		terminated(false),
		serial_sub("to_serial", &processSerialWordStatic)
{
	#ifdef debugLEDsSerial
	serialLEDs[0] = new DigitalOut(LED1);
	serialLEDs[1] = new DigitalOut(LED2);
	serialLEDs[2] = new DigitalOut(LED3);
	serialLEDs[3] = new DigitalOut(LED4);
	#endif
	init();
}

void ROSController::init()
{
	ros::NodeHandle cmd_in;

	cmd_in.initNode();
	cmd_in.subscribe(serial_sub);

	// Will check for low battery at startup and using an interrupt
	lowBatteryVoltageInput = new DigitalIn(lowBatteryVoltagePin);
	lowBatteryVoltageInput->mode(PullUp);
	lowBatteryInterrupt = new InterruptIn(lowBatteryVoltagePin);
	lowBatteryInterrupt->fall(lowBatteryCallbackSerialStatic);

	#ifdef debugLEDsSerial
	serialLEDs[0]->write(1);
	serialLEDs[1]->write(0);
	serialLEDs[2]->write(0);
	serialLEDs[3]->write(0);
	#endif
}


void ROSController::processSerialWord(const fish_msgs::joystick_in& cmd_msg) {

	fishController.setSelectButton(bool(0));

	// ctrl values are signed ints
	float pitch = ((cmd_msg.depth_ctrl+127) * (serialMaxPitch - serialMinPitch) / 254.0) + serialMinPitch;
	float yaw = ((cmd_msg.yaw_ctrl+127) * (serialMaxYaw - serialMinYaw) / 254.0) + serialMinYaw;
	float thrust = ((cmd_msg.speed_ctrl+127) * (serialMaxThrust - serialMinThrust) / 254.0) + serialMinThrust;
	float frequency = ((cmd_msg.freq_ctrl+127) * (serialMaxFrequency- serialMinFrequency) / 254.0) + serialMinFrequency;

	fishController.setPitch(pitch);
	fishController.setYaw(yaw);
	fishController.setThrust(thrust);
	fishController.setFrequency(frequency, 1.0/(2.0*frequency));
}


// Stop the controller (will also stop the fish controller)
//
void ROSController::stop()
{
	terminated = true;
}

// Main loop
// This is blocking - will not return until terminated by timeout or by calling stop() in another thread
void ROSController::run()
{

	#ifdef rosControllerControlFish
    // Start the fish controller
    fishController.start();
    #endif

    // Check for low battery voltage (also have the interrupt, but check that we're not starting with it low)
	if(lowBatteryVoltageInput == 0)
		lowBatteryCallback();

/*	#ifdef printStatusSerialController
	usbSerial->printf("\nStarting to listen for serial commands\n");
	#endif */

	#ifdef debugLEDsSerial
	serialLEDs[0]->write(1);
	serialLEDs[1]->write(1);
	serialLEDs[2]->write(0);
	serialLEDs[3]->write(0);
	#endif

	// Process any incoming serial commands
	programTimer.reset();
	programTimer.start();
	while(!terminated)
	{
		cmd_in.spinOnce();

		#ifndef infiniteLoopSerial
		if(programTimer.read_ms() > runTime)
			stop();
		#endif
	}
	programTimer.stop();
	#ifdef debugLEDsSerial
	serialLEDs[0]->write(0);
	serialLEDs[1]->write(0);
	serialLEDs[2]->write(0);
	serialLEDs[3]->write(0);
	#endif

	// Stop the fish controller
	#ifdef rosControllerControlFish
    fishController.stop();
    // If battery died, wait a bit for pi to clean up and shutdown and whatnot
    if(lowBatteryVoltageInput == 0)
    {
		wait(90); // Give the Pi time to shutdown
		fishController.setLEDs(255, false);
    }
    #endif

/*	#ifdef printStatusSerialController
	usbSerial->printf("\nSerial controller done!\n");
	#endif*/
}


void ROSController::lowBatteryCallback()
{
//    // Stop the serial controller
//    // This will end the main loop, causing main to terminate
//    // Main will also stop the fish controller once this method ends
//    stop();
//    // Also force the pin low to signal the Pi
//    // (should have already been done, but just in case)
//    // TODO check that this really forces it low after this method ends and the pin object may be deleted
//    DigitalOut simBatteryLow(lowBatteryVoltagePin);
//    simBatteryLow = 0;
//	#ifdef printStatusSerialController
//    usbSerial->printf("\nLow battery! Shutting down.\n");
//    wait(0.5); // wait for the message to actually flush
//	#endif
}



