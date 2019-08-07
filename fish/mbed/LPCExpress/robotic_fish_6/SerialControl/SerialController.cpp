/*
 * Author: Joseph DelPreto
 */

#include "SerialController.h"

#ifdef serialControl

// The static instance
SerialController serialController;

void lowBatteryCallbackSerialStatic()
{
	serialController.lowBatteryCallback();
}

// Initialization
SerialController::SerialController(Serial* serialObject /* = NULL */, Serial* usbSerialObject /* = NULL */):
		terminated(false)
{
	#ifdef debugLEDsSerial
	serialLEDs[0] = new DigitalOut(LED1);
	serialLEDs[1] = new DigitalOut(LED2);
	serialLEDs[2] = new DigitalOut(LED3);
	serialLEDs[3] = new DigitalOut(LED4);
	#endif
	init(serialObject, usbSerialObject);
}

void SerialController::init(Serial* serialObject /* = NULL */, Serial* usbSerialObject /* = NULL */)
{
	// Create serial object or use provided one
	if(serialObject == NULL)
	{
		serialObject = new Serial(serialDefaultTX, serialDefaultRX);
		serialObject->baud(serialDefaultBaud);
	}
	serial = serialObject;
	// Create usb serial object or use provided one
	if(usbSerialObject == NULL)
	{
		usbSerialObject = new Serial(USBTX, USBRX);
		usbSerialObject->baud(serialDefaultBaudUSB);
	}
	usbSerial = usbSerialObject;

	// Will check for low battery at startup and using an interrupt
	lowBatteryVoltageInput = new DigitalIn(lowBatteryVoltagePin);
	lowBatteryVoltageInput->mode(PullUp);
	// NOTE Switching to polling instead of interrupt
	// since it seems like the low battery detector chip may have spurious falls
	// that cause the interrupt to trigger when the battery isn't actually low
	//lowBatteryInterrupt = new InterruptIn(lowBatteryVoltagePin);
	//lowBatteryInterrupt->fall(lowBatteryCallbackSerialStatic);
	detectedLowBattery = false;
	lowBatteryTicker.attach(&lowBatteryCallbackSerialStatic, 5);

	#ifdef debugLEDsSerial
	serialLEDs[0]->write(1);
	serialLEDs[1]->write(0);
	serialLEDs[2]->write(0);
	serialLEDs[3]->write(0);
	#endif
}

// Parse the received word into the desired fish state
// FORMAT: 5 successive bytes indicating selectButton, Pitch, Yaw, Thrust, Frequency
//         a null termination character (0) ends the word
//         each one maps 1-255 to the range specified by the min and max values for that property
void SerialController::processSerialWord(uint8_t* word)
{
	// Scale the bytes into the desired ranges for each property
	bool selectButton = (word[0] > 127);
	float pitch = word[1];
	pitch = ((pitch) * (serialMaxPitch - serialMinPitch) / 6.0) + serialMinPitch;

	float yaw = word[2];
	yaw = ((yaw) * (serialMaxYaw - serialMinYaw) / 6.0) + serialMinYaw;

	float thrust = word[3];
	thrust = ((thrust) * (serialMaxThrust - serialMinThrust) / 3.0) + serialMinThrust;

	float frequency = word[4];
	frequency = ((frequency) * (serialMaxFrequency- serialMinFrequency) / 3.0) + serialMinFrequency;

	// Apply the new state to the fish
	fishController.setSelectButton(selectButton);
	fishController.setPitch(pitch);
	fishController.setYaw(yaw);
	fishController.setThrust(thrust);
	fishController.setFrequency(frequency, 1.0/(2.0*frequency));

	#ifdef printStatusSerialController
	//usbSerial->printf("%s", word);
	usbSerial->printf("Start %d\t Pitch %f\t Yaw %f\t Thrust %f\t Freq %.8f\r\n", selectButton, pitch, yaw, thrust, frequency);
	#endif
}

// Stop the controller (will also stop the fish controller)
//
void SerialController::stop()
{
	terminated = true;
}

// Main loop
// This is blocking - will not return until terminated by timeout or by calling stop() in another thread
void SerialController::run()
{

	#ifdef serialControllerControlFish
    // Start the fish controller
    fishController.start();
    #endif

    // Moved to ticker instead of interrupt (see comments in init), so don't need this check
    // Check for low battery voltage (also have the interrupt, but check that we're not starting with it low)
	//if(lowBatteryVoltageInput == 0)
	//	lowBatteryCallback();

	#ifdef printStatusSerialController
	usbSerial->printf("\r\nStarting to listen for serial commands 2\r\n");
	#endif

	#ifdef debugLEDsSerial
	serialLEDs[0]->write(1);
	serialLEDs[1]->write(1);
	serialLEDs[2]->write(0);
	serialLEDs[3]->write(0);
	#endif

	// Process any incoming serial commands
	uint8_t serialBuffer[20];
	uint32_t serialBufferIndex = 0;
	programTimer.reset();
	programTimer.start();
	while(!terminated)
	{
		if(serial->readable())
		{
			#ifdef debugLEDsSerial
			serialLEDs[2]->write(1);
			
			#endif
	
			//int nextByte = serial->putc('t');
			//usbSerial->printf("Processed <%s>: ", nextByte);
			uint8_t nextByte = serial->getc();
			serialBuffer[serialBufferIndex++] = nextByte;
			//usbSerial->printf("%c", serialBufferIndex);
			// If we've received a complete command, process it now
			if(nextByte == 8)
			{
				//usbSerial->printf("Got zero!\n");
				//usbSerial->printf((char*) (serialBuffer));
				processSerialWord(serialBuffer);
				serialBufferIndex = 0;
			}
		}
		else
		{
			#ifdef debugLEDsSerial
			serialLEDs[2]->write(0);
			#endif
		}
		#ifndef infiniteLoopSerial
		if(programTimer.read_ms() > runTimeSerial)
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
	#ifdef serialControllerControlFish
    fishController.stop();
    // If battery died, wait a bit for pi to clean up and shutdown and whatnot
    if(lowBatteryVoltageInput == 0)
    {
		wait(90); // Give the Pi time to shutdown
		fishController.setLEDs(255, false);
    }
    #endif

	#ifdef printStatusSerialController
	usbSerial->printf("\r\nSerial controller done!\r\n");
	#endif
}


void SerialController::lowBatteryCallback()
{
	if(lowBatteryVoltageInput == 0 && detectedLowBattery)
	{
		// Stop the serial controller
		// This will end the main loop, causing main to terminate
		// Main will also stop the fish controller once this method ends
		stop();
		// Also force the pin low to signal the Pi
		// (should have already been done, but just in case)
		// TODO check that this really forces it low after this method ends and the pin object may be deleted
		DigitalOut simBatteryLow(lowBatteryVoltagePin);
		simBatteryLow = 0;
		#ifdef printStatusSerialController
		usbSerial->printf("\r\nLow battery! Shutting down.\r\n");
		wait(0.5); // wait for the message to actually flush
		#endif
	}
	else if(lowBatteryVoltageInput == 0)
	{
		detectedLowBattery = true;
	}
}

#endif // #ifdef serialControl

