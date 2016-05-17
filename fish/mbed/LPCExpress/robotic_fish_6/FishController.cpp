
#include "FishController.h"

// The static instance
FishController fishController;

// Function to reset mbed
extern "C" void mbed_reset();

// Auto mode
float autoModeCommands[][4] = {FISH_STRAIGHT, FISH_LEFT, FISH_STRAIGHT, FISH_LEFT};
uint32_t autoModeDurations[] = {4000, 2000, 2000, 2000}; // durations in milliseconds
const uint8_t autoModeLength = sizeof(autoModeDurations)/sizeof(autoModeDurations[0]);

//============================================
// Initialization
//============================================

// Constructor
FishController::FishController():
    // Initialize variables
	autoMode(false),
	ignoreExternalCommands(false),
    tickerInterval(fishControllerTickerInterval),
	inTickerCallback(false),
	#ifdef FISH4
    curTime(0),
    fullCycle(true),
    raiser(3.5),
    // Outputs for motor and servos
    motorPWM(motorPWMPin),
    motorOutA(motorOutAPin),
    motorOutB(motorOutBPin),
    servoLeft(servoLeftPin),
    servoRight(servoRightPin),
    //brushlessMotor(p25),
    brushlessOffTime(30000),
	#endif
/*	#ifdef FISH6 // these are declared in BCU class
	pressureSensor(pressureSensorPinSDA, pressureSensorPinSCL, ms5837_addr_no_CS),
	imuSensor(imuSensorPinSDA, imuSensorPinSCL)
	#endif*/
    // Button board
	buttonBoard(buttonBoardSDAPin, buttonBoardSCLPin, buttonBoardInt1Pin, buttonBoardInt2Pin) // sda, scl, int1, int2

{
	streamFishStateEventController = 0;

    newSelectButton = resetSelectButtonValue;
    newPitch = resetPitchValue;
    newYaw = resetYawValue;
    newThrust = resetThrustValue;
    newFrequency = resetFrequencyValue;
    newPeriodHalf = resetPeriodHalfValue;

    selectButton = newSelectButton;
    pitch = newPitch;
    yaw = newYaw;
    thrust = newThrust;
    frequency = newFrequency;
#ifdef FISH4
    periodHalf = newPeriodHalf;
    thrustCommand = 0;
    dutyCycle = 0;
    brushlessOff = false;
#endif

    buttonBoard.registerCallback(&FishController::buttonCallback);
    buttonBoard.setLEDs(255, false);

    autoModeIndex = 0;
    autoModeCount = 0;

}

// Set the desired state
// They will take affect at the next appropriate time in the control cycle
void FishController::setSelectButton(bool newSelectButtonValue, bool master /* = false*/)
{
	if(!ignoreExternalCommands || master)
		newSelectButton = newSelectButtonValue;
}
void FishController::setPitch(float newPitchValue, bool master /* = false*/)
{
	if(!ignoreExternalCommands || master)
	{
		newPitch = newPitchValue;
		setLEDs(BTTN_PITCH_UP,   (newPitch-fishMinPitch) > (fishMaxPitch - newPitch));
		setLEDs(BTTN_PITCH_DOWN, (newPitch-fishMinPitch) < (fishMaxPitch - newPitch));
	}
}
void FishController::setYaw(float newYawValue, bool master /* = false*/)
{
	if(!ignoreExternalCommands || master)
	{
		newYaw = newYawValue;
		setLEDs(BTTN_YAW_LEFT,  (newYaw-fishMinYaw) < (fishMaxYaw - newYaw));
		setLEDs(BTTN_YAW_RIGHT, (newYaw-fishMinYaw) > (fishMaxYaw - newYaw));
	}
}
void FishController::setThrust(float newThrustValue, bool master /* = false*/)
{
	if(!ignoreExternalCommands || master)
	{
		newThrust = newThrustValue;
		setLEDs(BTTN_FASTER, newThrust>fishMinThrust);
		// If we're in button-control mode, keep the no-thrust light on as an indicator
		if(!ignoreExternalCommands)
			setLEDs(BTTN_SLOWER, newThrust==fishMinThrust);
		else
			setLEDs(BTTN_SLOWER, true);
	}
}
void FishController::setFrequency(float newFrequencyValue, float newPeriodHalfValue /* = -1 */, bool master /* = false*/)
{
	if(!ignoreExternalCommands || master)
	{
		newFrequency = newFrequencyValue;
		newPeriodHalf = newPeriodHalfValue > -1 ? newPeriodHalfValue : (1.0/(2.0*newFrequency));
	}
}
// Get the (possible pending) state
bool FishController::getSelectButton() {return newSelectButton;}
float FishController::getPitch() {return newPitch;}
float FishController::getYaw() {return newYaw;}
float FishController::getThrust() {return newThrust;}
float FishController::getFrequency() {return newFrequency;}
float FishController::getPeriodHalf() {return newPeriodHalf;}

void FishController::start()
{
    // Blink button board LEDs to indicate startup
    for(uint8_t i = 0; i < 3; i++)
    {
        buttonBoard.setLEDs(255, true);
        wait_ms(500);
        buttonBoard.setLEDs(255, false);
        wait_ms(500);
    }

#ifdef FISH6
    buoyancyControlUnit.start();
    pumpWithValve.start();
#endif

    // Start control ticker callback
    ticker.attach_us(&fishController, &FishController::tickerCallback, tickerInterval);
#ifdef debugFishState
    printf("Starting...\n");
#endif


}

void FishController::stop()
{
    // Stop updating the fish
    while(inTickerCallback); // wait for commands to settle
    ticker.detach(); // stop updating commands
    wait_ms(5);      // wait a bit to make sure it stops

    // Reset fish state to neutral
    newSelectButton = resetSelectButtonValue;
    newPitch = resetPitchValue;
    newYaw = resetYawValue;
    newThrust = resetThrustValue;
    newFrequency = resetFrequencyValue;
    newPeriodHalf = resetPeriodHalfValue;
    // Send commands to fish (multiple times to make sure we get in the right part of the cycle to actually update it)
    for(int i = 0; i < 200; i++)
    {
    	tickerCallback();
    	wait_ms(10);
    }
    // Make sure commands are sent to motors and applied
    wait(1);

	#ifdef FISH4
    // Put dive planes in a weird position to indicate stopped
    servoLeft = 0.3;
    servoRight = 0.3;
	#endif

	#ifdef FISH6
	pumpWithValve.stop();
	buoyancyControlUnit.stop();
	#endif // FISH6


    // Light the LEDs to indicate termination
    buttonBoard.setLEDs(255, true);
}

//============================================
// Processing
//============================================
#ifdef FISH4
void FishController::tickerCallback()
{
    inTickerCallback = true; // so we don't asynchronously stop the controller in a bad point of the cycle

    // get the current elapsed time since last reset (us)
    curTime += tickerInterval;

    // see if brushless should be shut down
    brushlessOff = curTime > (periodHalf-brushlessOffTime);

    // update every half cycle
    if(curTime > periodHalf)
    {
        // read new yaw value every half cycle
        yaw = newYaw; // a value from -1 to 1

        // Read frequency only every full cycle
        if(fullCycle)
        {
            // Read other new inputs
            thrust = newThrust; // a value from 0 to 1
            frequency = newFrequency;
            periodHalf = newPeriodHalf;
            // Adjust thrust if needed
            if(yaw < 0.0)
                thrustCommand = (1.0 + 0.75*yaw)*thrust; // 0.7 can be adjusted to a power of 2 if needed
            else
            	thrustCommand = thrust;
            fullCycle = false;
        }
        else
        {
            // Reverse for the downward slope
            if(yaw > 0.0)
            	thrustCommand = -(1.0 - 0.75*yaw)*thrust;
            else
            	thrustCommand = -thrust;
            fullCycle = true;
        }

        // Reset time
        curTime = 0;
    }

    // Update the servos
    pitch = newPitch;
    servoLeft = pitch - 0.05; // The 0.03 calibrates the angles of the servo
    servoRight = (1.0 - pitch) < 0.03 ? 0.03 : (1.0 - pitch);

    // Update the duty cycle
    dutyCycle = raiser * sin(PI2 * frequency * curTime); // add factor 4.0 to get a cut off sinus
    if(dutyCycle > 1)
        dutyCycle = 1;
    if(dutyCycle < -1)
        dutyCycle = -1;
    dutyCycle *= thrustCommand;
    if(dutyCycle >= 0 && dutyCycle < 0.01)
        dutyCycle = 0;
    if(dutyCycle < 0 && dutyCycle > -0.01)
        dutyCycle = 0;
    // Update the brushed motor
    if(dutyCycle >= 0)
    {
        motorOutA.write(0);
        motorOutB.write(1);
        motorPWM.write(dutyCycle);
    }
    else
    {
        motorOutA.write(1);
        motorOutB.write(0);
        motorPWM.write(-1 * dutyCycle);
    }
    // Update the brushless motor
    //brushlessMotor = dutyCycle * !brushlessOff;
    //brushlessMotor.pulsewidth_us(dutyCycle*500+1500);
    //brushlessMotor();


#ifdef debugFishState
    printDebugState();
#endif
    //printf("%f\n", dutyCycle);
    //printf("%f %f\r\n", pitch, servoLeft.read());
    inTickerCallback = false;
}
#endif

#ifdef FISH6
void FishController::tickerCallback()
{
    inTickerCallback = true; // so we don't asynchronously stop the controller in a bad point of the cycle

    //set current state to newly commanded value
    frequency = newFrequency;
    yaw = newYaw;
    thrust = newThrust;
    pitch = newPitch;

    pumpWithValve.set(frequency, yaw, thrust);
    buoyancyControlUnit.set(pitch);

#ifdef debugFishState
    printDebugState();
#endif

    inTickerCallback = false;
}
#endif


// button will be mask indicating which button triggered this interrupt
// pressed will indicate whether that button was pressed or released
// buttonState will be a mask that indicates which buttons are currently pressed
void FishController::buttonCallback(char button, bool pressed, char state) // static
{
    //printf("button %d\t pressed: %d\t state: %d\n", button, pressed, state);
    //fishController.buttonBoard.setLEDs(button, !fishController.buttonBoard.getLEDs(button));
    // Only act on button presses (not releases)
    if(!pressed)
        return;

    DigitalOut* simBatteryLow;
    float newYaw, newThrust, newPitch;
    switch(state)
    {
        case BTTN_YAW_LEFT:
        	newYaw = fishController.newYaw;
        	newYaw -= (fishMaxYaw - fishMinYaw)/4.0;
        	newYaw = newYaw < fishMinYaw ? fishMinYaw : newYaw;
        	fishController.setYaw(newYaw, true);
            fishController.streamFishStateEventController = 6;
            break;
        case BTTN_YAW_RIGHT:
        	newYaw = fishController.newYaw;
			newYaw += (fishMaxYaw - fishMinYaw)/4.0;
			newYaw = newYaw > fishMaxYaw ? fishMaxYaw : newYaw;
			fishController.setYaw(newYaw, true);
            fishController.streamFishStateEventController = 7;
            break;
        case BTTN_FASTER:
        	newThrust = fishController.newThrust;
        	newThrust += (fishMaxThrust - fishMinThrust)/4.0;
        	newThrust = newThrust > fishMaxThrust ? fishMaxThrust : newThrust;
        	fishController.setThrust(newThrust, true);
            fishController.streamFishStateEventController = 8;
            break;
        case BTTN_SLOWER:
        	newThrust = fishController.newThrust;
			newThrust -= (fishMaxThrust - fishMinThrust)/4.0;
			newThrust = newThrust < fishMinThrust ? fishMinThrust : newThrust;
			fishController.setThrust(newThrust, true);
            fishController.streamFishStateEventController = 9;
            break;
        case BTTN_PITCH_UP:
        	newPitch = fishController.newPitch;
        	newPitch += (fishMaxPitch - fishMinPitch)/4.0;
        	newPitch = newPitch > fishMaxPitch ? fishMaxPitch : newPitch;
        	fishController.setPitch(newPitch, true);
            fishController.streamFishStateEventController = 10;
            break;
        case BTTN_PITCH_DOWN:
        	newPitch = fishController.newPitch;
			newPitch -= (fishMaxPitch - fishMinPitch)/4.0;
			newPitch = newPitch < fishMinPitch ? fishMinPitch : newPitch;
			fishController.setPitch(newPitch, true);
            fishController.streamFishStateEventController = 11;
            break;
        case BTTN_SHUTDOWN_PI: // signal a low battery signal to trigger the pi to shutdown
        	fishController.streamFishStateEventController = 12;
            simBatteryLow = new DigitalOut(lowBatteryVoltagePin);
            simBatteryLow->write(0);
            break;
        case BTTN_RESET_MBED:
        	fishController.streamFishStateEventController = 13; // ... if you see this, it didn't happen :)
            mbed_reset();
            break;
        case BTTN_AUTO_MODE:
        	fishController.streamFishStateEventController = 14;
        	if(fishController.autoMode)
        		fishController.stopAutoMode();
        	else
        		fishController.startAutoMode();
        	break;
        case BTTN_BTTN_MODE:
        	fishController.setIgnoreExternalCommands(!fishController.getIgnoreExternalCommands());
        	break;
        default:
        	fishController.streamFishStateEventController = 15;
        	break;
    }
}

void FishController::setIgnoreExternalCommands(bool ignore)
{
	ignoreExternalCommands = ignore;
}

bool FishController::getIgnoreExternalCommands()
{
	return ignoreExternalCommands;
}

void FishController::startAutoMode()
{
	// Start ignoring external commands so as not to interfere with auto mode
	// But remember what the previous setting was so we can restore it after auto mode
	ignoreExternalCommandsPreAutoMode = ignoreExternalCommands;
	setIgnoreExternalCommands(true);
	// Reset state
	autoModeCount = 0;
	autoModeIndex = 0;
	// Start executing the auto loop
	autoMode = true;
	autoModeTicker.attach_us(&fishController, &FishController::autoModeCallback, 10000);
}

void FishController::stopAutoMode()
{
	autoModeTicker.detach();
	// Auto mode was terminated - put fish into a neutral position
	setSelectButton(resetSelectButtonValue, true);
	setPitch(resetPitchValue, true);
	setYaw(resetYawValue, true);
	setThrust(resetThrustValue, true);
	setFrequency(resetFrequencyValue, resetPeriodHalfValue, true);
	// Restore external mode to what is was previously
	setIgnoreExternalCommands(ignoreExternalCommandsPreAutoMode);
	autoMode = false;
}

void FishController::autoModeCallback()
{
	// Assign the current state (stored as pitch, yaw, thrust, frequency)
	setPitch(autoModeCommands[autoModeIndex][0], true);
	setYaw(autoModeCommands[autoModeIndex][1], true);
	setThrust(autoModeCommands[autoModeIndex][2], true);
	setFrequency(autoModeCommands[autoModeIndex][3], 1.0/(2.0*autoModeCommands[autoModeIndex][3]), true);
	// See if we advance to the next command
	autoModeCount++;
	if(autoModeCount*10 > autoModeDurations[autoModeIndex])
	{
		autoModeCount = 0;
		autoModeIndex = (autoModeIndex+1) % autoModeLength; // loop continuously through commands
	}
}

#ifdef debugFishState
void FishController::printDebugState()
{
    printf("pitch: %2.2f yaw: %2.2f thrust: %2.2f frequency: %2.2f\n",
            pitch, yaw, thrust, frequency);
}
#endif

void FishController::setLEDs(char mask, bool turnOn)
{
    buttonBoard.setLEDs(mask, turnOn);
}
