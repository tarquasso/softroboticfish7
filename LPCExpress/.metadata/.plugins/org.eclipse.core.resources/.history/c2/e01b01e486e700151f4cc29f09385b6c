
#include "FishController.h"

// The static instance
FishController fishController;

volatile uint8_t streamFishStateEvent = 0;
volatile uint16_t streamCurFishState = 0;

// Function to reset mbed
extern "C" void mbed_reset();

// Map received state to fish values
const float pitchLookup[] = {0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8}; // [0.2 - 0.8]
//const float yawLookup[] = {-1, -0.6667, -0.3333, 0, 0.3333, 0.6667, 1}; // [-1, 1]
//const float yawLookup[] = {-1, -0.875, -0.7, 0, 0.7, 0.875, 1}; // [-1, 1]
const float yawLookup[] = {-1, -0.7, -0.5, 0, 0.5, 0.7, 1}; // [-1, 1]
const float thrustLookup[] = {0, 0.25, 0.50, 0.75};  
const float frequencyLookup[] = {0.0000009, 0.0000012, 0.0000014, 0.0000016}; // cycles/us
const float periodHalfLookup[] = {555555, 416666, 357142, 312500}; // 1/(2*frequencyLookup) -> us

//============================================
// Initialization
//============================================

// Constructor
FishController::FishController() :
    // Initialize variables
    tickerInterval(1000),
    curTime(0),
    fullCycle(true),
    raiser(3.5),
    inTickerCallback(false),
    // Outputs for motor and servos
    motorPWM(p23),
    motorOutA(p11),
    motorOutB(p12),
    servoLeft(p21),
    servoRight(p24),
    //brushlessMotor(p25),
    brushlessOffTime(30000),
    // Button board
	autoMode(false),
    buttonBoard(p9, p10, p29, p30) // sda, scl, int1, int2
{
    newSelectButtonIndex = 0;
    newPitchIndex = 3;
    newYawIndex = 3;
    newThrustIndex = 0;
    newFrequencyIndex = 1;
    
    selectButton = getSelectButton();
    pitch = getPitch();
    yaw = getYaw();
    thrust = getThrust();
    thrustCommand = 0;
    frequency = getFrequency();
    periodHalf = getPeriodHalf();   
    dutyCycle = 0;
    brushlessOff = false;
    
    buttonBoard.registerCallback(&FishController::buttonCallback);
    buttonBoard.setLEDs(255, false);
}

void FishController::start()
{
    //printf("Arming brushless motor\n");
    //wait_ms(3000);
    //Timer timemotor;
    //timemotor.start();
    //brushlessMotor.period_ms(20);
    //brushlessMotor.pulsewidth_us(1500); // neutral position
    //brushlessMotor();
    //timemotor.stop();
    //printf("Setting took %d us\n", timemotor.read_us());
    //wait_ms(3000); // to arm brushless motor
    
    // Blink button board LEDs to indicate startup
    for(uint8_t i = 0; i < 3; i++)
    {
        buttonBoard.setLEDs(255, true);
        wait_ms(500);
        buttonBoard.setLEDs(255, false);
        wait_ms(500);
    }
    
    
    // Start control ticker callback
    printf("Fish controller starting\n");
    ticker.attach_us(&fishController, &FishController::tickerCallback, tickerInterval);
//    #ifdef debugFishState
//    printf("Starting...\n");
//    #endif
}

void FishController::stop()
{
    // Stop updating the fish
    while(inTickerCallback);
    ticker.detach();
    wait_ms(5);
    
    // Reset fish state to neutral
    newSelectButtonIndex = 0;
    newPitchIndex = 3;
    newYawIndex = 3;
    newThrustIndex = 0;
    newFrequencyIndex = 1;
    // Send commands to fish (multiple times to make sure we get in the right part of the cycle to actually update it)
    for(int i = 0; i < 200; i++)
    {
    	tickerCallback();
    	wait_ms(10);
    }
    wait(1);
    
    // Put dive planes in a weird position
    servoLeft = pitchLookup[0];
    servoRight = pitchLookup[0];
    
    // Light the LEDs to indicate termination
    buttonBoard.setLEDs(255, true);
    wait(90);
    buttonBoard.setLEDs(255, false);
}

//============================================
// Processing
//============================================ 
void FishController::processDataWord(uint16_t word)
{
    while(inTickerCallback);
    // Extract state from word
    newSelectButtonIndex = getSelectIndex(word);
    newPitchIndex = getPitchIndex(word);
    newYawIndex = getYawIndex(word);
    newThrustIndex = getThrustIndex(word);
    newFrequencyIndex = getFrequencyIndex(word);
}

void FishController::tickerCallback()
{
    inTickerCallback = true;
    
    // get the current elapsed time since last reset (us)
    curTime += tickerInterval; 

    // see if brushless should be shut down
    brushlessOff = curTime > (periodHalf-brushlessOffTime);
    
    // update every half cycle
    if(curTime > periodHalf) 
    { 
        // read new yaw value every half cycle
        yaw = getYaw(); // a value from -1 to 1

        // Read frequency only every full cycle
        if(fullCycle) 
        { 
            // Read other new inputs
            thrust = getThrust(); // a value from 0 to 1
            frequency = getFrequency(); 
            periodHalf = getPeriodHalf();
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
    pitch = getPitch();
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
    
    
//    #ifdef debugFishState
//    //printDebugState();
//    #endif
    //printf("%f\n", dutyCycle);
    streamCurFishState = getCurStateWord;
    //printf("%d %d\n", streamCurFishState, newPitchIndex);
    inTickerCallback = false;
}

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
    switch(state)
    {
        case BTTN_YAW_LEFT:
            if(fishController.newYawIndex >= 0 && fishController.newYawIndex <= 3)
                fishController.newYawIndex = 1;
            else if(fishController.newYawIndex >= 4 && fishController.newYawIndex <= 6)
				fishController.newYawIndex = 3;
            streamFishStateEvent = 6;
            break;
        case BTTN_YAW_RIGHT:
            //if(fishController.newYawIndex < 6)
            //    fishController.newYawIndex++;
            if(fishController.newYawIndex >= 3 && fishController.newYawIndex <= 6)
				fishController.newYawIndex = 5;
			else if(fishController.newYawIndex >= 0 && fishController.newYawIndex <= 2)
				fishController.newYawIndex = 3;
            streamFishStateEvent = 7;
            break;
        case BTTN_FASTER:
            if(fishController.newThrustIndex < 3)
                fishController.newThrustIndex++;
            streamFishStateEvent = 8;
            break;
        case BTTN_SLOWER:
            if(fishController.newThrustIndex > 0)
                fishController.newThrustIndex--;
            streamFishStateEvent = 9;
            break;
        case BTTN_PITCH_UP:
            if(fishController.newPitchIndex < 6)
                fishController.newPitchIndex++;
            streamFishStateEvent = 10;
            break;
        case BTTN_PITCH_DOWN:
            if(fishController.newPitchIndex > 0)
                fishController.newPitchIndex--;
            streamFishStateEvent = 11;
            break;
        case BTTN_SHUTDOWN_PI:
        	streamFishStateEvent = 12;
            simBatteryLow = new DigitalOut(p26);
            simBatteryLow->write(0);
            break;
        case BTTN_RESET_MBED:
        	streamFishStateEvent = 13;
            mbed_reset();
            break;
        case BTTN_AUTO_MODE:
        	streamFishStateEvent = 14;
        	fishController.autoMode = !fishController.autoMode;
        	if(fishController.autoMode)
        		fishController.setLEDs(21, true);
        	else
        	{
        		fishController.setLEDs(255, false);
        		fishController.processDataWord(0x0236);
        	}
        	break;
        default:
        	streamFishStateEvent = 15;
        	break;
    }
}

//#ifdef debugFishState
//void FishController::printDebugState()
//{
//    printf("\npitch: %f\nyaw: %f\nthrust: %f\nfrequency: %f\nservoLeft: %f\nservoRight: %f\ndutyCycle: %f\n",
//            pitch, yaw, thrust, frequency, servoLeft.read(), servoRight.read(), dutyCycle);
//}
//#endif

void FishController::setLEDs(char mask, bool turnOn)
{
    buttonBoard.setLEDs(mask, turnOn);
}
