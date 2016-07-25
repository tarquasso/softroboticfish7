#include <utility.h>
#include "mbed.h"
#include "BtuLinear.h"

#define NUM_FLOATS 6
#define TIMESTEP 0.05
#define DEPTH_THRESHOLD 0.5
#define MIN_MISSION_DEPTH 0.5
#define MISSION_TIMEOUT 60.0
#define ERROR_THRESHOLD 0.125
#define SUCCESS_TIME 8.0
#define DEBRIEF_TIME_LIMIT 20.0
#define UNWOUND_POS 1.0

#define DEF_V_KC 0.5
#define DEF_V_TAUI 0
#define DEF_V_TAUD 0

#define DEF_P_KC 16
#define DEF_P_TAUI 2
#define DEF_P_TAUD 1

#define DRY_RUN 1

#define CONTROL_MODE 55
#define MISSION_MODE 66

#define FREQUENCY 20


#include "MODSERIAL/MODSERIAL.h"
#include "SerialComm/SerialComm.h"

MODSERIAL pcSerial(USBTX, USBRX);
// AnalogIn pot1(p15);
DigitalOut TestLED(LED1);
DigitalOut TestLED2(LED2);
DigitalOut inMission(LED3);
DigitalOut missionSuccess(LED4);

LocalFileSystem local("local");

// bool clk = true;
BtuLinear btu = BtuLinear();
int counter = 0;
int mode = 2;
float Kc = 1.0;
float TauI = 0.0;
float TauD = 0.0;
float floorVal;                 // temp value for mission parameters
float jumpVal;                  // temp value for mission parameters
float missionFloor;             // Lowest depth of a mission
float missionStep;              // Distance between waypoints of a mission
float setVal = 0.0;             // meters
Ticker Control;
Ticker Mission;                 // Ticker for running Mission in parallel with main loop
float timeout = 0.0;            // time in seconds since last waypoint reached
float missionTime = 0.0;        // time in seconds that mission has taken
float successTime = 0.0;        // time in seconds spent within waypoint
float missionDepth = 0.0;       // current waypoint depth
bool missionStarted = false;    // boolean detailing whether or not robot has begun automatically following mission parameters
float debriefTime = 0.0;        // amount of time spent recording log after mission has ended
bool debriefMode = false;       // boolean indicating whether in debrief mode
FILE *fp;                       // pointer for writing log
bool returnTrip = false;        // determines whether the robot is in the descending or ascending part of the mission.  true = ascending
float valueFloats[NUM_FLOATS];
bool missionMode = false;
SerialComm serialComm(&pcSerial);


// end a mission and reset the various parameters.
void terminateMission() {
    Mission.detach();           // stop running mission loop asynchronously
    fclose(fp);                 // close the file
    counter = 0;                // resets the LED mission loop counter
    TestLED = 0;                // turns off the mission loop LED
    inMission = 0;              // turns off the inMission LED
    timeout = 0.0;
    successTime = 0.0;
    missionTime = 0.0;
    missionStarted = false;
    debriefMode = false;
    debriefTime = 0.0;
    returnTrip = false;

    btu.updateAndRunCycle(VELOCITY_CTRL_MODE, UNWOUND_POS); // attempt to return to surface
}

// returns true if we're close enough to current waypoint
bool checkThreshold() {
    float error = btu.getDepth() - missionDepth;
    // float error = btu.getServoPos() - missionDepth;
    float absError = (error > 0) ? error : (-1 * error);
    return (absError <= ERROR_THRESHOLD);
}


// Main mission loop
void runMission() {
    // if in debrief mode, update timer and loop again
	if(debriefMode) {
		inMission = 0;
		missionDepth = 0;
		btu.updateAndRunCycle(VELOCITY_CTRL_MODE, UNWOUND_POS);
        // finally tidy up and kill loop if debrief time has completed
		if(debriefTime >= DEBRIEF_TIME_LIMIT) {
			terminateMission();
		}
		debriefTime += TIMESTEP;
		return;                 // start over the mission loop without checking all the later conditions
	}
    // counter just for flicking the LED every second
    counter = (counter + 1) % 20;
    if(!counter) {
        TestLED = !TestLED;
    }
    // start the mission if we're below depth threshold. Does not reset any values.  Cannot end a mission, so running it multiple times is innocuous
    if(btu.getDepth() >= DEPTH_THRESHOLD) {
        inMission = 1;
        missionStarted = true;
        btu.updateMode(DEPTH_CTRL_MODE);
    }
    // Do nothing if mission has not yet begun
    if(!missionStarted) {
        return;
    }
    // otherwise run depth controller
    btu.runCycle(missionDepth);

    // if we're close enough to our next waypoint, begin counting success time.  Otherwise, reset success time
    if(checkThreshold()) {
    	successTime += TIMESTEP;
    } else {
    	successTime = 0.0;
    }

    // if we've been near the waypoint for long enough
    if (successTime >= SUCCESS_TIME) {
        // check if we're completely done.  i.e. done surfacing on our return trip.  If so, report success and begin debriefing
        if((missionDepth <= MIN_MISSION_DEPTH) && (returnTrip == true)) {
            missionSuccess = 1;
            debriefMode = true;
            return;
        } else if(missionDepth == missionFloor) { // otherwise, check if we reached the bottom of our mission and need to begin resurfacing
            returnTrip = true;
            successTime = 0.0;
            missionDepth = utility::clip(missionDepth - missionStep, MIN_MISSION_DEPTH, missionFloor);
            timeout = 0.0;
        } else {                // otherwise, just advance the mission
        	successTime = 0.0;
            // if we're on the return trip, move up instead of down
            if(returnTrip) {
                missionDepth = utility::clip(missionDepth - missionStep, MIN_MISSION_DEPTH, missionFloor);
            } else {
                missionDepth = utility::clip(missionDepth + missionStep, MIN_MISSION_DEPTH, missionFloor);
            }
            timeout = 0.0;
        }
    }
    // mission failure.  Taken too long to reach a waypoint.  Begin resurfacing, report failure.
    if (timeout >= MISSION_TIMEOUT) {
        missionSuccess = 0;
        debriefMode = true;
        return;
    }
    // advance time
    missionTime += TIMESTEP;
    timeout += TIMESTEP;
}


// sets various mission parameters, being sure to detach a mission if it is already present.
void startMission(float kc, float taui, float taud, float setDepth, float stepDist) {
    terminateMission();
    fp = fopen("/local/log", "w");
    fprintf(fp, "MISSION START, TARGET: %.2f, STEP DISTANCE: %.2f\r\n", setDepth, stepDist);

    missionSuccess = 0;
    missionFloor = setDepth;
    missionDepth = utility::clip(stepDist, MIN_MISSION_DEPTH, missionFloor);
    missionStep = stepDist;
    returnTrip = false;
    btu.updateDepthTunings(kc, taui, taud);
    btu.updateVelTunings(DEF_V_KC, DEF_V_TAUI, DEF_V_TAUD);
    btu.updatePosTunings(DEF_P_KC, DEF_P_TAUI, DEF_P_TAUD);
    // btu.update(SPEC_POSITION_CTRL_MODE, kc, taui, taud);
    Mission.attach(&runMission, TIMESTEP);
}

void runControl() {
  counter = (counter + 1) % FREQUENCY;
  if(!counter) {
	// blink every second
    TestLED = !TestLED;
  }

  //setVal = pot1;
  //setVal = (b1-a1)*setVal+a1;


  // Update Mode and Run Cycle
  btu.updateAndRunCycle(mode, setVal);
}

void switchModes() {
	Control.detach();
	terminateMission();
	btu.stop();
	missionMode = !missionMode;
	if(!missionMode) {
		Control.attach(&runControl, TIMESTEP);
	}
}

void missionCycle() {
	// log in either mission or debrief modes
	if(inMission || debriefMode) {
		float depth = btu.getDepth();
		fprintf(fp, "m:%d, kc:%f, ti:%f, td:%f, s:%.2f, cu:%.2f %.2f, de:%.2f, depth_er:%.4f, time: %.2f, to:%.2f, rt:%d\r\n",
				btu.getMode(), btu.getDkC(), btu.getDkI(), btu.getDkD(), missionDepth, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, missionDepth - depth, missionTime, timeout, returnTrip);
	} else {                  // otherwise just try to surface
		btu.updateAndRunCycle(VELOCITY_CTRL_MODE, UNWOUND_POS);
	}

	// take in serial messages to start new missions
	if(serialComm.checkIfNewMessage()) {
		serialComm.getFloats(valueFloats, NUM_FLOATS);
		if(valueFloats[0] == CONTROL_MODE) {
			switchModes();
			return;
		}
		// mode = (int) valueFloats[0];
		Kc = valueFloats[0];
		TauI = valueFloats[1];
		TauD = valueFloats[2];
		jumpVal = valueFloats[3];
		floorVal = valueFloats[4];
		btu.setDryMode(!valueFloats[5]);
		startMission(Kc, TauI, TauD, floorVal, jumpVal);
	}
}




void testCycle() {
	float depth = 0;
	depth = btu.getDepth();
	if (mode == DEPTH_CTRL_MODE) {
		pcSerial.printf("m:%d, kc:%.2f, ti:%.2f, td:%.2f, s:%.2f, cu:%.2f %.2f, de:%.2f, depth_er:%.4f\r\n",
				btu.getMode(), btu.getDkC(), btu.getDkI(), btu.getDkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - depth);
	} else if (mode == POSITION_CTRL_MODE) {
		pcSerial.printf("m:%d, kc:%.2f, ti:%.2f, td:%.2f, s:%.2f, cu:%.2f %.2f, de:%.2f, pos_er:%.4f %.4f\r\n",
				btu.getMode(), btu.getPkC(), btu.getPkI(), btu.getPkD(), setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth, setVal - btu.getActPosition(ACT_A), setVal - btu.getActPosition(ACT_B));
	} else {
		pcSerial.printf("m:%d, kc:%.2f, ti:%.2f, td:%.2f, s:%.2f, cu:%.2f %.2f, de:%.2f\r\n", btu.getMode(), Kc, TauI, TauD, setVal, btu.getActPosition(ACT_A), btu.getActPosition(ACT_B), depth);
	}

	if(serialComm.checkIfNewMessage()) {
		serialComm.getFloats(valueFloats, NUM_FLOATS);

		if(valueFloats[0] == MISSION_MODE) {
			switchModes();
			return;
		}
		mode = (int) valueFloats[0];
		Kc = valueFloats[1];
		TauI = valueFloats[2];
		TauD = valueFloats[3];
		setVal = valueFloats[4];
		btu.setDryMode(!valueFloats[5]);

		// Based on Mode, update Tunings
		if(mode == VELOCITY_CTRL_MODE) {
			btu.updateVelTunings(Kc, TauI, TauD);
		} else if (mode == POSITION_CTRL_MODE){
			btu.updatePosTunings(Kc, TauI, TauD);
		} else {
			btu.updateDepthTunings(Kc, TauI, TauD);
		}
	}
}


int main() {
  pcSerial.printf("Start!\n");

  btu.init();
  pcSerial.printf("pressure at start: %.6f\r\n", btu.getPressure());
  TestLED = 0;
  Control.attach(&runControl, TIMESTEP);

  while(1) {
	  if(missionMode) {
		  missionCycle();
	  } else {
		  testCycle();
	  }
      wait_ms(1000);
      TestLED2 = !TestLED2;
  }
}
