#include "BtuLinearServo.h"

BtuLinearServo::BtuLinearServo(bool dryRun):
    m_depthPid(DEP_KC, DEP_KI, DEP_KD, PID_FREQ, DEPTH_MIN, DEPTH_MAX, VEL_MIN, VEL_MAX, 0),
    m_actA(PIN_ACTA_SERVO, PID_FREQ),
    m_actB(PIN_ACTB_SERVO, PID_FREQ),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_dryRunPot(DRY_RUN_POT_PIN)
{
    m_dryRun = dryRun;
};

BtuLinearServo::~BtuLinearServo(){}

void BtuLinearServo::init() {
    m_mode = DEFAULT_CTRL_MODE;

    // default gain values for depth controller
    this->updateDepthTunings(DEP_KC, DEP_KI, DEP_KD);

    // default gain values for position controller
    this->updatePosTunings(POS_KC,POS_KI,POS_KD);

    // default gain values for velocity controller
    this->updateVelTunings(VEL_KC,VEL_KI,VEL_KD);

    // initialize Pressure Sensor
    //m_pressureSensor.MS5837Init();
    //m_pressureSensor.MS5837Start();
    wait(0.1);                    // remnant from old BTU class TODO: check if can be removed

    // initialize the actuators
    m_actA.reset();
    m_actB.reset();

    // initialize the Windows for SMA to 0
    // for(int i = 0; i < AVG_WINDOW_WIDTH; i++) {
    //     m_avg_windowA[i] = 0;
    //     m_avg_windowB[i] = 0;
    // }

    // initialize starting voltage for velocity control to 0
    // m_currentVoltage = 0;
}

// return a pressure reading
float BtuLinearServo::getPressure() {
    return m_pressureSensor.MS5837_Pressure();
}

// resets values of the controllers
void BtuLinearServo::stop() {
	m_depthPid.reset();
    m_actA.reset();
    m_actB.reset();
	return;
}

// updates depth PID tunings
void BtuLinearServo::updateDepthTunings(float kc, float kI, float kD) {
    m_kc = kc;
    m_kI = kI;
    m_kD = kD;
    m_depthPid.setTunings(kc, kI, kD);
}

// updates Position PID tunings
void BtuLinearServo::updatePosTunings(float kc, float kI, float kD) {
    m_p_kc = kc;
    m_p_kI = kI;
    m_p_kD = kD;
    m_actA.setPosTunings(kc, kI, kD);
    m_actB.setPosTunings(kc, kI, kD);
}

// updates Velocity PID tunings
void BtuLinearServo::updateVelTunings(float kc, float kI, float kD) {
    m_v_kc = kc;
    m_v_kI = kI;
    m_v_kD = kD;
    m_actA.setVelTunings(kc, kI, kD);
    m_actB.setVelTunings(kc, kI, kD);
}

// updates Mode.  Resets most values if the mode has changed
void BtuLinearServo::updateMode(int mode) {
    if(m_mode != mode) {
        stop();
        m_mode = mode;
    }
}

// runs one cycle of the controller dictated by mode
void BtuLinearServo::runCycle(float setVal) {
  //m_pressureSensor.Barometer_MS5837();
    switch (m_mode) {

    case VOLTAGE_CTRL_MODE:
        voltageControl(setVal);
        break;

    case VELOCITY_CTRL_MODE:
        velocityControl(setVal);
        break;

    case DEPTH_CTRL_MODE:
        depthControl(setVal);
        break;

    case POSITION_CTRL_MODE:
        positionControl(setVal);
        break;
    }
}

// convenience function, updates mode, then runs a cycle in the chosen mode
void BtuLinearServo::updateAndRunCycle(int mode, float value) {
    updateMode(mode);
    runCycle(value);
}


// calls voltageControlHelper on both actuators
void BtuLinearServo::voltageControl(float setDuty) {
    m_actA.runVoltControl(setDuty);
    m_actB.runVoltControl(setDuty);
}

// updates the SMA window with the current position reading
/*
void BtuLinearServo::updatePositionReadings() {
    float aPosition = m_actAPot;
    float bPosition = m_actBPot;

    float aOldPos = m_avg_windowA[m_avg_windowPtr];
    float bOldPos = m_avg_windowB[m_avg_windowPtr];
    m_avg_windowA[m_avg_windowPtr] = aPosition;
    m_avg_windowB[m_avg_windowPtr] = bPosition;
    m_avg_windowPtr = (m_avg_windowPtr+1) % AVG_WINDOW_WIDTH;
    if(m_avg_windowSize >= AVG_WINDOW_WIDTH) {
        // buffer is full
    	m_currentAvgA = m_currentAvgA + (aPosition / AVG_WINDOW_WIDTH)- (aOldPos / AVG_WINDOW_WIDTH);
        m_currentAvgB = m_currentAvgB + (bPosition / AVG_WINDOW_WIDTH)- (bOldPos / AVG_WINDOW_WIDTH);
    } else {
    	// buffer is still filling up
        m_avg_windowSize++;
        m_currentAvgA = 0;
        m_currentAvgB = 0;
        for(int i = 0; i < m_avg_windowSize; i++) {
            m_currentAvgA = (m_avg_windowA[i] / m_avg_windowSize);
            m_currentAvgB = (m_avg_windowB[i] / m_avg_windowSize);
        }
    }
}
*/

// gets the current Actuator Position.  No SMA, just reads and rescales the potentiometer
float BtuLinearServo::getActPosition(int act) {
    if(act == ACT_A) {
        return m_actA.getPosition();
    } else {
        return m_actB.getPosition();
    }
}


// does velocity control on both actuators
void BtuLinearServo::velocityControl(float setVel) {
    m_actA.runVelControl(setVel);
    m_actB.runVelControl(setVel);
}

// control position of both actuators
void BtuLinearServo::positionControl(float setPos) {
    m_actA.runPosControl(setPos);
    m_actB.runPosControl(setPos);
}

// control depth via master-slave
void BtuLinearServo::depthControlHelper(float cmdVoltage) {
	// control velocity on one actuator
    m_actA.runVoltControl(cmdVoltage);
    // have the second mirror the first
    m_actB.runVelControl(m_actA.getPosition());
}

// do depth control
void BtuLinearServo::depthControl(float setDepthMeters) {
    // Read Pressure Value and Convert into Depth in Meters
	float curDepth = getDepth();

	// current depth becomes process variable for PID controller
    m_depthPid.setProcessValue(curDepth);

    // desired depth in meters becomes set point of PID controller
	m_depthPid.setSetPoint(setDepthMeters);

	// commanded voltage is calculated by PID controller
    float cmdVolt = m_depthPid.compute();

    // extending the actuator increases buoyancy -> less depth
    // therefore the axis direction has to be reversed
    // and commanded voltage with opposite sign is therefore applied
    depthControlHelper( -1 * cmdVolt );
}

// get a depth reading
float BtuLinearServo::getDepth() {
    if(m_dryRun) {
        float pvDepth = m_dryRunPot * (DEPTH_MAX);
        return pvDepth;
    }
    // read out pressure in mbar
    float pvDepth = getPressure();
    // convert pressure to meters
    float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
    return pvDepthMeters;
}

void BtuLinearServo::setDryMode(bool dry) {
	m_dryRun = dry;
}
