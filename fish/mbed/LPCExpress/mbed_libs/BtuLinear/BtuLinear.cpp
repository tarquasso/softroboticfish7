#include "BtuLinear.h"

BtuLinear::BtuLinear():
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ, DEPTH_MIN, DEPTH_MAX, VEL_MIN, VEL_MAX, 0),
    m_actA(PIN_ACTA_PWM, PIN_ACTA_DIR, PIN_ACTA_POT, PID_FREQ),
    m_actB(PIN_ACTB_PWM, PIN_ACTB_DIR, PIN_ACTB_POT, PID_FREQ),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL)
{};

BtuLinear::~BtuLinear(){}

void BtuLinear::init() {
    m_mode = DEFAULT_CTRL_MODE;

    // default gain values for depth controller
    m_kc = DEP_K_C;
    m_tauI = DEP_TAU_I;
    m_tauD = DEP_TAU_D;

    // default gain values for position controller
    m_p_kc = POS_KC;
    m_p_tauI = POS_TAUI;
    m_p_tauD = POS_TAUD;

    // default gain values for velocity controller
    m_v_kc = VEL_KC;
    m_v_tauI = VEL_TAUI;
    m_v_tauD = VEL_TAUD;

    // initialize Pressure Sensor
    m_pressureSensor.MS5837Init();
    m_pressureSensor.MS5837Start();
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
float BtuLinear::getPressure() {
    return m_pressureSensor.MS5837_Pressure();
}

// resets values of the controllers
void BtuLinear::stop() {
	m_depthPid.reset();
    m_actA.reset();
    m_actB.reset();
	return;
}

// updates depth PID tunings
void BtuLinear::updateDepthTunings(float kc, float tauI, float tauD) {
    m_kc = kc;
    m_tauI = tauI;
    m_tauD = tauD;
    m_depthPid.setTunings(kc, tauI, tauD);
}

// updates Position PID tunings
void BtuLinear::updatePosTunings(float kc, float tauI, float tauD) {
    m_p_kc = kc;
    m_p_tauI = tauI;
    m_p_tauD = tauD;
    m_actA.setPosTunings(kc, tauI, tauD);
    m_actB.setPosTunings(kc, tauI, tauD);
}

// updates Velocity PID tunings
void BtuLinear::updateVelTunings(float kc, float tauI, float tauD) {
    m_v_kc = kc;
    m_v_tauI = tauI;
    m_v_tauD = tauD;
    m_actA.setVelTunings(kc, tauI, tauD);
    m_actB.setVelTunings(kc, tauI, tauD);
}

// updates Mode.  Resets most values if the mode has changed
void BtuLinear::updateMode(int mode) {
    if(m_mode != mode) {
        stop();
        m_mode = mode;
    }
}

// runs one cycle of the controller dictated by mode
void BtuLinear::runCycle(float setVal) {
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
void BtuLinear::updateAndRunCycle(int mode, float value) {
    updateMode(mode);
    runCycle(value);
}


// calls voltageControlHelper on both actuators
void BtuLinear::voltageControl(float setDuty) {
    m_actA.runVoltControl(setDuty);
    m_actB.runVoltControl(setDuty);
}

// updates the SMA window with the current position reading
/*
void BtuLinear::updatePositionReadings() {
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
float BtuLinear::getActPosition(int act) {
    if(act == ACT_A) {
        return m_actA.getPosition();
    } else {
        return m_actB.getPosition();
    }
}


// does velocity control on both actuators
void BtuLinear::velocityControl(float setVel) {
    m_actA.runVelControl(setVel);
    m_actB.runVelControl(setVel);
}

// control position of both actuators
void BtuLinear::positionControl(float setPos) {
    m_actA.runPosControl(setPos);
    m_actB.runPosControl(setPos);
}

// control depth via master-slave
void BtuLinear::depthControlHelper(float cmdVoltage) {
	// control velocity on one actuator
    m_actA.runVoltControl(cmdVoltage);
    // have the second mirror the first
    m_actB.runPosControl(m_actA.getPosition());
}

// do depth control
void BtuLinear::depthControl(float setDepthMeters) {
    float curDepth = getDepth();

    m_depthPid.setProcessValue(curDepth);
	m_depthPid.setSetPoint(setDepthMeters);

    float cmdVolt = m_depthPid.compute();

    // extending the actuator increases buoyancy -> less depth
    // therefore the axis direction has to be reversed
    depthControlHelper( -1 * cmdVolt );
}

// get a depth reading
float BtuLinear::getDepth() {
    float pvDepth = getPressure();
    float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
    return pvDepthMeters;
}

// float BTU::getServoPos() {
// 	return m_motorServo.readPosition();
// }
