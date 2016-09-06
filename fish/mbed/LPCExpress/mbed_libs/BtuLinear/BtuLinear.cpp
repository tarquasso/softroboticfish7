#include "BtuLinear.h"

BtuLinear::BtuLinear(bool dryRun):
    m_depthPid(DEP_KC, DEP_KI, DEP_KD, PID_FREQ, DEPTH_MIN, DEPTH_MAX, ACC_MIN, ACC_MAX, 0),
    m_vvPid(VV_KC, VV_KI, VV_KD, PID_FREQ, VV_MIN, VV_MAX, ACC_MIN, ACC_MAX, 0),
    m_accelPid(ACC_KC, ACC_KI, ACC_KD, PID_FREQ, ACC_MIN, ACC_MAX, VOLT_MIN, VOLT_MAX, 0),
    m_actA(PIN_ACTA_PWM, PIN_ACTA_DIR, PIN_ACTA_POT, PID_FREQ),
    m_actB(PIN_ACTB_PWM, PIN_ACTB_DIR, PIN_ACTB_POT, PID_FREQ),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_dryRunPot(DRY_RUN_POT_PIN),
    m_imu(p28, p27)
{
    m_dryRun = dryRun;
};

BtuLinear::~BtuLinear(){}

void BtuLinear::init() {
    m_mode = DEFAULT_CTRL_MODE;
    m_oldDepth = 0;
    m_oldVel = 0;

    // default gain values for depth controller
    updateDepthTunings(DEP_KC, DEP_KI, DEP_KD);

    // default gain values for position controller
    updatePosTunings(POS_KC,POS_KI,POS_KD);

    // default gain values for velocity controller
    updateVelTunings(VEL_KC,VEL_KI,VEL_KD);

    updateVVTunings(VV_KC, VV_KI, VV_KD);

    DigitalOut imuPower = DigitalOut(p5);
    imuPower = 0;
    wait(5);
    imuPower = 1;
    bool status = m_imu.begin(OPERATION_MODE_NDOF);

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
void BtuLinear::updateDepthTunings(float kc, float kI, float kD) {
    m_kc = kc;
    m_kI = kI;
    m_kD = kD;
    m_depthPid.setTunings(kc, kI, kD);
}

// updates Position PID tunings
void BtuLinear::updatePosTunings(float kc, float kI, float kD) {
    m_p_kc = kc;
    m_p_kI = kI;
    m_p_kD = kD;
    m_actA.setPosTunings(kc, kI, kD);
    m_actB.setPosTunings(kc, kI, kD);
}

// updates Velocity PID tunings
void BtuLinear::updateVelTunings(float kc, float kI, float kD) {
    m_v_kc = kc;
    m_v_kI = kI;
    m_v_kD = kD;
    m_actA.setVelTunings(kc, kI, kD);
    m_actB.setVelTunings(kc, kI, kD);
}

void BtuLinear::updateVVTunings(float kc, float kI, float kD) {
  m_vv_kc = kc;
  m_vv_kI = kI;
  m_vv_kD = kD;
  m_vvPid.setTunings(kc, kI, kD);
}

void BtuLinear::updateAccelTunings(float kc, float kI, float kD) {
  m_acc_kc = kc;
  m_acc_kI = kI;
  m_acc_kD = kD;
  m_accelPid.setTunings(kc, kI, kD);
}

// void BtuLinear::updatePitchTunings(float kc, float kI, float kD) {
//   m_pitch_kc = kc;
//   m_pitch_kI = kI;
//   m_pitch_kD = kD;
//   m_pitchPid.setTunings(kc, kI, kD);
// }

// updates Mode.  Resets most values if the mode has changed
void BtuLinear::updateMode(int mode) {
    if(m_mode != mode) {
        stop();
        m_mode = mode;
    }
}

// runs one cycle of the controller dictated by mode
void BtuLinear::runCycle(float setVal) {
  //m_pressureSensor.Barometer_MS5837();
	m_actA.updatePosition();
	m_actB.updatePosition();
	//updateDepth();
	m_currentVel = (m_curDepth - m_oldDepth) / PID_FREQ;
	// m_currentAccel = (m_currentVel - m_oldVel) / PID_FREQ;
    m_currentAccel = getAccel();
	m_oldVel = m_currentVel;
	m_oldDepth = m_curDepth;
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

    case VV_CTRL_MODE:
      vvControl(setVal);
      break;

    case ACCELERATION_CTRL_MODE:
      accelControl(setVal);
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
    // Read Pressure Value and Convert into Depth in Meters

	// current depth becomes process variable for PID controller
    m_depthPid.setProcessValue(m_curDepth);

    // desired depth in meters becomes set point of PID controller
	m_depthPid.setSetPoint(setDepthMeters);

	// commanded voltage is calculated by PID controller
    float cmdVel = m_depthPid.compute();

    // extending the actuator increases buoyancy -> less depth
    // therefore the axis direction has to be reversed
    // and commanded voltage with opposite sign is therefore applied
    //vvControl( cmdVel );

    // depthControlHelper(-1*cmdVel);
    accelControl(cmdVel);
}

void BtuLinear::vvControl(float setVelMeters) {
  m_vvPid.setProcessValue(m_currentVel);
  m_vvPid.setSetPoint(setVelMeters);
  float cmdVolt = m_vvPid.compute();
  // depthControlHelper(-1 * cmdVolt);
  accelControl(cmdVolt);
}

float BtuLinear::getAccel() {
  Vector gravVector = m_imu.getVector(VECTOR_GRAVITY);
  Vector linAccel = m_imu.getVector(VECTOR_LINEARACCEL);
  float gMag = (sqrt((gravVector[0]*gravVector[0]) + (gravVector[1]*gravVector[1]) + (gravVector[2]*gravVector[2])));
  float downAccel = ((linAccel[0] * gravVector[0] / gMag) + (linAccel[1] * gravVector[1] / gMag) + (linAccel[2] * gravVector[2] / gMag));
  // m_currentAccel = downAccel;
  return downAccel;
}

void BtuLinear::accelControl(float setAccMeters) {
  m_accelPid.setProcessValue(m_currentAccel);
  m_accelPid.setSetPoint(setAccMeters);
  float cmdVolt = m_accelPid.compute();
  depthControlHelper(-1 * cmdVolt);
}

// void BtuLinear::

// get a depth reading
float BtuLinear::getDepth() {
    if(m_dryRun) {
        float pvDepth = m_dryRunPot * (DEPTH_MAX);
        return pvDepth;
    }
    // read out pressure in mbar
    if(!m_dryRun) {
		float pvDepth = getPressure();
		    // convert pressure to meters
		float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
		//m_curDepth = m_mvgDepthAvg.computeMovingAverage(pvDepthMeters);
		m_curDepth = pvDepthMeters;
	}
    return m_curDepth;
}


void BtuLinear::setDryMode(bool dry) {
	m_dryRun = dry;
}
