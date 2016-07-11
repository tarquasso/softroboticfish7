#include "BTU_Actuate/btu_PID_lite.h"

float clip(float val, float min, float max) {
    float newVal = (val > max) ? max : val;
    return (newVal < min) ? min : newVal;
}



BTU::BTU():
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ, DEPTH_MIN, DEPTH_MAX, VEL_MIN, VEL_MAX, 0),
    m_specPid(SP_K_C, SP_TAU_I, SP_TAU_D, PID_FREQ, POS_MIN, POS_MAX, VEL_MIN, VEL_MAX, 0),
    m_velPid(VEL_K_C, VEL_TAU_I, VEL_TAU_D, PID_FREQ, VEL_MIN, VEL_MAX, -1, 1, 0),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_actPwm1(PIN_ACT_PWM1),
    m_actPwm2(PIN_ACT_PWM2),
    m_actPot(PIN_ACT_POT)
{};

BTU::~BTU(){}

void BTU::init() {
    m_mode = 2;                   // vel control
    m_kc = DEP_K_C;
    m_tauI = DEP_TAU_I;
    m_tauD = DEP_TAU_D;

    m_v_kc = VEL_K_C;
    m_v_tauI = VEL_TAU_I;
    m_v_tauD = VEL_TAU_D;

    // m_motorServo.calibrate(SERVO_PWM_WIDTH, SERVO_DEGREE_WIDTH);
    m_pressureSensor.MS5837Init();
    // m_pressureSensor.MS5837Start();
    wait(0.1);                    // remnant from old BTU class

    m_oldPos = getActPosition();
}

float BTU::getPressure() {
    return m_pressureSensor.MS5837_Pressure();
}

void BTU::stop() {
    return;
}

void BTU::update(int mode, float kc, float tauI, float tauD) {
    updateMode(mode);
    m_kc = kc;
    m_tauI = tauI;
    m_tauD = tauD;
    m_depthPid.setTunings(kc, tauI, tauD);
    m_specPid.setTunings(kc, tauI, tauD);
}

void BTU::updateVelTunings(float kc, float tauI, float tauD) {
    m_v_kc = kc;
    m_v_tauI = tauI;
    m_v_tauD = tauD;
    m_velPid.setTunings(kc, tauI, tauD);
}

void BTU::updateMode(int mode) {
    if(m_mode != mode) {
        stop();
        m_mode = mode;
        m_depthPid.reset();
        m_specPid.reset();
        m_velPid.reset();
    }
}

void BTU::runCycle(float setVal) {
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

    case SPEC_POSITION_CTRL_MODE:
        specialPosControl(setVal);
        break;
    }
}

void BTU::updateAndRunCycle(int mode, float value) {
    updateMode(mode);
    runCycle(value);
}


// void BTU::positionControl(float setPosDeg) {
//     float setPos = clip(setPosDeg, -SERVO_DEGREE_WIDTH, SERVO_DEGREE_WIDTH);
//     m_motorServo.position(setPos);
// }

void BTU::voltageControl(float setDuty) {
    if(setDuty > 0) {
        m_actPwm1 = setDuty;
        m_actPwm2 = 0;
    } else {
        m_actPwm1 = 0;
        m_actPwm2 = -setDuty;
    }
}

float BTU::getActPosition() {
    float position = m_actPot;
    return position;
}

void BTU::velocityControl(float setVel) {
    // float pos = m_motorServo.readPosition();
    float pos = getActPosition();
    m_velPid.setProcessValue((pos - m_oldPos) / PID_FREQ);
    m_velPid.setSetPoint(setVel);
    float cmdVolt = m_velPid.compute();
    m_oldPos = pos;
    voltageControl(cmdVolt);
}



void BTU::depthControl(float setDepthMeters) {
    m_depthPid.setSetPoint(setDepthMeters);

    float curDepth = getDepth();

    m_depthPid.setProcessValue(curDepth);

    float cmdVel = m_depthPid.compute();
    velocityControl(-1 * cmdVel);
}

void BTU::specialPosControl(float setPosDeg) {
    m_specPid.setSetPoint(setPosDeg);

    // m_specPid.setProcessValue(m_motorServo.readPosition());
    m_specPid.setProcessValue(getActPosition());
    float cmdVel = m_specPid.compute();
    // m_specCmd = (int)(cmdVel >= 0); // just for test recording purposes
    velocityControl(cmdVel);
}

float BTU::getDepth() {
    float pvDepth = getPressure();
    float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
    return pvDepthMeters;
}

// float BTU::getServoPos() {
// 	return m_motorServo.readPosition();
// }
