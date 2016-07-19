#include <BtuRotary/BtuRotary.h>

// float clip(float val, float min, float max) {
//     float newVal = (val > max) ? max : val;
//     return (newVal < min) ? min : newVal;
// }



BtuRotary::BtuRotary():
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ, DEPTH_MIN, DEPTH_MAX, VEL_MIN, VEL_MAX, 0),
    m_specPid(SP_K_C, SP_TAU_I, SP_TAU_D, PID_FREQ, POS_MIN, POS_MAX, VEL_MIN, VEL_MAX, 0),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_motorServo(PIN_PWM_SERVO)
{};

BtuRotary::~BtuRotary(){}

void BtuRotary::init() {
    m_mode = 1;                   // pos control
    m_kc = DEP_K_C;
    m_tauI = DEP_TAU_I;
    m_tauD = DEP_TAU_D;

    m_motorServo.calibrate(SERVO_PWM_WIDTH, SERVO_DEGREE_WIDTH);
    m_pressureSensor.MS5837Init();
    m_pressureSensor.MS5837Start();
    wait(0.1);                    // remnant from old BTU class
}

float BtuRotary::getPressure() {
    return m_pressureSensor.MS5837_Pressure();
}

void BtuRotary::stop() {
    return;
}

void BtuRotary::update(int mode, float kc, float tauI, float tauD) {
    updateMode(mode);
    m_kc = kc;
    m_tauI = tauI;
    m_tauD = tauD;
    m_depthPid.setTunings(kc, tauI, tauD);
    m_specPid.setTunings(kc, tauI, tauD);
}

void BtuRotary::updateMode(int mode) {
    if(m_mode != mode) {
        stop();
        m_mode = mode;
        m_depthPid.reset();
        m_specPid.reset();
    }
}

void BtuRotary::runCycle(float setVal) {
    switch (m_mode) {

    case POSITION_CTRL_MODE:
        positionControl(setVal);
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

void BtuRotary::updateAndRunCycle(int mode, float value) {
    updateMode(mode);
    runCycle(value);
}


void BtuRotary::positionControl(float setPosDeg) {
    float setPos = utility::clip(setPosDeg, -SERVO_DEGREE_WIDTH, SERVO_DEGREE_WIDTH);
    m_motorServo.position(setPos);
}

void BtuRotary::velocityControl(float setVel) {
    float pos = m_motorServo.readPosition();
    m_motorServo.position(pos + (setVel * PID_FREQ));
}


void BtuRotary::depthControl(float setDepthMeters) {
    m_depthPid.setSetPoint(setDepthMeters);

    float curDepth = getDepth();

    m_depthPid.setProcessValue(curDepth);

    float cmdVel = m_depthPid.compute();
    velocityControl(cmdVel);
}

void BtuRotary::specialPosControl(float setPosDeg) {
    m_specPid.setSetPoint(setPosDeg);

    m_specPid.setProcessValue(m_motorServo.readPosition());
    float cmdVel = m_specPid.compute();
    // m_specCmd = (int)(cmdVel >= 0); // just for test recording purposes
    velocityControl(cmdVel);
}

float BtuRotary::getDepth() {
    float pvDepth = getPressure();
    float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
    return pvDepthMeters;
}
