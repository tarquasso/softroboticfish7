#include "BtuLinear.h"

BtuLinear::BtuLinear():
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ, DEPTH_MIN, DEPTH_MAX, VEL_MIN, VEL_MAX, 0),
    m_posAPid(SP_K_C, SP_TAU_I, SP_TAU_D, PID_FREQ, POS_MIN, POS_MAX, VEL_MIN, VEL_MAX, 0),
    m_posBPid(SP_K_C, SP_TAU_I, SP_TAU_D, PID_FREQ, POS_MIN, POS_MAX, VEL_MIN, VEL_MAX, 0),
    m_velAPid(VEL_K_C, VEL_TAU_I, VEL_TAU_D, PID_FREQ, VEL_MIN, VEL_MAX, -1, 1, 0),
    m_velBPid(VEL_K_C, VEL_TAU_I, VEL_TAU_D, PID_FREQ, VEL_MIN, VEL_MAX, -1, 1, 0),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_actAPwm(PIN_ACTA_PWM),
    m_actBPwm(PIN_ACTB_PWM),
    m_actAPot(PIN_ACTA_POT),
    m_actBPot(PIN_ACTB_POT),
    m_actADir(PIN_ACTA_DIR),
    m_actBDir(PIN_ACTB_DIR)
{};

BtuLinear::~BtuLinear(){}

void BtuLinear::init() {
    m_mode = 2;                   // vel control
    m_kc = DEP_K_C;
    m_tauI = DEP_TAU_I;
    m_tauD = DEP_TAU_D;

    m_v_kc = VEL_K_C;
    m_v_tauI = VEL_TAU_I;
    m_v_tauD = VEL_TAU_D;

    // m_motorServo.calibrate(SERVO_PWM_WIDTH, SERVO_DEGREE_WIDTH);
    m_pressureSensor.MS5837Init();
    m_pressureSensor.MS5837Start();
    wait(0.1);                    // remnant from old BTU class

    m_oldPosA = getActPosition(ACT_A);
    m_oldPosB = getActPosition(ACT_B);

    for(int i = 0; i < AVG_WINDOW_WIDTH; i++) {
        m_avg_windowA[i] = 0;
        m_avg_windowB[i] = 0;
    }
    m_avg_windowPtr = 0;
    m_avg_windowSize = 0;
    m_currentAvgA = 0;

    m_currentAvgB = 0;
    m_currentVoltage = 0;
}

float BtuLinear::getPressure() {
    return m_pressureSensor.MS5837_Pressure();
}

void BtuLinear::stop() {
    return;
}

void BtuLinear::update(int mode, float kc, float tauI, float tauD) {
    updateMode(mode);
    m_kc = kc;
    m_tauI = tauI;
    m_tauD = tauD;
    m_depthPid.setTunings(kc, tauI, tauD);
}

void BtuLinear::updatePosTunings(float kc, float tauI, float tauD) {
    m_p_kc = kc;
    m_p_tauI = tauI;
    m_p_tauD = tauD;
    m_posAPid.setTunings(kc, tauI, tauD);
    m_posBPid.setTunings(kc, tauI, tauD);
}

void BtuLinear::updateVelTunings(float kc, float tauI, float tauD) {
    m_v_kc = kc;
    m_v_tauI = tauI;
    m_v_tauD = tauD;
    m_velAPid.setTunings(kc, tauI, tauD);
    m_velBPid.setTunings(kc, tauI, tauD);
}

void BtuLinear::updateMode(int mode) {
    if(m_mode != mode) {
        stop();
        m_mode = mode;
        m_currentVoltage = 0;
        m_currentAvgA = 0;
        m_currentAvgB = 0;
        m_avg_windowSize = 0;
        m_depthPid.reset();
        m_posAPid.reset();
        m_posBPid.reset();
        m_velAPid.reset();
        m_velBPid.reset();
    }
}

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

void BtuLinear::updateAndRunCycle(int mode, float value) {
    updateMode(mode);
    runCycle(value);
}


// void BTU::positionControl(float setPosDeg) {
//     float setPos = clip(setPosDeg, -SERVO_DEGREE_WIDTH, SERVO_DEGREE_WIDTH);
//     m_motorServo.position(setPos);
// }

void BtuLinear::voltageControlHelper(float setDuty, int ctrl) {
    // PwmOut* actPwm;
    // DigitalOut* actDir;
    // if(ctrl == ACT_A) {
    //     actPwm = &m_actAPwm;
    //     actDir = &m_actADir;
    // } else {
    //     actPwm = &m_actBPwm;
    //     actDir = &m_actBDir;
    // }
    if(ctrl == ACT_A) {
        if(setDuty > 0) {
            m_actAPwm = setDuty;
            m_actADir = 1;
        } else {
            m_actADir = 0;
            m_actAPwm = -setDuty;
        }
    }

    if(ctrl == ACT_B) {
        if(setDuty > 0) {
            m_actBPwm = setDuty;
            m_actBDir = 1;
        } else {
            m_actBDir = 0;
            m_actBPwm = -setDuty;
        }
    }
    // if(setDuty > 0) {
    //     *actPwm = setDuty;
    //     *actDir = 1;
    // } else {
    //     *actDir = 0;
    //     *actPwm = -setDuty;
    // }
}

void BtuLinear::voltageControl(float setDuty) {
    voltageControlHelper(setDuty, ACT_A);
    voltageControlHelper(setDuty, ACT_B);
}

void BtuLinear::updatePositionReadings() {
    float aPosition = m_actAPot;
    float bPosition = m_actBPot;

    float aOldPos = m_avg_windowA[m_avg_windowPtr];
    float bOldPos = m_avg_windowB[m_avg_windowPtr];
    m_avg_windowA[m_avg_windowPtr] = aPosition;
    m_avg_windowB[m_avg_windowPtr] = bPosition;
    if(m_avg_windowSize >= AVG_WINDOW_WIDTH) {
        m_currentAvgA = m_currentAvgA + (aPosition / AVG_WINDOW_WIDTH)- (aOldPos / AVG_WINDOW_WIDTH);
        m_currentAvgB = m_currentAvgB + (bPosition / AVG_WINDOW_WIDTH)- (bOldPos / AVG_WINDOW_WIDTH);
    } else {
        m_avg_windowSize++;
        m_currentAvgA = 0;
        m_currentAvgB = 0;
        for(int i = 0; i < m_avg_windowSize; i++) {
            m_currentAvgA = (m_avg_windowA[i] / m_avg_windowSize);
            m_currentAvgB = (m_avg_windowB[i] / m_avg_windowSize);
        }
    }
}

float BtuLinear::getActPosition(int act) {
    // updatePositionReadings();
    // float position;
    // if(act == ACT_A) {
    //     position = m_currentAvgA;
    // } else {
    //     position = m_currentAvgB;
    // }
    float position;
    if(act == ACT_A) {
        position = m_actAPot;
    } else {
        position = m_actBPot;
    }
    float scaledPos = (position - POT_MIN) / (POT_MAX - POT_MIN);
    return scaledPos;
}

void BtuLinear::velocityControl(float setVel) {
    velocityControlHelper(setVel, ACT_A);
    velocityControlHelper(setVel, ACT_B);
}

float btu_abs(float a) {
    return (a >= 0) ? a : -a;
}

void BtuLinear::velocityControlHelper(float setVel, int ctrl) {
    // float pos = m_motorServo.readPosition();
    if(ctrl != ACT_A && ctrl != ACT_B) {
        return;
    }
    float pos;
    float deltaVolt;
    float cmdVolt;
    if(ctrl == ACT_A) {
        pos = getActPosition(ACT_A);
        m_velAPid.setProcessValue((pos - m_oldPosA) / PID_FREQ);
        m_velAPid.setSetPoint(setVel);
        deltaVolt = m_velAPid.compute();
        m_oldPosA = pos;
    } else if(ctrl == ACT_B){
        pos = getActPosition(ACT_B);
        m_velBPid.setProcessValue((pos - m_oldPosB) / PID_FREQ);
        m_velBPid.setSetPoint(setVel);
        deltaVolt = m_velBPid.compute();
        m_oldPosB = pos;
    }
    m_currentVoltage += deltaVolt;
    cmdVolt = m_currentVoltage;
    if(btu_abs(cmdVolt) <= VOLTAGE_THRESHOLD || (getActPosition(ctrl) <= 0.01 && setVel < 0) || (getActPosition(ctrl) >= 0.99 && setVel > 0)) {
        cmdVolt = 0;
    }
    voltageControlHelper(cmdVolt, ctrl);

}

void BtuLinear::positionControlHelper(float setPosDeg, int ctrl) {
    if(ctrl == ACT_A) {
        m_posAPid.setSetPoint(setPosDeg);
        // m_specPid.setProcessValue(m_motorServo.readPosition());
        m_posAPid.setProcessValue(getActPosition(ACT_A));
        float cmdVelA = m_posAPid.compute();
        // m_specCmd = (int)(cmdVel >= 0); // just for test recording purposes
        velocityControlHelper(cmdVelA, ACT_A);
    } else {
        m_posBPid.setSetPoint(setPosDeg);
        // m_specPid.setProcessValue(m_motorServo.readPosition());
        m_posBPid.setProcessValue(getActPosition(ACT_B));
        float cmdVelB = m_posBPid.compute();
        // m_specCmd = (int)(cmdVel >= 0); // just for test recording purposes
        velocityControlHelper(cmdVelB, ACT_B);
    }

}

void BtuLinear::positionControl(float setPos) {
    positionControlHelper(setPos, ACT_A);
    positionControlHelper(setPos, ACT_B);
}

void BtuLinear::depthControlHelper(float cmdVelocity) {
    velocityControlHelper(cmdVelocity, ACT_A);
    positionControlHelper(getActPosition(ACT_A), ACT_B);
}

void BtuLinear::depthControl(float setDepthMeters) {
    m_depthPid.setSetPoint(setDepthMeters);

    float curDepth = getDepth();

    m_depthPid.setProcessValue(curDepth);

    float cmdVel = m_depthPid.compute();
    depthControlHelper(cmdVel);
    // velocityControl(cmdVel);
}

float BtuLinear::getDepth() {
    float pvDepth = getPressure();
    float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
    return pvDepthMeters;
}

// float BTU::getServoPos() {
// 	return m_motorServo.readPosition();
// }
