#include "Actuator.h"

Actuator::Actuator(PinName pwmPin, PinName dirPin, PinName potPin, float freq):
    m_posPid(POS_KC, POS_KI, POS_KD, freq, POS_MIN, POS_MAX, VOLT_MIN, VOLT_MAX, 0),
    m_velPid(VEL_KC, VEL_KI, VEL_KD, freq, VEL_MIN, VEL_MAX, VOLT_MIN, VOLT_MAX, 0),
    m_actPwm(pwmPin),
    m_actDir(dirPin),
    m_actPot(potPin)
{
    m_timestep = freq;
};

Actuator::~Actuator(){}

void Actuator::reset() {
    m_posPid.reset();
    m_velPid.reset();
    m_oldPos = getPosition();
}

float Actuator::getPosition() {
    float position = m_actPot;
    float scaledPos = (position - POT_MIN) / (POT_MAX - POT_MIN);
    return scaledPos;
}

void Actuator::setPosTunings(float kc, float kI, float kD) {
    m_posPid.setTunings(kc, kI, kD);
}

void Actuator::setVelTunings(float kc, float kI, float kD) {
    m_velPid.setTunings(kc, kI, kD);
}

void Actuator::runVoltControl(float setDuty) {
    float pos = getPosition();
    float cmdVolt = utility::clip(setDuty, VOLT_MIN, VOLT_MAX);

    if((pos <= POSITION_MIN && cmdVolt <= 0) || (pos >= POSITION_MAX && cmdVolt >= 0)) {
        cmdVolt = 0;
    }

    cmdVolt = utility::deadzone(cmdVolt, VOLTAGE_THRESHOLD);

    if(cmdVolt > 0) {
        m_actPwm = cmdVolt;
        m_actDir = 1;
    } else {
        m_actPwm = -cmdVolt;
        m_actDir = 0;
    }
}

void Actuator::runVelControl(float setVel) {
    float pos = getPosition();
    float cmdVel = setVel;
    if((pos <= POS_MIN && setVel < 0) || (pos >= POS_MAX && setVel > 0)) {
        cmdVel = 0;
    }

    float currentVel = (pos - m_oldPos) / m_timestep;
    m_velPid.setProcessValue(currentVel);
    m_velPid.setSetPoint(cmdVel);
    float deltaVolt = m_velPid.compute();
    m_oldPos = pos;

    m_currentVoltage = utility::clip(m_currentVoltage + deltaVolt, VOLT_MIN, VOLT_MAX);
}

void Actuator::runPosControl(float setPos) {
    float pos = getPosition();

    m_posPid.setProcessValue(pos);
    m_posPid.setSetPoint(setPos);
    float cmdVolt = m_posPid.compute();

    runVoltControl(cmdVolt);
}
