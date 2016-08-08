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
    m_mvgAvg.reset();
    updatePosition();
    m_oldPos = getPosition();
}

void Actuator::updatePosition() {
	float position = m_actPot;
	float filteredPosition = m_mvgAvg.computeMovingAverage(position);
	m_currentPosition = filteredPosition;
}

float Actuator::getPosition() {

	return m_currentPosition;
    //float scaledPos = (filteredPosition - POT_MIN) / (POT_MAX - POT_MIN);
    //return scaledPos;
}

void Actuator::setPosTunings(float kc, float kI, float kD) {
    m_posPid.setTunings(kc, kI, kD);
}

void Actuator::setVelTunings(float kc, float kI, float kD) {
    m_velPid.setTunings(kc, kI, kD);
}

void Actuator::runVoltControl(float setDuty) {
	// clip commanded duty cycle to VOLT_MIN, VOLT_MAX range
	float cmdVolt = utility::clip(setDuty, VOLT_MIN, VOLT_MAX);

	// read in potentiometer position within 0.0 -> 1.0 range
	float pos = getPosition();

	// check if at limit and accordingly do not allow the driving voltage push actuator further than the limit
    if((pos <= POT_MIN && cmdVolt <= 0) || (pos >= POT_MAX && cmdVolt >= 0)) {
        cmdVolt = 0;
    }

    // check and set to zero if voltage is within dead zone
    //cmdVolt = utility::deadzone(cmdVolt, VOLTAGE_THRESHOLD);

    // set pwm values and direction pin for motor driver
    if(cmdVolt > 0) {
        m_actPwm = cmdVolt;
        m_actDir = 1;
    } else {
        m_actPwm = -cmdVolt;
        m_actDir = 0;
    }
}

void Actuator::runVelControl(float setVel) {
	// read in potentiometer position within 0.0 -> 1.0 range
	float pos = getPosition();

	// set velocity is commanded velocity TODO: clipping?
	float cmdVel = setVel;

	// check if at limit and accordingly do not allow driving velocity further than the limit
	if((pos <= POT_MIN && setVel < 0) || (pos >= POT_MAX && setVel > 0)) {
        cmdVel = 0;
    }

	// calculate current velocity by backward differentiation
    float currentVel = (pos - m_oldPos) / m_timestep;
    // current velocity set as process value of PID controller
    m_velPid.setProcessValue(currentVel);
    // commanded velocity as set point of PID controller
    m_velPid.setSetPoint(cmdVel);
    // delta in voltage as offset from current operating velocity point computed by PID
    float deltaVolt = m_velPid.compute();
    // store current pose at old pose for next iteration (needed for differentiation)
    m_oldPos = pos;
    // add delta in voltage to current voltage and clip at min max if necessary
    m_currentVoltage = utility::clip(m_currentVoltage + deltaVolt, VOLT_MIN, VOLT_MAX);
    runVoltControl(m_currentVoltage);
}

void Actuator::runPosControl(float setPos) {

	// clip commanded position
	float setPosClip = utility::clip(setPos, POT_MIN, POT_MAX);

	// read in potentiometer position within 0.0 -> 1.0 range
	float pos = getPosition();
	// current position set as process value of PID controller
    m_posPid.setProcessValue(pos);
    // commanded position as set point of PID controller
    m_posPid.setSetPoint(setPosClip);
    // commanded voltage as output from PID controller
    float cmdVolt = m_posPid.compute();

    // run voltage control with commanded voltage
    runVoltControl(cmdVolt);
}
