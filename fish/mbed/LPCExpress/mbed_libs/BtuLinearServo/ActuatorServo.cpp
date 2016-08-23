#include "ActuatorServo.h"

ActuatorServo::ActuatorServo(PinName servoPin, float freq):
    m_posPid(POS_KC, POS_KI, POS_KD, freq, POS_MIN, POS_MAX, VOLT_MIN, VOLT_MAX, 0),
    m_velPid(VEL_KC, VEL_KI, VEL_KD, freq, VEL_MIN, VEL_MAX, VOLT_MIN, VOLT_MAX, 0),
    m_servo(servoPin)

{
    m_timestep = freq;
    m_servo.calibrate(0.0005);
};

ActuatorServo::~ActuatorServo(){}

void ActuatorServo::reset() {
    m_posPid.reset();
    m_velPid.reset();
    m_oldPos = getPosition();
}



float ActuatorServo::getPosition() {

  return 1-m_servo.read();
    //float scaledPos = (filteredPosition - POT_MIN) / (POT_MAX - POT_MIN);
    //return scaledPos;
}

void ActuatorServo::setPosTunings(float kc, float kI, float kD) {
    m_posPid.setTunings(kc, kI, kD);
}

void ActuatorServo::setVelTunings(float kc, float kI, float kD) {
    m_velPid.setTunings(kc, kI, kD);
}

void ActuatorServo::runVoltControl(float setDuty) {
	// clip commanded duty cycle to VOLT_MIN, VOLT_MAX range
	// float cmdVolt = utility::clip(setDuty, VOLT_MIN, VOLT_MAX);

	// // read in potentiometer position within 0.0 -> 1.0 range
	// float pos = getPosition();

	// // check if at limit and accordingly do not allow the driving voltage push actuator further than the limit
    // if((pos <= POT_MIN && cmdVolt <= 0) || (pos >= POT_MAX && cmdVolt >= 0)) {
    //     cmdVolt = 0;
    // }

    // // check and set to zero if voltage is within dead zone
    // //cmdVolt = utility::deadzone(cmdVolt, VOLTAGE_THRESHOLD);

    // // set pwm values and direction pin for motor driver
    // if(cmdVolt > 0) {
    //     m_actPwm = cmdVolt;
    //     m_actDir = 1;
    // } else {
    //     m_actPwm = -cmdVolt;
    //     m_actDir = 0;
    // }
  return;
}

void ActuatorServo::runVelControl(float setVel) {
	// read in potentiometer position within 0.0 -> 1.0 range
	float pos = getPosition();

    runPosControl(pos + (setVel * m_timestep));
}

void ActuatorServo::runPosControl(float setPos) {
	float setPosClip = setPos;
    m_servo.write(1-setPosClip);
}
