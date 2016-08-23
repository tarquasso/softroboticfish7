#ifndef ACTUATORSERVO_H
#define ACTUATORSERVO_H

#include "mbed.h"
#include "PidController.h"
#include "utility.h"
#include "Servo.h"

#define VOLTAGE_THRESHOLD 0.01

#define POS_KC 16
#define POS_KI 2
#define POS_KD 1

#define VEL_KC 0.5
#define VEL_KI 0
#define VEL_KD 0

#define POT_MIN 0.00
#define POT_MAX 1.00

#define POS_MIN 0.0
#define POS_MAX 1.0

#define VEL_MIN -1.0
#define VEL_MAX 1.0

#define VOLT_MIN -1.0
#define VOLT_MAX 1.0


class ActuatorServo {
 private:
    PidController m_posPid;
    PidController m_velPid;
    float m_timestep;
    float m_oldPos;
    float m_currentVoltage;
    Servo m_servo;
    float m_currentPosition;

 public:
    ActuatorServo(PinName servoPin, float freq);
    ~ActuatorServo();
    void reset();
    float getPosition();
    void setPosTunings(float kc, float kI, float kD);
    void setVelTunings(float kc, float kI, float kD);
    void runVoltControl(float setDuty);
    void runVelControl(float setVel);
    void runPosControl(float setPos);
};

#endif
