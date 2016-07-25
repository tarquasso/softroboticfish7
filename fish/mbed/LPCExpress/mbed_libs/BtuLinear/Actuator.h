#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mbed.h"
#include "PidController.h"
#include "utility.h"
#include "MovingAverage.h"

#define VOLTAGE_THRESHOLD 0.01

#define POS_KC 16
#define POS_KI 2
#define POS_KD 1

#define VEL_KC 0.5
#define VEL_KI 0
#define VEL_KD 0

#define POT_MIN 0.03
#define POT_MAX 0.97

#define VEL_MIN -1.0
#define VEL_MAX 1.0

#define VOLT_MIN -1.0
#define VOLT_MAX 1.0


class Actuator {
 private:
    PidController m_posPid;
    PidController m_velPid;
    PwmOut m_actPwm;
    DigitalOut m_actDir;
    AnalogIn m_actPot;
    float m_timestep;
    float m_oldPos;
    float m_currentVoltage;
    MovingAverage m_mvgAvg;

 public:
    Actuator(PinName pwmPin, PinName dirPin, PinName potPin, float freq);
    ~Actuator();
    void reset();
    float getPosition();
    void setPosTunings(float kc, float kI, float kD);
    void setVelTunings(float kc, float kI, float kD);
    void runVoltControl(float setDuty);
    void runVelControl(float setVel);
    void runPosControl(float setPos);
};

#endif
