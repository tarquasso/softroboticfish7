#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mbed.h"
#include "PidController.h"
#include "utility.h"


#define POS_KC 1
#define POS_TAUI 0
#define POS_TAUD 0

#define VEL_KC 0.5
#define VEL_TAUI 0
#define VEL_TAUD 0

#define POS_MIN 0.0
#define POS_MAX 1.0

#define VEL_MIN -1.0
#define VEL_MAX 1.0

#define VOLT_MIN -1.0
#define VOLT_MAX 1.0

#define POSITION_MAX 0.99
#define POSITION_MIN 0.01

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

 public:
    Actuator();
    ~Actuator();
    void reset();
    float getPosition();
    void updatePosTunings(float, float, float);
    void updateVelTunings(float, float, float);
    void runVoltControl(float);
    void runVelControl(float);
    void runPosControl(float);
}
