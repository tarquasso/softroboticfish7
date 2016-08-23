#ifndef ACTUATOR_H
#define ACTUATOR_H

#include "mbed.h"
#include "PidController.h"
#include "utility.h"
#include "MovingAverage.h"

#define VOLTAGE_THRESHOLD 0.01

#define POS_KC 8
#define POS_KI 2
#define POS_KD 1

#define VEL_KC 0.5
#define VEL_KI 0
#define VEL_KD 0

#define POT_MIN 0.04
#define POT_MAX 0.96

#define POS_MIN 0.0
#define POS_MAX 1.0

#define VEL_MIN -1.0
#define VEL_MAX 1.0

#define VOLT_MIN -1.0
#define VOLT_MAX 1.0

/**
 * Linear Actuator Control Class
 */
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
    float m_currentPosition;

 public:
    /**
     * Constructor.
     *
     * Uses the pwmPin and dirPin to command movement of the Actuator.
     *
     * Uses the potPin to read the position of the Actuator.
     *
     * Can be used to run PID control for position, velocity, or setDuty
     *
     * @param pwmPin mbed pin for pwm used to run the actuator.
     * @param dirPin mbed pin for boolean used to determine which direction to move the actuator.
     * @param potPin mbed pin for the potentiometer used to read the extension of the actuator.
     * @param timestep the expected length of a timestep in the controllers.  Used for velocity and derivative calculations.
     */
    Actuator(PinName pwmPin, PinName dirPin, PinName potPin, float timestep);
    ~Actuator();

    /**
     * Reset the controllers
     *
     * Resets all the controllers and starts over the Moving Average Window
     */
    void reset();

    /**
     * Returns the last recorded position of the Actuator.
     *
     * Note: User needs to update the last recorded position using updatePosition to get non-stale results
     * @return Actuator position in units of full stroke (i.e. 1.0 is full extension, 0.5 is half extension).
     */
    float getPosition();

    /**
     * Sets the tunings (gains) for the Position PID Controller
     *
     * @param kc The proportional term gain.
     * @param kI The integral term gain.
     * @param kD the derivative term gain.
     */
    void setPosTunings(float kc, float kI, float kD);


    /**
     * Sets the tunings (gains) for the Velocity PID Controller
     *
     * @param kc The proportional term gain.
     * @param kI The integral term gain.
     * @param kD the derivative term gain.
     */
    void setVelTunings(float kc, float kI, float kD);

    /**
     * Controls the actuator pwm and direction signals.  A negative duty Cycle indicates movement in the opposite direction.
     *
     * @param setDuty The duty cycle for the pwm (negative indicates moving back)
     */
    void runVoltControl(float setDuty);

    /**
     * Controls the actuator velocity.  Uses a PID controller to maintain velocity.
     *
     * @param setVel The desired Velocity of the actuator (in terms of stroke lengths per second)
     */
    void runVelControl(float setVel);

    /**
     * Controls the actuator position.  Uses a PID controller to control position.
     *
     * @param setPos The desired Position of the actuator (in terms of the stroke length)
     */
    void runPosControl(float setPos);

    /**
     * Reads the position of the actuator and updates the Moving Average
     *
     */
    void updatePosition();
};

#endif
