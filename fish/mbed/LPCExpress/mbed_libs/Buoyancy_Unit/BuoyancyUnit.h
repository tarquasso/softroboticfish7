#ifndef BuoyancyUnit_H
#define BuoyancyUnit_H

#include "mbed.h"
#include "QEI.h"
#include "PID.h"
#include "MS5837.h" // pressure sensor
#include "Servo.h"

// Encoder number of pulses per revolution
#define PULSEPERREV 11837 // (pulses per rev * gear ratio)
// Max pressure to be experienced (mbar); currently at ~5m under water
#define MAXDEPTH 1516
#define PERIOD_PWM 0.00345 //period of PWM
// Position PID parameters
#define POS_K_C 1
#define POS_TAU_I 1
#define POS_TAU_D 1
// Depth PID parameters
#define DEP_K_C 0.1
#define DEP_TAU_I 0.1
#define DEP_TAU_D 0.1
#define PID_FREQ 0.1 // placeholder;rate at which the PID is run is not used here but should be used when BTU::Run() is called. BTU::Run() only computes PID once

#define SERVO_CONNECTED true
#define SERVO_PWM_WIDTH 0.0007
#define SERVO_DEGREE_WIDTH 82.5

#define VOLTAGE 	1
#define POSITION	2
#define DEPTH		3

// Define pinouts from mbed
#define PIN_ENCODER_A p16
#define PIN_ENCODER_B p17
#define PIN_PWM_OUT1 p21
#define PIN_PWM_OUT2 p22
#define PIN_IMU_SDA p28
#define PIN_IMU_SCL p27
#define PIN_PWM_SERVO p23

/**
 * This class sets the motor position to the desired state
 * It includes instances of the classes PwmOut, QEI, PID, and MS5837
 */
class Buoyancy_Unit
{
private:
    PwmOut 	m_pwm1;
    PwmOut 	m_pwm2;
    QEI     m_encoder_bcu_motor;
    PID     m_posPid;
    PID     m_depthPid;
    MS5837  m_pressureSensor;
    Servo   m_motorServo;
    Ticker  m_timer;
    int m_mode;
    float m_setval;
    float m_kc;
    float m_taui;
    float m_taud;
    float m_currentval;
    float m_output;
    int m_counter;

public:
    Buoyancy_Unit();
    ~Buoyancy_Unit();
    void run();
    void init();
    void stop();
    void update(int M, float A, float B, float C, float D);
    void updateMode(int mode);
    void voltageControl();
    void printValues();
    void voltageControlHelper(float setDuty);
    void positionControl();
    void depthControl();
    void voltageDefault();
};

#endif /* BTU_H */

// Create a static instance of BTU to be used by anyone controlling the Buoyancy Test Unit
extern Buoyancy_Unit m_BTU;
