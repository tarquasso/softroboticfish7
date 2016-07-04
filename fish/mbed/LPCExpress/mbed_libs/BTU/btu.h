#ifndef BTU_H
#define BTU_H

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
#define PID_FREQ_NOT_USED 0.1 // placeholder;rate at which the PID is run is not used here but should be used when BTU::Run() is called. BTU::Run() only computes PID once

#define SERVO_CONNECTED true
#define SERVO_PWM_WIDTH 0.0010
#define SERVO_DEGREE_WIDTH 91.0

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
class BTU
{
 /* private: */
 public:
    PwmOut 	m_pwm1;
    PwmOut 	m_pwm2;
    QEI     m_encoder_bcu_motor;
    PID     m_posPid;
    PID     m_depthPid;
    MS5837  m_pressureSensor;
    Servo   m_motorServo;
    /* Ticker  m_timer; */
    int m_mode;
    float m_setval;
    float m_kc;
    float m_taui;
    float m_taud;
    float m_currentval;
    float m_cmdVoltage;
    int m_counter;
    float m_pvDepth;
    float m_pvDepthMeters;
    float m_cmdPosDeg;
    float m_pAtmosMbar;
    float m_pWaterNoDepthMbar;
    float m_setPressure;

    BTU();
    ~BTU();
    /* void run(); */
    void init(float timeStep);
    void stop();
    void update(int mode, float setVal, float kc, float taui, float taud);
    void updateMode(int mode);
    void runCycle();
    void updateAndRunCycle(int mode, float value);
    void voltageControl(float setDuty);
    void positionControl(float setPosDeg);
    void depthControl(float setDepthMeters);
    void voltageDefault();

};

#endif
