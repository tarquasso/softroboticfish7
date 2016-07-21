#ifndef BTULINEAR_H
#define BTULINEAR_H

#include "mbed.h"
#include "MS5837.h" // pressure sensor
#include "PidController.h"
#include "utility.h"
#include "Actuator.h"
/* #include "Servo.h" */


#define P_ATMOS_MBAR 1000
#define P_WATER_SURFACE_MBAR 98.02

#define VOLTAGE_CTRL_MODE 1
#define VELOCITY_CTRL_MODE 2
#define DEPTH_CTRL_MODE 4
#define POSITION_CTRL_MODE 3
#define DEFAULT_CTRL_MODE VELOCITY_CTRL_MODE

#define DEPTH_MIN 0
#define DEPTH_MAX 5

/* #define PERIOD_PWM 0.00345 */

#define DEP_K_C 1.0
#define DEP_TAU_I 0.0
#define DEP_TAU_D 0.0

#define PID_FREQ 0.05

#define PIN_IMU_SDA p28
#define PIN_IMU_SCL p27
#define PIN_ACTA_POT p16
#define PIN_ACTB_POT p20
#define PIN_ACTA_PWM p22
#define PIN_ACTB_PWM p21
#define PIN_ACTA_DIR p29
#define PIN_ACTB_DIR p30
/* #define PIN_PWM_SERVO p23 */

#define TIME_STEP_MIN 0.01

#define ACT_A 1
#define ACT_B 2

#define AVG_WINDOW_WIDTH 5

/**
 * This class is used for controlling and accessing data from the Buoyancy Test Unit
 * It includes instances of the classes PwmOut
 */
class BtuLinear {
private:
  PidController m_depthPid;
  Actuator m_actA;
  Actuator m_actB;
  MS5837 m_pressureSensor;
  int m_mode;
  float m_kc, m_tauI, m_tauD;
  float m_v_kc, m_v_tauI, m_v_tauD;
  float m_p_kc, m_p_tauI, m_p_tauD;

  /* int m_avg_windowPtr; */
  /* int m_avg_windowSize; */

  /* float m_avg_windowA[AVG_WINDOW_WIDTH]; */
  /* float m_currentAvgA; */

  /* float m_avg_windowB[AVG_WINDOW_WIDTH]; */
  /* float m_currentAvgB; */

  void voltageControl(float setDuty);
  void velocityControl(float setVel);
  void positionControl(float setPos);
  void depthControl(float setDepthMeters);
  void depthControlHelper(float cmdVoltage);
  /* void updatePositionReadings(); */

protected:


public:
  BtuLinear();
  ~BtuLinear();
  void init();
  void stop();
  void updateDepthTunings(float kc, float tauI, float tauD);
  void updatePosTunings(float kc, float tauI, float tauD);
  void updateVelTunings(float kc, float tauI, float tauD);
  float getActPosition(int act);
  void updateMode(int mode);
  void runCycle(float setVal);
  void updateAndRunCycle(int mode, float value);
  float getPressure();
  float getDepth();
  int getMode() { return m_mode; };
  float getDKc() { return m_kc; };
  float getDTauI() { return m_tauI; };
  float getDTauD() { return m_tauD; };
  float getVKc() {return m_v_kc; };
  float getVTauI() { return m_v_tauI; };
  float getVTauD() { return m_v_tauD; };
  float getPKc() { return m_p_kc; };
  float getPTauI() { return m_p_tauI; };
  float getPTauD() { return m_p_tauD; };

};

#endif
