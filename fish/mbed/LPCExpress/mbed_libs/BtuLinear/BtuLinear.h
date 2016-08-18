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
#define VV_CTRL_MODE 5
#define DEFAULT_CTRL_MODE POSITION_CTRL_MODE

#define DRY_RUN_POT_PIN p18


#define DEPTH_MIN 0
#define DEPTH_MAX 5

#define VV_MIN -0.3
#define VV_MAX 0.3

/* #define PERIOD_PWM 0.00345 */

#define DEP_KC 16.0
#define DEP_KI 2.0
#define DEP_KD 30.0

#define VV_KC 1.0
#define VV_KI 0.0
#define VV_KD 0.0

#define PID_FREQ 0.05

#define PIN_IMU_SDA p28
#define PIN_IMU_SCL p27
#define PIN_ACTA_POT p19
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
  PidController m_vvPid;
  Actuator m_actA;
  Actuator m_actB;
  MS5837 m_pressureSensor;
  AnalogIn m_dryRunPot;
  int m_mode;
  bool m_dryRun;
  float m_kc, m_kI, m_kD;
  float m_v_kc, m_v_kI, m_v_kD;
  float m_p_kc, m_p_kI, m_p_kD;
  float m_vv_kc, m_vv_kI, m_vv_kD;
  float m_oldDepth, m_oldVel, m_curDepth, m_currentVel, m_currentAccel;
  MovingAverage m_mvgDepthAvg;
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
  void vvControl(float setVelMeters);
  /* void updatePositionReadings(); */

protected:


public:

  BtuLinear(bool dryRun = true);
  ~BtuLinear();
  void init();
  void stop();
  void updateDepthTunings(float kc, float kI, float kD);
  void updatePosTunings(float kc, float kI, float kD);
  void updateVelTunings(float kc, float kI, float kD);
  void updateVVTunings(float kc, float kI, float kD);
  float getActPosition(int act = ACT_A);
  void updateMode(int mode);
  void runCycle(float setVal);
  void updateAndRunCycle(int mode, float value);
  float getPressure();
  float getDepth();
  void setDryMode(bool);
  void updateDepth();
  int getMode() { return m_mode; };
  float getDkC() { return m_kc; };
  float getDkI() { return m_kI; };
  float getDkD() { return m_kD; };
  float getVkC() {return m_v_kc; };
  float getVkI() { return m_v_kI; };
  float getVkD() { return m_v_kD; };
  float getPkC() { return m_p_kc; };
  float getPkI() { return m_p_kI; };
  float getPkD() { return m_p_kD; };
  float getVVkC() {return m_vv_kc; };
  float getVVkI() { return m_vv_kI; };
  float getVVkD() { return m_vv_kD; };
  float getCurrentVel() {
    	return m_currentVel;
    };

    float getCurrentAccel() {
    	return m_currentAccel;
    };

};

#endif
