#ifndef BTULINEAR_H
#define BTULINEAR_H

#include "mbed.h"
#include "QEI.h"
#include "MS5837.h" // pressure sensor
#include "PidController.h"
#include "utility.h"
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

#define VEL_MIN -1.0          /* servoLen/sec */
#define VEL_MAX 1.0

#define POS_MIN -1.0
#define POS_MAX 1.0

#define PERIOD_PWM 0.00345

#define DEP_K_C 1.0
#define DEP_TAU_I 0.0
#define DEP_TAU_D 0.0
#define PID_FREQ 0.05

#define SP_K_C 8.0
#define SP_TAU_I 0.0
#define SP_TAU_D 0.0

#define VEL_K_C 1
#define VEL_TAU_I 0
#define VEL_TAU_D 0

/* #define SERVO_PWM_WIDTH 0.0006 */
/* #define SERVO_DEGREE_WIDTH 91.0 */

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

#define POT_MIN 0.01
#define POT_MAX 0.98

#define AVG_WINDOW_WIDTH 5

#define VOLTAGE_THRESHOLD 0.05
/**
 * This class is used for controlling and accessing data from the Buoyancy Test Unit
 * It includes instances of the classes PwmOut
 */
class BtuLinear {
private:
  PidController m_depthPid;
  PidController m_posAPid;
  PidController m_posBPid;
  PidController m_velAPid;
  PidController m_velBPid;
  MS5837 m_pressureSensor;
  int m_mode;
  float m_kc, m_tauI, m_tauD;
  float m_v_kc, m_v_tauI, m_v_tauD;
  float m_p_kc, m_p_tauI, m_p_tauD;
  PwmOut m_actAPwm;
  PwmOut m_actBPwm;
  AnalogIn m_actAPot;
  AnalogIn m_actBPot;
  DigitalOut m_actADir;
  DigitalOut m_actBDir;
  float m_oldPosA;
  float m_oldPosB;
  float m_currentVoltage;

  int m_avg_windowPtr;
  int m_avg_windowSize;

  float m_avg_windowA[AVG_WINDOW_WIDTH];
  float m_currentAvgA;

  float m_avg_windowB[AVG_WINDOW_WIDTH];
  float m_currentAvgB;

  /* void positionControl(float setPosDeg); */
  void voltageControl(float setDuty);
  void voltageControlHelper(float setDuty, int ctrl);
  void velocityControl(float setVel);
  void velocityControlHelper(float setVelocity, int ctrl);
  void positionControl(float setPos);
  void positionControlHelper(float setPos, int ctrl);
  void depthControl(float setDepthMeters);
  void depthControlHelper(float cmdVelocity);
  void updatePositionReadings();

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
  float getKc() { return m_kc; };
  float getTauI() { return m_tauI; };
  float getTauD() { return m_tauD; };
  float getVKc() {return m_v_kc; };
  float getVTauI() { return m_v_tauI; };
  float getVTauD() { return m_v_tauD; };
  float getPKc() { return m_p_kc; };
  float getPTauI() { return m_p_tauI; };
  float getPTauD() { return m_p_tauD; };

};

#endif
