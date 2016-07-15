#ifndef BTU_H
#define BTU_H

#include "mbed.h"
#include "QEI.h"
#include "PIDControl/PIDControl.h"
#include "MS5837.h" // pressure sensor
/* #include "Servo.h" */


#define P_ATMOS_MBAR 1000
#define P_WATER_SURFACE_MBAR 98.02

#define VOLTAGE_CTRL_MODE 1
#define VELOCITY_CTRL_MODE 2
#define DEPTH_CTRL_MODE 3
#define SPEC_POSITION_CTRL_MODE 4

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

#define SP_K_C 23.7
#define SP_TAU_I 0.175
#define SP_TAU_D 0.05

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

#define POT_MIN 0.02
#define POT_MAX 0.97

#define AVG_WINDOW_WIDTH 5

#define VOLTAGE_THRESHOLD 0.05
/**
 * This class is used for controlling and accessing data from the Buoyancy Test Unit
 * It includes instances of the classes PwmOut
 */
class BTU {
private:
  PID m_depthPid;
  PID m_posAPid;
  PID m_posBPid;
  PID m_velAPid;
  PID m_velBPid;
  MS5837 m_pressureSensor;
  int m_mode;
  float m_kc, m_tauI, m_tauD;
  float m_v_kc, m_v_tauI, m_v_tauD;
  float m_p_kc, m_p_tauI, m_p_tauD;
  PwmOut m_actAPwm;
  PwmOut m_actBPwm;
  AnalogIn m_actBPot;
  AnalogIn m_actAPot;
  DigitalOut m_actADir;
  DigitalOut m_actBDir;
  float m_oldPosA;
  float m_oldPosB;
  float avg_window[AVG_WINDOW_WIDTH];
  int avg_windowPtr;
  int avg_windowSize;
  float currentAvg;

  /* void positionControl(float setPosDeg); */
  void voltageControl(float setDuty);
  void voltageControlHelper(float setDuty, int ctrl);
  void velocityControl(float setVel);
  void velocityControlHelper(float setVel, int ctrl);
  void depthControl(float setDepthMeters);
  void specialPosControl(float setPosDeg);

public:
  BTU();
  ~BTU();
  void init();
  void stop();
  void update(int mode, float kc, float tauI, float tauD);
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
