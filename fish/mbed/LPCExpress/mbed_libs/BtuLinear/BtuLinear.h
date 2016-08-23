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
  /* void updatePositionReadings(); */

protected:

  /**
   * Controls the duty cycle of both Actuators simultaneously.
   */
  void voltageControl(float setDuty);

  /**
   * Controls the velocity of both Actuators simultaneously.
   */
  void velocityControl(float setVel);

  /**
   * Controls the positions of both Actuators simultaneously.
   */
  void positionControl(float setPos);

  /**
   * Controls the depth of the BCU using the set duty of the Actuators.  Uses a PID controller on depth error.
   */
  void depthControl(float setDepthMeters);

  /**
   * Helper function to control the actuators in depth control mode.
   */
  void depthControlHelper(float cmdVoltage);

  /**
   * Controls the ascent/descent velocity of the BCU using PID control.
   */
  void vvControl(float setVelMeters);


public:

  /**
   * Constructor.
   *
   * Sets up and initializes two Actuators and the Depth and VV PID controllers.  Uses dryRun to determine source of depth readings.
   *
   * @param dryRun True means the BCU should get its depth readings from a potentiometer.  False means it uses the pressure sensor.
   */
  BtuLinear(bool dryRun = true);
  ~BtuLinear();

  /**
   * Initializes PID controllers, Actuators, and the PressureSensor.
   */
  void init();

  /**
   * Resets the PIDs and Actuators
   */
  void stop();

  /**
   * Sets the Tunings (gains) of the Depth PID controller
   *
   * @param kc The proportional term gain.
   * @param kI The integral term gain.
   * @param kD the derivative term gain.
   */
  void updateDepthTunings(float kc, float kI, float kD);

  /**
   * Sets the Tunings (gains) of the Actuator Position PID controller
   *
   * @param kc The proportional term gain.
   * @param kI The integral term gain.
   * @param kD the derivative term gain.
   */
  void updatePosTunings(float kc, float kI, float kD);

  /**
   * Sets the Tunings (gains) of the Actuator Velocity PID controller
   *
   * @param kc The proportional term gain.
   * @param kI The integral term gain.
   * @param kD the derivative term gain.
   */
  void updateVelTunings(float kc, float kI, float kD);

  /**
   * Sets the Tunings (gains) of the Descent Velocity PID controller
   *
   * @param kc The proportional term gain.
   * @param kI The integral term gain.
   * @param kD the derivative term gain.
   */
  void updateVVTunings(float kc, float kI, float kD);

  /**
   * Returns the most recently recorded position from an Actuator.
   *
   * @param act The actuator to get position readings from (ACT_A or ACT_B).
   * @return The most recently recorded position from the specified actuator
   */
  float getActPosition(int act = ACT_A);

  /**
   * Changes the control mode of the BCU
   *
   * @param mode Control mode to change to (i.e. DEPTH_CTRL_MODE).
   */
  void updateMode(int mode);

  /**
   * Runs one cycle of the BCU control loop
   *
   * @param setVal the desired value to set the controller to.
   */
  void runCycle(float setVal);

  /**
   * Updates mode and then runs one cycle of the BCU control loop
   *
   * @param mode Control mode to use (i.e. DEPTH_CTRL_MODE).
   * @param value desired value to set the controller to.
   */
  void updateAndRunCycle(int mode, float value);

  /**
   * Gets the most recently recorded pressure reading
   *
   * @return The most recently recorded pressure reading
   */
  float getPressure();

  /**
   * Returns the current depth reading, calculated from either the pressure sensor or a potentiometer, depending on if the BCU is in Dry Run Mode
   *
   * @return The current depth reading
   */
  float getDepth();

  /**
   * Set Dry Mode.  When Dry Mode is true, depth readings come from a potentiometer.  When false, readings come from the pressure sensor.
   *
   * @param dryMode Whether to set Dry Mode to True
   */
  void setDryMode(bool);

  /**
   * Returns the current control mode of the BCU
   *
   * @return current control mode
   */
  int getMode() { return m_mode; };

  /**
   * Returns the current Depth Proportional Gain
   *
   * @return Depth Proportional Gain
   */
  float getDkC() { return m_kc; };

  /**
   * Returns the current Depth Integral Gain
   *
   * @return Depth Integral Gain
   */
  float getDkI() { return m_kI; };
  /**
   * Returns the current Depth Derivative Gain
   *
   * @return Depth Derivative Gain
   */
  float getDkD() { return m_kD; };

  /**
   * Returns the current Actuator Velocity Proportional Gain
   *
   * @return Velocity Proportional Gain
   */
  float getVkC() { return m_v_kc; };

  /**
   * Returns the current Actuator Velocity Integral Gain
   *
   * @return Velocity Integral Gain
   */
  float getVkI() { return m_v_kI; };

  /**
   * Returns the current Actuator Velocity Derivative Gain
   *
   * @return Velocity Derivative Gain
   */
  float getVkD() { return m_v_kD; };

  /**
   * Returns the current Actuator Position Proportional Gain
   *
   * @return Position Proportional Gain
   */
  float getPkC() { return m_p_kc; };

  /**
   * Returns the current Actuator Position Integral Gain
   *
   * @return Position Integral Gain
   */
  float getPkI() { return m_p_kI; };

  /**
   * Returns the current Actuator Position Derivative Gain
   *
   * @return Position Derivative Gain
   */
  float getPkD() { return m_p_kD; };

  /**
   * Returns the current Descent Velocity Proportional Gain
   *
   * @return Descent Velocity Proportional Gain
   */
  float getVVkC() { return m_vv_kc; };

  /**
   * Returns the current Descent Velocity Integral Gain
   *
   * @return Descent Velocity Integral Gain
   */
  float getVVkI() { return m_vv_kI; };

  /**
   * Returns the current Descent Velocity Derivative Gain
   *
   * @return Descent Velocity Derivative Gain
   */
  float getVVkD() { return m_vv_kD; };

  /**
   * Returns current Descent Velocity
   *
   * @return current Descent Velocity
   */
  float getCurrentVel() {
    return m_currentVel;
  };

  /**
   * Returns current Descent Acceleration (subject to a lot of noise)
   *
   * @return current Descent Acceleration
   */
  float getCurrentAccel() {
    return m_currentAccel;
  };

};

#endif
