#include "BTU/btu_PID_lite.h"

float clip(float val, float min, float max) {
  float newVal = (val > max) ? max : val;
  return (newVal < min) ? min : newVal;
}

BTU::BTU():
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ, DEPTH_MIN, DEPTH_MAX, VEL_MIN, VEL_MAX, 0),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_motorServo(PIN_PWM_SERVO)
{};

BTU::~BTU(){}

void BTU::init() {
  m_mode = 1;                   // pos control
  m_kc = DEP_K_C;
  m_tauI = DEP_TAU_I;
  m_tauD = DEP_TAU_D;

  m_motorServo.calibrate(SERVO_PWM_WIDTH, SERVO_DEGREE_WIDTH);
  m_pressureSensor.MS5837Init();
  m_pressureSensor.MS5837Start();
  wait(0.1);                    // remnant from old BTU class
}

float BTU::getPressure() {
  return m_pressureSensor.MS5837_Pressure();
}

void BTU::stop() {
  return;
}

void BTU::update(int mode, float kc, float tauI, float tauD) {
  updateMode(mode);
  m_kc = kc;
  m_tauI = tauI;
  m_tauD = tauD;
  m_depthPid.setTuning(kc, tauI, tauD);
}

void BTU::updateMode(int mode) {
  if(m_mode != mode) {
    stop();
    m_mode = mode;
    m_depthPid.reset();
  }
}

void BTU::runCycle(float setVal) {
  switch (m_mode) {

  case POSITION_CTRL_MODE:
    positionControl(setVal);
    break;

  case VELOCITY_CTRL_MODE:
    velocityControl(setVal);

  case DEPTH_CTRL_MODE:
    depthControl(setVal);
  }
}

void BTU::updateAndRunCycle(int mode, float value) {
  updateMode(mode);
  runCycle(value);
}


void BTU::positionControl(float setPosDeg) {
  float setPos = clip(setPosDeg, -SERVO_DEGREE_WIDTH, SERVO_DEGREE_WIDTH);
  m_motorServo.position(setPos);
  m_currentVal = m_motorServo.readPosition();
}

void BTU::velocityControl(float setVel) {
  float pos = m_motorServo.readPosition();
  m_motorServo.position(pos + setVel);
}


void BTU::depthControl(float setDepthMeters) {
  m_depthPid.setSetPoint(setDepthMeters);

  float curDepth = getDepth();

  m_depthPid.setProcessValue(curDepth);

  float cmdVel = m_depthPid.compute();
  velocityControl(m_cmdVel);
}

float BTU::getDepth() {
  float pvDepth = getPressure();
  float pvDepthMeters = (pvDepth - P_ATMOS_MBAR) / P_WATER_SURFACE_MBAR;
  return pvDepthMeters;
}
