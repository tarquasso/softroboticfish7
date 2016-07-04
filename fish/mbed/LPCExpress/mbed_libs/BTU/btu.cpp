#include "BTU/btu.h"

// The static instance
// Buoyancy_Unit m_BTU;

BTU::BTU():
    m_pwm1(PIN_PWM_OUT1),
	m_pwm2(PIN_PWM_OUT2),
	m_encoder_bcu_motor(PIN_ENCODER_A, PIN_ENCODER_B, NC, PULSEPERREV, QEI::X4_ENCODING),
	m_posPid(POS_K_C, POS_TAU_I, POS_TAU_D, PID_FREQ_NOT_USED),
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ_NOT_USED),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
    m_motorServo(PIN_PWM_SERVO)
{};

BTU::~BTU(){}

void BTU::init(float timeStep) {

	// calculate depth to mbar
	float pAtmos = 101325;
	float pa_to_mbar = 0.01;
	m_pAtmosMbar = pAtmos * pa_to_mbar;
	float densityWater = 1000;
	float gravity = 9.802;
	float pWaterNoDepth =  (gravity*densityWater);
	m_pWaterNoDepthMbar = pa_to_mbar * pWaterNoDepth;

	m_setPressure = 0.0;
	m_currentval = 0.0;
	m_cmdVoltage = 0.0;
	m_mode = 2; //pos control
	m_kc = 0;
	m_taui = 0;
	m_taud = 0;
	m_setval = 0;
	this->voltageDefault();

	m_depthPid.setMode(1); // nonzero: AUTO
	m_depthPid.setInputLimits(0.0, MAXDEPTH); // analog input of position to be scaled 0-100%
	m_depthPid.setOutputLimits(-360, 360);
	m_depthPid.setInterval(timeStep);

	if (SERVO_CONNECTED) {

		m_depthPid.setOutputLimits(-SERVO_DEGREE_WIDTH, SERVO_DEGREE_WIDTH);
		m_motorServo.calibrate(SERVO_PWM_WIDTH, SERVO_DEGREE_WIDTH);
		m_pressureSensor.MS5837Init();
	} else {
		m_posPid.setMode(1);
		m_posPid.setInputLimits(-360, 360);
		m_posPid.setOutputLimits(-1, 1);
	}

	m_pvDepth = 1100.0;
	m_pvDepthMeters = (m_pvDepth-m_pAtmosMbar)/m_pWaterNoDepthMbar;
	m_cmdPosDeg = 0.0;
}

void BTU::stop() {
    m_pwm1 = 0;
    m_pwm2 = 0;
}

void BTU::update(int mode, float setVal, float kc, float taui, float taud) {

	this->updateMode(mode);
    m_setval = setVal;
    m_kc = kc;
    m_taui = taui;
    m_taud = taud;
}

void BTU::updateMode(int mode) {
    if (m_mode != mode) {
    	this->stop();
    	m_mode = mode;
    	m_posPid.reset();
    	m_depthPid.reset();
    	// TODO: maybe reset PID controllers?
    }

}

void BTU::runCycle() {
    switch (m_mode) {

    case VOLTAGE:
        voltageControl(m_setval);
        break;

    case POSITION:
        positionControl(m_setval);
        break;

    case DEPTH:
        depthControl(m_setval);
        break;
    }
}

void BTU::updateAndRunCycle(int mode, float value) {
    update(mode, value, m_kc, m_taui, m_taud);
    runCycle();
}



void BTU::voltageControl(float setDuty) {
    if (setDuty > 0) {          // CCW
        m_pwm1 = setDuty;
        m_pwm2 = 0;
    } else {
        m_pwm1 = 0;
        m_pwm2 = -setDuty;
    }
}



void BTU::positionControl(float setPosDeg) {
	if (SERVO_CONNECTED) {
		m_motorServo.position(setPosDeg);
		//m_motorServo.write(1.0);
		m_currentval = m_motorServo.readPosition();
		return;
	} else {
		m_posPid.setTunings(m_kc, m_taui, m_taud);
		m_posPid.setSetPoint(setPosDeg);
		// Detect motor position
		float pvPos = m_encoder_bcu_motor.getPulses() % PULSEPERREV;
		float pvDeg = pvPos / PULSEPERREV * 360;

		if (pvPos > PULSEPERREV) // rotated more than 360deg CCW
		{
			m_currentval = 360;
		} else if (pvPos < -PULSEPERREV) // rotated more than 360deg CW
		{
			m_currentval = -360;
		} else {
			m_currentval = pvDeg;
		}
		// Set motor voltage
		m_posPid.setProcessValue(m_currentval); // update the process variable
		m_cmdVoltage = m_posPid.compute(); //commanded voltage value
		this->voltageControl(m_cmdVoltage); // change voltage provided to motor
	}
}

void BTU::depthControl(float setDepthMeters) {
    m_depthPid.setTunings(m_kc, m_taui, m_taud);

    // Run PID
    m_setPressure = (m_pAtmosMbar + m_pWaterNoDepthMbar * setDepthMeters);
    m_depthPid.setSetPoint(m_setPressure); // we want the process variable to be the desired value

    // Detect depth
    m_pressureSensor.Barometer_MS5837();
    m_pvDepth = m_pressureSensor.MS5837_Pressure();
   // m_pvDepth = (m_pAtmosMbar + m_pWaterNoDepthMbar * setDepthMeters);
    m_pvDepthMeters = (m_pvDepth-m_pAtmosMbar)/m_pWaterNoDepthMbar;
    // Set motor position
    m_depthPid.setProcessValue(m_pvDepth); // update the process variable
    m_cmdPosDeg = m_depthPid.compute(); // set the new output of position
    //pc.printf("setDepth: %.1f mbar, pvPos: %.1f mbar, setPos: %.2f deg\n",setDepth, pvDepth, SETVAL);
    this->positionControl(m_cmdPosDeg); // change position of motor
}

void BTU::voltageDefault() {
    m_pwm1.period(0.00345); // 3.45ms period
    m_pwm2.period(0.00345); // 3.45ms period
    m_pwm1 = 0;           // duty cycle of 50%
    m_pwm2 = 0;
}
