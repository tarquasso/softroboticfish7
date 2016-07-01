#include "btu.h"

BTU::BTU():
    m_pwm1(PIN_PWM_OUT1),
	m_pwm2(PIN_PWM_OUT2),
	m_encoder_bcu_motor(PIN_ENCODER_A, PIN_ENCODER_B, NC, PULSEPERREV, QEI::X4_ENCODING),
	m_posPid(POS_K_C, POS_TAU_I, POS_TAU_D, PID_FREQ_NOT_USED),
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ_NOT_USED),
	m_pressureSensor(PIN_IMU_TX, PIN_IMU_RX),
    m_motorServo(PIN_PWM_SERVO)
{};

BTU::~BTU(){}

void BTU::init() {
    m_currentval = 555;
    m_output = 444;
    m_mode = 0;
    m_kc = 0;
    m_taui = 0;
    m_taud = 0;
    m_setval = 0;
    voltageDefault();

    m_posPid.setMode(1);
    m_posPid.setInputLimits(-360, 360);
    m_posPid.setOutputLimits(-1,1);

	m_depthPid.setMode(1); // nonzero: AUTO
	m_depthPid.setInputLimits(0.0, MAXDEPTH); // analog input of position to be scaled 0-100%
    m_depthPid.setOutputLimits(-360, 360);

    if(SERVO_CONNECTED) {
        m_posPid.setInputLimits(-82.5, 82.5);
        m_depthPid.setOutputLimits(-82.5, 82.5);
        m_motorServo.calibrate(SERVO_PWM_WIDTH, SERVO_DEGREE_WIDTH );
        m_pressureSensor.MS5837Init();
    }
}

void BTU::stop() {
    m_pwm1 = 0;
    m_pwm2 = 0;
}

void BTU::update(int mode, float setVal, float kc, float taui, float taud) {
    if (m_mode != mode) {
        updateMode(mode);
    }
    m_mode = mode;
    m_setval = setVal;
    m_kc = kc;
    m_taui = taui;
    m_taud = taud;
}

void BTU::updateMode(int mode) {
    stop();
    // TODO: maybe reset PID controllers?

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


float depthToMbar(float depth) {
  float pAtmos = 101000;
  float pWater =  (9.8 * depth * 1000);
  float pa_to_mbar = 0.01;
  return pa_to_mbar * (pAtmos + pWater);
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



void BTU::positionControl(float setPos) {
    if(SERVO_CONNECTED) {
        m_motorServo.position(setPos);
        m_currentval = m_motorServo.read();
        return;
    }
    m_posPid.setTunings(m_kc, m_taui, m_taud);
    m_posPid.setSetPoint(setPos);
        // Detect motor position
    float pvPos = m_encoder_bcu_motor.getPulses() % PULSEPERREV;
    float pvDeg = pvPos/PULSEPERREV*360;

    if (pvPos > PULSEPERREV) // rotated more than 360deg CCW
    {
    	m_currentval = 360;
    }
    else if (pvPos < -PULSEPERREV) // rotated more than 360deg CW
	{
		m_currentval = -360;
	}
    else
    {
    	m_currentval = pvDeg;
    }
    // Set motor voltage
    m_posPid.setProcessValue(m_currentval); // update the process variable
    m_output = m_posPid.compute();
    voltageControl(m_output); // change voltage provided to motor
}

void BTU::depthControl(float setDepth) {
    m_depthPid.setTunings(m_kc, m_taui, m_taud);
    // Run PID
    float setPressure = depthToMbar(setDepth);
    m_depthPid.setSetPoint(setPressure); // we want the process variable to be the desired value

    // Detect depth
    m_pressureSensor.Barometer_MS5837();
    float pvDepth = m_pressureSensor.MS5837_Pressure();

    // Set motor position
    m_depthPid.setProcessValue(pvDepth); // update the process variable
    float desiredPos = m_depthPid.compute(); // set the new output of position
    //pc.printf("setDepth: %.1f mbar, pvPos: %.1f mbar, setPos: %.2f deg\n",setDepth, pvDepth, SETVAL);
    positionControl(desiredPos); // change position of motor
}

void BTU::voltageDefault() {
    m_pwm1.period(0.00345); // 3.45ms period
    m_pwm2.period(0.00345); // 3.45ms period
    m_pwm1 = 0;           // duty cycle of 50%
    m_pwm2 = 0;
}
