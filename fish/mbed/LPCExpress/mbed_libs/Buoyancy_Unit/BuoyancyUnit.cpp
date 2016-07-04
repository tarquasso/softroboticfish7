#include <Buoyancy_Unit/BuoyancyUnit.h>

// The static instance
Buoyancy_Unit m_BTU;

Buoyancy_Unit::Buoyancy_Unit():
	m_pwm1(PIN_PWM_OUT1),
	m_pwm2(PIN_PWM_OUT2),
	m_encoder_bcu_motor(PIN_ENCODER_A, PIN_ENCODER_B, NC, PULSEPERREV, QEI::X4_ENCODING),
	m_posPid(POS_K_C, POS_TAU_I, POS_TAU_D, PID_FREQ_NOT_USED),
    m_depthPid(DEP_K_C, DEP_TAU_I, DEP_TAU_D, PID_FREQ_NOT_USED),
	m_pressureSensor(PIN_IMU_SDA, PIN_IMU_SCL),
	m_motorServo(PIN_PWM_SERVO)
{};

Buoyancy_Unit::~Buoyancy_Unit(){}

void Buoyancy_Unit::init() {
	m_counter = 0;
	m_currentval = 555;
	m_output = 444;
	m_mode = 0;
	m_kc = 0;
	m_taui = 0;
	m_taud = 0;
	m_setval = 0;
    voltageDefault();

	m_posPid.setMode(1); // AUTO != 0
	m_posPid.setInputLimits(-360, 360); // analog input of position to be scaled 0-100%
	m_posPid.setOutputLimits(-1,1); // PWM output from -1 (CW) to 1 (CCW)

	m_depthPid.setMode(1); // nonzero: AUTO
	m_depthPid.setInputLimits(0.0, MAXDEPTH); // analog input of position to be scaled 0-100%
	m_depthPid.setOutputLimits(-360, 360); // position output from -360 to 360 deg
	m_pressureSensor.MS5837Init();
}


/**
 * Update global variables
 */
void Buoyancy_Unit::update(int M, float A, float B, float C, float D)
{
	if (m_mode != M)
	{
		updateMode(M);
	}
	m_mode = M;
	m_setval = A;
	m_kc = B;
	m_taui = C;
	m_taud = D;
}


void Buoyancy_Unit::updateMode(int mode)
{
	stop();
    switch (mode)
    {
        case VOLTAGE:
        m_pressureSensor.MS5837Reset();
        m_timer.attach(this, &Buoyancy_Unit::voltageControl,0.05);
        break;

        case POSITION:
        m_pressureSensor.MS5837Reset();
		m_timer.attach(this, &Buoyancy_Unit::positionControl,0.05);
        break;

        case DEPTH:
        m_pressureSensor.MS5837Start();
        m_timer.attach(this, &Buoyancy_Unit::depthControl,0.05);
        break;
    }
}


void Buoyancy_Unit::stop()
{
	m_timer.detach();
	m_pwm1 = 0;
	m_pwm2 = 0;
}


void Buoyancy_Unit::printValues()
{
	printf("counter: %d, mode: %d, Kc:%f, TauI:%f, TauD:%f, SetVal: %.2f, CurrentVal: %.2f, duty cycle: %.2f \n",m_counter, m_mode, m_kc, m_taui, m_taud, m_setval, m_currentval, m_output*100);
}


/**
 * This function sets the voltage provided to the motor to a desired voltage
 * by changing the PWM
 * @param duty     Desired pwm duty cycle in range [-1,1]; negative for CW, positive for CCW rotation
 * @param T        (Optional arg) change period
 */
void Buoyancy_Unit::voltageControlHelper(float setDuty)
{
    if (setDuty > 0) //CCW
    {
        m_pwm1 = setDuty;
        m_pwm2 = 0;
    }
    else //CW
    {
        m_pwm1 = 0; // set duty cycle
        m_pwm2 = -setDuty;
    }
}

void Buoyancy_Unit::voltageControl()
{
	voltageControlHelper(m_setval);
}

/** 
 * This function receives a desired position and tries to rotate the motor to that position
 * setDeg is in range [-360, 360] where negative value is CW and positive value is CCW
 * Note: make sure to set the motor to its rest position before starting
 */
void Buoyancy_Unit::positionControl()
{
	m_counter++;
	m_posPid.setTunings(m_kc, m_taui, m_taud);
    m_posPid.setSetPoint(m_setval); // we want the process variable to be the desired value

    // Detect motor position
    float pvPos = m_encoder_bcu_motor.getPulses() % PULSEPERREV;
    float pvDeg = pvPos/PULSEPERREV*360;

    if (m_encoder_bcu_motor.getPulses() > PULSEPERREV) // rotated more than 360deg CCW
    {
    	m_currentval = 360;
    }
    else if (m_encoder_bcu_motor.getPulses() < -PULSEPERREV) // rotated more than 360deg CW
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
    voltageControlHelper(m_output); // change voltage provided to motor
}


/** 
 * This function changes the motor position to a desired value based on
 * the readings on the pressure sensor
 * setDepth is the pressure reading
 */
void Buoyancy_Unit::depthControl()
{
	m_depthPid.setTunings(m_kc, m_taui, m_taud);
    // Run PID
	float setDepth = m_setval;
    m_depthPid.setSetPoint(setDepth); // we want the process variable to be the desired value
    
    // Detect depth
    float pvDepth = m_pressureSensor.MS5837_Pressure();
    
    // Set motor position
    m_depthPid.setProcessValue(pvDepth); // update the process variable
    m_setval = m_depthPid.compute(); // set the new output of position
    //pc.printf("setDepth: %.1f mbar, pvPos: %.1f mbar, setPos: %.2f deg\n",setDepth, pvDepth, SETVAL);
    positionControl(); // change position of motor
}


/**
 * This function sets the PWM to the default values
 * to rotate the motor CCW at half the supplied voltage
 */
void Buoyancy_Unit::voltageDefault()
{
    m_pwm1.period(0.00345); // 3.45ms period
    m_pwm2.period(0.00345); // 3.45ms period
    m_pwm1 = 0;           // duty cycle of 50%
    m_pwm2 = 0;
}
