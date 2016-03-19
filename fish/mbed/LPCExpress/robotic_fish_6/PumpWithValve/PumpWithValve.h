#include "mbed.h"
#include "../QEI/QEI.h"

#define valvePin p21
#define pumpPin p23
#define hallPin p20 // actually p33 but compiler is mad at me
#define encoderPinA p25
#define encoderPinB p24
#define valveCurrentPin p19

#define count2rev 12
#define valveMotorGearRatio 297.92
#define freq_PGain 0.05 // random dummy number
#define freq_DGain 0.01 // dummy val
#define valveOffsetGain 0.05

class PumpWithValve
{
    public:
        // Initialization
        PumpWithValve();
        void start();
        void stop();
	void flip();
        void set(float freq_in, float yaw_in, float thrust_in);

    private:

	volatile float frequency;
	volatile float yaw;
	volatile float thrust;

	volatile bool valve_side;
	volatile float valveV1;
	volatile float valveV2;
	volatile float Vfreq;
	volatile float Vthrust;

	volatile float dt;
	volatile float rot;
	volatile float freq_act;
	volatile float freq_error;
	volatile float prev_freq_error;
	volatile float dV_freq;
	
	PwmOut pumpPWM;
	PwmOut valvePWM;
	QEI valveEncoder;
	AnalogIn valveCurrent; // actually is a voltage value proportional to current
	InterruptIn hallSignal;
	Timer timer;
};


