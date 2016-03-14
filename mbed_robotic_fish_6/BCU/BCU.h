#include "mbed.h"

#define bcuPin p22
#define bcuCurrentPin p20

#define maxBCUCurrent 0.5 // need to measure
#define minBCUCurrent 0.1 // need to measure

#define depth_PGain 0.5 // random dummy val
#define depth_DGain 0.0 // ignore for now; start with P control

class BCU
{
    public:
        // Initialization
        BCU();
        void start();
        void stop();
        void set(float depth_in);

    private:
	volatile float depth_cmd;
	volatile float depth_act;
	volatile float depth_error;
	volatile float prev_depth_error;
	volatile float Vref;
	volatile float dV_in;
	
	PwmOut bcuPWM;
	AnalogIn bcuCurrent; // actually is a voltage value proportional to current

};


