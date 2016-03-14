
#include "Valve.h"

// The static instance
Valve valve;
void flipStatic()
{
	valve.flip();
}

//============================================
// Initialization
//============================================

// Constructor
Valve::Valve() :
    // Initialize variables
    pumpPWM(pumpPin),
    valvePWM(valvePin),
    valveCurrent(valveCurrentPin),
    valveEncoder(encoderPinA, encoderPinB, NC, count2rev), // use X2 encoding by default
    hallSignal(hallPin)
{
    hallSignal.rise(&flipStatic);

    frequency = 0;
    thrust = 0;
    yaw = 0;

    Vthrust = 0;

    valve_side = 0;
    valveV1 = 0;
    valveV2 = 0;
    Vfreq = 0;
    dV_freq = 0;
    freq_error = 0;
    prev_freq_error = 0;
}


void Valve::start()
{
    valvePWM.write(0.0); // apply nothing to start
    pumpPWM.write(0.0);
    timer.start();
}

void Valve::stop()
{
    valvePWM.write(0.0);
    pumpPWM.write(0.0);
}

void Valve::flip()
{
    // when the hall sensor sees a rising edge, we have rotated 180 degrees
    // --> want to adjust the applied voltage based on the side we are on
    valve_side = !valve_side;
}

//============================================
// Processing
//============================================
void Valve::set(float freq_in, float yaw_in, float thrust_in)
{
    Vthrust = thrust_in; // not sure if more conversions are required
    pumpPWM.write(Vthrust);

    // measure current tail frequency
    dt = timer.read_us();
    rot = valveEncoder.getPulses()/count2rev; // TODO also need to divide by gear ratio if encoder is attached directly to motor
    freq_act = rot/dt;
    freq_error = frequency - freq_act;

    // update tail frequency
    dV_freq = freq_PGain * freq_error + freq_DGain * (freq_error - prev_freq_error);
    Vfreq += dV_freq;

    // split tail frequency voltage into voltage on either side of the valve
    // TODO figure out ideal relationship between yaw and offset between V1 and V2
    // is it additive or multiplicative? start with super simple math
    
    valveV1 = Vfreq + valveOffsetGain * yaw;
    valveV2 = Vfreq - valveOffsetGain * yaw;


    // TODO need to decide whether to give up frequency or yaw when we are at input limits
    if (valveV1 > 1.0) {valveV1 = 1.0;}
    else if (valveV1 < 0.0) {valveV1 = 0.0;}

    if (valveV2 > 1.0) {valveV2 = 1.0;}
    else if (valveV2 < 0.0) {valveV1 = 0.0;}


    // write valve voltage based on which side the hall sensor says we are on
    if (valve_side) {valvePWM.write(valveV1);} // directions are slightly arbitrary at the moment
    else {valvePWM.write(valveV2);}

    prev_freq_error = freq_error;
    valveEncoder.reset();
    timer.reset();
}


