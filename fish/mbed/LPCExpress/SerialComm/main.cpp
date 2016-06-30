#include "mbed.h"
#include "MODSERIAL.h"
#include "SerialComm.h"

#define NUM_FLOATS 5
int main()
{
    MODSERIAL* pcSerial = new MODSERIAL(USBTX,USBRX); //dynamic allocation of the serial device

    pcSerial->printf("Alive!\n");

    //Set up a serial communication object
    SerialComm serialComm(pcSerial);


    while(1) {
        if(serialComm.checkIfNewMessage()) {

            int valueInteger = serialComm.getInt();

            float valueFloat = serialComm.getFloat();

            pcSerial->printf("Int:%d, Float:%f\n",valueInteger,valueFloat);
            
            
            float valueFloats[NUM_FLOATS];
            
            serialComm.getFloats(valueFloats,NUM_FLOATS);
            
            for (int i = 0; i < NUM_FLOATS; i++)
            {
                pcSerial->printf("Val#%d: %f\n",i,valueFloats[i]);    
            }
        }
    }

}



