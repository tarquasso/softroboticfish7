/*
 * SerialComm.h
 *
 * Author: Robert Katzschmann
 */

#ifndef SERIALCONTROL_SerialComm_H_
#define SERIALCONTROL_SerialComm_H_

#include "mbed.h"
#include "MODSERIAL.h"

#define SERIAL_DEFAULT_BAUD 9600
#define SERIAL_DEFAULT_TX USBTX
#define SERIAL_DEFAULT_RX USBRX
#define BUFFERSIZE 100
#define NUMBER_COUNT_DEF 5
#define NUMBER_COUNT_MAX 20

class SerialComm
{
public:
    // Initialization
    SerialComm(MODSERIAL* serialObject = NULL); // if objects are null, ones will be created
    void init(MODSERIAL* serialObject = NULL); // if serial objects are null, ones will be created
    // Execution control
    float getFloat(); //returns the first float
    void getFloats(float* floats, int howMany = NUMBER_COUNT_DEF);
    
    int getInt(); //returns the first int
    void getString();
    void run();
    bool checkIfNewMessage();

protected:
    void rxCallback(MODSERIAL_IRQ_INFO *q);

private:

    bool messageProcessed;
    int byte_idx;
    int val_idx;
    int valueInts[NUMBER_COUNT_MAX];
    float valueFloats[NUMBER_COUNT_MAX];
    char input;
    char value[NUMBER_COUNT_MAX][BUFFERSIZE];
    MODSERIAL* serial;

};

#endif /* SERIALCONTROL_SerialComm_H_ */