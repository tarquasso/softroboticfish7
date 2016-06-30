/*
 * Author: Robert Katzschmann
 */

#include "SerialComm.h"

// Initialization
SerialComm::SerialComm(MODSERIAL* serialObject /* = NULL */):
    messageProcessed(false),
    byte_idx(0),
    val_idx(0)
{
    this->init(serialObject);
}

void SerialComm::init(MODSERIAL* serialObject /* = NULL */)
{
    // Create serial object or use provided one
    if(serialObject == NULL) {
        serialObject = new MODSERIAL(SERIAL_DEFAULT_TX, SERIAL_DEFAULT_RX);
        serialObject->baud(SERIAL_DEFAULT_BAUD);
    }
    serial = serialObject;
    serial->baud(9600);
    serial->attach(this, &SerialComm::rxCallback, MODSERIAL::RxIrq);
}


void SerialComm::rxCallback(MODSERIAL_IRQ_INFO *q)
{
    MODSERIAL *serial = q->serial;
    if ( serial->rxGetLastChar() == '\n') {

        //  // reset byte_idx
        byte_idx = 0;
        val_idx = 0;
        while(serial->readable()) {

            input = serial->getc(); //temporarily  write to a char
            if (input == ' ') {
                if(byte_idx > 0) {
                    value[val_idx][byte_idx]='\x0';
                    val_idx++;
                    byte_idx = 0;
                }
            } else if (byte_idx < BUFFERSIZE) {
                // just to avoid buffer overflow
                value[val_idx][byte_idx++] = input;  // put it into the value array and increment the byte_idx
            }

        }

        value[val_idx][byte_idx-1]='\x0';  // remove newline by writing to byte_idx-1 and add a \x0 to end the c string
        //value[byte_idx]='\x0';  // add an 0 to end the c string, after the newline character

        messageProcessed = true;
    }

}

float SerialComm::getFloat()
{
    //Conversion From Character Array to Float
    valueFloats[0] =atof(value[0]);

    messageProcessed = false;
    return valueFloats[0];
}

void SerialComm::getFloats(float* floats, int howMany)
{
    if(howMany<NUMBER_COUNT_MAX)
        for(int i = 0; i < howMany; i++) {
            if(i < howMany) {
                valueFloats[i] = atof(value[i]);
                floats[i] = valueFloats[i];
            } else {
                floats[i] = 0.0;
            }
        }
}

int SerialComm::getInt()
{
    // Conversion From Character Array to Integer
    valueInts[0] = atoi(value[0]);

    messageProcessed = false;
    return valueInts[0];
}

bool SerialComm::checkIfNewMessage()
{
    return messageProcessed;

}

//int SerialComm::getString(char* string)
//{
//    return valueInts;
//}
