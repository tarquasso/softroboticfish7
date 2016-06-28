#include "mbed.h"
#include "BTU.h"


Serial      pc(USBTX, USBRX);
//AnalogIn    enc1(p16);
//AnalogIn    enc2(p17);
//DigitalOut  myled1(LED1);
//DigitalOut  myled2(LED2);
//BusOut      unused(p15,p18,p19,p20);  // Make unused analog pin to DigitalOut

//QEI wheel (p16, p17, NC, 1786); //add: ,QEI::X4_ENCODING

int main() 
{
    BTU btuobj;
    btuobj.Run(1,0.1);
    return 0;
}