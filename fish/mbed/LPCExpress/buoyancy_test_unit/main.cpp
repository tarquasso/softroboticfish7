#include "Buoyancy_Unit/BuoyancyUnit.h"
#include "mbed.h"

#include "MODSERIAL.h"
#include "SerialComm.h"

#define NUM_FLOATS 4
MODSERIAL pcSerial(USBTX,USBRX); //serial device
AnalogIn	pot1(p15);
AnalogIn	pot2(p19);
AnalogIn	pot3(p20);

int main()
{
	MODSERIAL* pcSerial = new MODSERIAL(USBTX,USBRX); //dynamic allocation of the serial device
	pcSerial->printf("Start!\n");

	//Set up a serial communication object
    SerialComm serialComm(pcSerial);
    
	//BTU m_BTU single instance is made in .h .cpp files;
	int mode = 2; //default
	float Kc = 0.0005;
	float TauI = 0.00003;
	float TauD = 0.000000000;
	float setVal = 90;

	m_BTU.init();
	
//	while (1)
//	{
//		if(serialComm.checkIfNewMessage()) {
			//serialComm.checkIfNewMessage()=1;
            int valueInteger = serialComm.getInt();

            float valueFloat = serialComm.getFloat();

            pcSerial->printf("Int:%d, Float:%f\n",valueInteger,valueFloat);

            float valueFloats[NUM_FLOATS];

            serialComm.getFloats(valueFloats,NUM_FLOATS);

//            // printing out for debugging
//            for (int i = 0; i < NUM_FLOATS; i++)
//            {
//                pcSerial->printf("Val#%d: %f\n",i,valueFloats[i]);
//            }
            setVal = valueFloats[0];
            Kc = valueFloats[1];
            TauI = valueFloats[2];
            TauD = valueFloats[3];
            wait(1);
//        }

//		Kc = pot1;
//		float a1 = 0; 		// scale for range in [a,b]
//		float b1 = 10;
//		Kc = (b1-a1)*Kc+a1;
//
//		TauI = pot2;
//		float a2 = 0;
//		float b2 = 0.0001;
//		TauI = (b2-a2)*TauI+a2;
//
//		TauD = pot3;
//		float a3 = 0;
//		float b3 = 0.0001;
//		TauD = (b3-a3)*TauD+a3;

    while(1){
		m_BTU.update(mode, setVal, Kc, TauI, TauD);
		m_BTU.printValues();
	}
}
