#include "Buoyancy_Unit/BuoyancyUnit.h"
#include "mbed.h"

#include "MODSERIAL.h"
#include "SerialComm.h"


//MODSERIAL pcSerial(USBTX,USBRX); //serial device

#define NUM_FLOATS 5

AnalogIn	pot1(p15);
AnalogIn	pot2(p19);
AnalogIn	pot3(p20);

int main()
{
	MODSERIAL* pcSerial = new MODSERIAL(USBTX,USBRX); //dynamic allocation of the serial device
		
	pcSerial->printf("Start!\n");
	
	//Set up a serial communication object
    SerialComm serialComm(pcSerial);
    
	//BTU m_BTU instance is made in .h .cpp files;
	int mode = 2;
	//float input;
	float Kc = 0.0005;
	float TauI = 0.00003;
	float TauD = 0.000000000;
	float setVal = 90;

	m_BTU.init();

/*
	pc.printf("Enter Mode");
	pc.scanf("%d", &mode);
	pc.printf("You input: %d", mode);

	while(1)
	{
		pc.printf("Enter desired position in degrees, 1000+Kc for Kc value, 2000+TauI for TauI value, 3000+TauD for TauD value, or -1000 to stop motor\n");
		pc.scanf("%f", &input);
		pc.printf("You input: %f", input);
		if (input == -1000)
		{
			m_BTU.stop();
		}
		else if (input > 1000 && input < 2000)
		{
			Kc = input - 1000;
		}
		else if (input > 2000 && input < 3000)
		{
			TauI = input - 2000;
		}
		else if (input > 3000 && input < 4000)
		{
			TauD = input - 3000;
		}
		else
		{
			setVal = input; //desired position of motor, in degrees
		}*/
	



	while (1)
	{
		if(serialComm.checkIfNewMessage()) {

            //int valueInteger = serialComm.getInt();

            //float valueFloat = serialComm.getFloat();

            //pcSerial->printf("Int:%d, Float:%f\n",valueInteger,valueFloat);
            
            float valueFloats[NUM_FLOATS];
            
            serialComm.getFloats(valueFloats,NUM_FLOATS);
            
            // printing out for debugging
            for (int i = 0; i < NUM_FLOATS; i++)
            {
                pcSerial->printf("Val#%d: %f\n",i,valueFloats[i]);    
            }
        }
        
		Kc = pot2;
		// scale for range in [a,b]
		float a1 = 0;
		float b1 = 10;
		Kc = (b1-a1)*Kc+a1;

		TauI = pot1;
		float a2 = 0;
		float b2 = 0.0001;
		TauI = (b2-a2)*TauI+a2;

		TauD = pot3;
		float a3 = 0;
		float b3 = 0.0001;
		TauD = (b3-a3)*TauD+a3;

		m_BTU.update(mode, setVal, Kc, TauI, TauD);
		//m_BTU.printGlobal();
	}
}
