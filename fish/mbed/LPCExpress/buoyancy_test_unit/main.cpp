#include "mbed.h"
#include "BTU.h"
#include "Ticker.h"

Serial      pc(USBTX, USBRX);
AnalogIn	pot1(p15);
AnalogIn	pot2(p19);
AnalogIn	pot3(p20);

int main()
{
	pc.printf("Start!\n");
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
		Kc = pot2;
		// scale for range in [a,b]
		float a1 = 6.8;
		float b1 = 7.5;
		Kc = (b1-a1)*Kc+a1;

		TauI = pot1;
		float a2 = 0;
		float b2 = 0.0001;
		TauI = (b2-a2)*TauI+a2;

		TauD = pot3;
		float a3 = 0;
		float b3 = 0.0001;
		TauD = (b3-a3)*TauD+a3;

		m_BTU.update(mode, setVal, 0.5,0.1,0.1);
		m_BTU.printGlobal();
	}
	return 0;
}
