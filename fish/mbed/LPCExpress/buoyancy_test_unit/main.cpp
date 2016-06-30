#include "mbed.h"
#include "BTU.h"
#include "Ticker.h"

Serial      pc(USBTX, USBRX);

int main()
{
	pc.printf("Start!\n");
	//BTU m_BTU instance is made in .h .cpp files;
	int mode;
	float input;
	float Kc = 0;
	float TauI = 0;
	float TauD = 0;
	float setVal = 0;

	while(1)
	{
		pc.printf("Enter Mode");
		pc.scanf("%d", &mode);

		pc.printf("Enter Desired position in degrees, 1000+Kc for Kc value, 2000+TauI for TauI value, 3000+TauD for TauD value, or -1000 to stop motor\n");
		pc.scanf("%f", &input);
		// DO MODE AND EVERYTHING
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
		}

		pc.printf("setDeg: %.3f deg, Kc:%.3f, TauI:%.3f, TauD:%.3f \n", mode, setVal, Kc, TauI, TauD);
		m_BTU.updateParam(mode, setVal, Kc, TauI, TauD);
		m_BTU.start();
	}
	return 0;
}
