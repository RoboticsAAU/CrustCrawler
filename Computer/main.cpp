#include "MyoBand.h"
#include "Filter.h"
#include "SerialLink.h"

#include "iostream"

int main() {
	// We create a myoband object first
	MyoBand MyoBand;

	// We then create our filter object which depends on the myoband link
	Filtering Filter(100, MyoBand);

	// Then we need to specify the desired com port
	char* comPort = (char*)"COM8";
	// And the baud rate. They prefix with CBR_. So e.x. CBR_9600, CBR_56000, CBR_115200, etc...
	DWORD baudRate = CBR_9600;
	// From this we can now create our serial link
	SerialLink SerialPort(comPort, baudRate, Filter);

	while (true)
	{
#ifdef _DEBUG //Is true when we select "Debug Mode" in VS
		printf("\r");
		MyoBand.print();
		Filter.print();
		SerialPort.print();
		fflush(stdout);
#endif
		if (GetKeyState(VK_ESCAPE) & 0x8000)
		{
			return 0;
		}
		SerialPort.sendData();
	}
}