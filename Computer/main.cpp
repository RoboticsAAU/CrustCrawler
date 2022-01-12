#include "MyoBand.h"
#include "Filter.h"
#include "SerialLink.h"

#include "iostream"

int main() {
	//We create a myoband object first
	MyoBand MyoBand;

	//We then create our filter object which depends on the myoband link
	Filtering Filter(80, MyoBand);

	//Then we need to specify the desired com port. We use the syntax "\\\\.\\COMx". for expressing 
	//serial ports above COM9, so will be the default here to support all users...
	//Documentation for expression: https://docs.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea?redirectedfrom=MSDN#communications-resources
	char* comPort = (char*)"\\\\.\\COM4";
	
	//And the baud rate. They prefix with CBR_. So e.x. CBR_9600, CBR_56000, CBR_115200, etc...
	DWORD baudRate = CBR_57600;
	
	//From this we can now create our serial link
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
