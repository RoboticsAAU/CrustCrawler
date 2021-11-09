#include "MyoBand.h"
#include "Filter.h"
#include "SerialLink.h"

#include "iostream"
#include <string>

int main() {
	// We create a myoband object first
	MyoBand MyoBand;

	// We then create our filter object which depends on the myoband link
	Filtering Filter(100, MyoBand);

	// Then we need to specify the desired com port
	char* comPort = (char*)"\\\\.\\COM6";
	// And the baud rate. They prefix with CBR_. So e.x. CBR_9600, CBR_56000, CBR_115200, etc...
	DWORD baudRate = CBR_9600;
	// From this we can now create our serial link
	SerialLink SerialPort(comPort, baudRate, Filter);


	while (true)
	{
		#ifdef _DEBUG //Is true when we select "Debug Mode" in VS
		//std::string test = "Hello";
		//MyoBand.print();
		//std::cout << "Moving average: [" << Filter.MoveAvg() << "]" << std::flush;
		#endif

		SerialPort.sendData();
	}

	return 0;
}