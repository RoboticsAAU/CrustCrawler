#include "MyoBand.h"
#include "Filter.h"
#include "iostream"
#include "SerialLink.h"
#include <cstring>

int main() {
	MyoBand MyoBand;
	Filtering Filter(100,MyoBand);
	std::string comport = "\\\\.\\COM11";
	SerialLink SerialTest(comport, (DWORD)CBR_9600);

	string test = "Hello";
	char com_port[] = "\\\\.\\COM11";
    DWORD COM_BAUD_RATE = CBR_9600;

	while (true)
	{
		MyoBand.print();

		#ifdef _DEBUG //Is true when we select "Debug Mode" in VS
		std::cout << "Moving average: [" << Filter.MoveAvg() << "]" << std::flush;
		#endif

		SerialTest.sendData(test);
	}

	return 0;
}