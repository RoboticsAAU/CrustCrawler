#include "MyoBand.h"
#include "Filter.h"
#include "iostream"

int main() {
	MyoBand MyoBand;
	Filtering Filter(100,MyoBand);

	while (true)
	{
		MyoBand.print();

		#ifdef _DEBUG //Is true when we select "Debug Mode" in VS
		std::cout << "Moving average: [" << Filter.MoveAvg() << "]" << std::flush;
		#endif
	}

	return 0;
}