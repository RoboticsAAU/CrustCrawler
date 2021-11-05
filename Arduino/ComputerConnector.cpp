#include "ComputerConnector.h"

ComputerConnector::ComputerConnector() {
	DEBUG_SERIAL.begin(115200);
	//while (!DEBUG_SERIAL.available()) {};
}

ComputerConnector::~ComputerConnector() {

}



int* ComputerConnector::getComputerData() {
	/*emergencyStop = false;
	controlMode = 1;
	positiveDirection = true;
	speed = 200;
	
	dataArray = { emergencyStop, controlMode, positiveDirection, speed };

	int* tmpArray = dataArray;
	return tmpArray;*/
}


