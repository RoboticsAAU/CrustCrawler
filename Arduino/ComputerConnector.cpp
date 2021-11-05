#include "ComputerConnector.h"
#include "HardwareSerial.h"

ComputerConnector::ComputerConnector() {
	Serial1.begin(115200);
	while (!Serial1.available()) {};
}

ComputerConnector::~ComputerConnector() {

}

void ComputerConnector::debugPrintLine() {
	Serial1.println();
}

int* ComputerConnector::getComputerData() {
	emergencyStop = false;
	controlMode = 1;
	positiveDirection = true;
	speed = 200;

	dataArray = { emergencyStop, controlMode, positiveDirection, speed };

	int* tmpArray = dataArray;
	return tmpArray;
}


