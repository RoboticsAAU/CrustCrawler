#include "ComputerConnector.h"
#include "HardwareSerial.h"
#include "string.h"

ComputerConnector::ComputerConnector() {
	Serial1.begin(115200);
}

ComputerConnector::~ComputerConnector() {

}

void ComputerConnector::debugPrintLine() {
	Serial1.println(input);
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


