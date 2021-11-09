#include "ComputerConnector.h"

ComputerConnector::ComputerConnector() {
	DEBUG_SERIAL.begin(115200);
	DATA_SERIAL.begin(115200);

	//while (!DEBUG_SERIAL.available()) {};
}

ComputerConnector::~ComputerConnector() {
}



void ComputerConnector::updateComputerData() {
	if (DATA_SERIAL.available()) {

		_newData = DATA_SERIAL.readStringUntil('\n');
		if (_newData == _currentData && _newData != "00000") {
			return;
		}
		_currentData = _newData;

		String tmpSpeedString = _newData.substring(3);
		int arraySize = _newData.length();

		
		emergencyStop = _currentData[0] - '0';
		controlMode = _currentData[1] - '0';
		direction = _currentData[2] - '0';
		speed = tmpSpeedString.toDouble();

	}

  }


	/*emergencyStop = false;
	controlMode = 1;
	positiveDirection = true;
	speed = 200;
	
	dataArray = { emergencyStop, controlMode, positiveDirection, speed };

	int* tmpArray = dataArray;
	return tmpArray;*/


