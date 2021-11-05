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

		this->_newData = DATA_SERIAL.readStringUntil('\n');
		if (this->_newData == this->_currentData && this->_newData != "00000") {
			return;
		}
		this->_currentData = this->_newData;

		String tmpSpeedString = _newData.substring(3);
		int arraySize = _newData.length();

		
		this->emergencyStop = this->_currentData[0] - '0';
		this->controlMode = this->_currentData[1] - '0';
		this->direction = this->_currentData[2] - '0';
		this->speed = tmpSpeedString.toDouble();

	}

  }


	/*emergencyStop = false;
	controlMode = 1;
	positiveDirection = true;
	speed = 200;
	
	dataArray = { emergencyStop, controlMode, positiveDirection, speed };

	int* tmpArray = dataArray;
	return tmpArray;*/


