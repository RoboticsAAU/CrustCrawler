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

		_incommingData = (int)DATA_SERIAL.read();
		if (_incommingData == 255) {
			DATA_SERIAL.readBytes(_dataBuffer, 4);
		}

		//_newData = DATA_SERIAL.readStringUntil('\n');
		//if (_newData == _currentData && _newData != "00000") {
		//	return;
		//}
		//_currentData = _newData;

		//String tmpSpeedString = _newData.substring(3);
		//int arraySize = _newData.length();
				
		_emergencyStop = (bool)_dataBuffer[0];
		_controlMode = (uint8_t)_dataBuffer[1];
		_directionSign = (bool)_dataBuffer[2];
		_speed = (uint8_t)_dataBuffer[3];

		_ComputerDataToVelocity();

	}

  }


	/*emergencyStop = false;
	controlMode = 1;
	positiveDirection = true;
	speed = 200;
	
	dataArray = { emergencyStop, controlMode, positiveDirection, speed };

	int* tmpArray = dataArray;
	return tmpArray;*/




void ComputerConnector::_ComputerDataToVelocity() {
	if (_emergencyStop) {
		MotionData.m_Vel1 = 0;
		MotionData.m_Vel2 = 0;
		MotionData.m_Vel3 = 0;
		return;
	}

	int _direction = _directionSign ? 1 : -1;


	//Control mode: 1 = base, 2 = in/out, 3 = up/down
	switch (_controlMode){
	case 1:{
		MotionData.m_Vel1 = _direction*_speed;
		MotionData.m_Vel2 = 0;
		MotionData.m_Vel3 = 0;

		MotionData.currentSpaceType = JointSpace;
		break;
	}
	case 2:{
		MotionData.m_Vel1 = _direction*_speed;
		MotionData.m_Vel2 = 0;
		MotionData.m_Vel3 = 0;

		MotionData.currentSpaceType = CartesianSpace;
		break;
	}
	case 3: {
		MotionData.m_Vel1 = 0;
		MotionData.m_Vel2 = 0;
		MotionData.m_Vel3 = _direction*_speed;

		MotionData.currentSpaceType = CartesianSpace;
		break;
	}
	}
}
