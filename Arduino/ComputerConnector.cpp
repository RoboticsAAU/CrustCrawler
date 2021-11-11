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
		_speed_mm_s = (uint8_t)_dataBuffer[3];

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
	Joint1.m_vel = 0;
	Joint2.m_vel = 0;
	Joint3.m_vel = 0;
	Joint4.m_vel = 0;
	Joint5.m_vel = 0;

	int _direction = _directionSign ? 1 : -1;


	//Control mode: 1 = base, 2 = in/out, 3 = up/down
	switch (_controlMode) {
		case 0: {
			int fingerSpeed = 20; //Remember to change fittingly
			Joint4.m_vel = -_direction * fingerSpeed;
			Joint5.m_vel = _direction * fingerSpeed;

			MotionData.currentSpaceType = JointSpace;
			break;
		}
		case 1: {
			Joint1.m_vel = _direction * _speed_mm_s;

			MotionData.currentSpaceType = JointSpace;
			break;
		}
		case 2: {
			Joint1.m_vel = _direction * _speed_mm_s;

			MotionData.currentSpaceType = CartesianSpace;
			break;
		}
		case 3: {
			Joint3.m_vel = _direction * _speed_mm_s;

			MotionData.currentSpaceType = CartesianSpace;
			break;
		}
		case 4: {
			break;
		}
	}

}




