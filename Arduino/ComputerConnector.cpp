#include "ComputerConnector.h"

ComputerConnector::ComputerConnector() {
	DEBUG_SERIAL.begin(115200);
	DATA_SERIAL.begin(115200);
}

void ComputerConnector::updateComputerData() {
	if (DATA_SERIAL.available()) {

		static int incommingData = (int)DATA_SERIAL.read();
		if (incommingData == 255) {
			DATA_SERIAL.readBytes(_dataBuffer, 4);
		}
	}
}


/*
void ComputerConnector::_ComputerDataToVelocity() {
	Joints[1]->m_vel = 0;
	Joints[2]->m_vel = 0;
	Joints[3]->m_vel = 0;
	Joint4.m_vel = 0;
	Joint5.m_vel = 0;

	int _direction = _directionSign ? 1 : -1;

	double _speed_m_s = (double)_speed_mm_s / 1000.0;

	//Control mode: 0 = gripper, 1 = base, 2 = in/out, 3 = up/down, 4 = stop
	switch (_controlMode) {
		case 0: {
			Joint4.m_vel = -_direction;
			Joint5.m_vel = _direction;

			MotionData.currentSpaceType = JointSpace;
			break;
		}
		case 1: {
			double _speed_rad_s = _speed_m_s * _ratioLinToAng;

			Joints[1]->m_vel = _direction * _speed_rad_s;

			MotionData.currentSpaceType = JointSpace;
			break;
		}
		case 2: {
			Joints[1]->m_vel = _direction * _speed_m_s;

			MotionData.currentSpaceType = CartesianSpace;
			break;
		}
		case 3: {
			Joints[3]->m_vel = _direction * _speed_m_s;

			MotionData.currentSpaceType = CartesianSpace;
			break;
		}
		case 4: {
			break;
		}
	}

}
*/

