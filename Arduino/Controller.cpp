#include "Controller.h"

Controller::Controller(){
	computerConnector = new ComputerConnector();
	dynamics = new Dynamics();
	kinematics = new Kinematics();
	dynamixelConnector = new DynamixelConnector();

	_initialize();
}

Controller::~Controller(){
	delete computerConnector;
	delete dynamics;
	delete kinematics;
	delete dynamixelConnector;
} 

void Controller::main(){

	//deltaTime = _UpdateLoopTime();
	//computerConnector->updateComputerData();
	dynamixelConnector->getJointAngles(Degree,AngleData);
	//kinematics->UpdateForwardkinematics();
	//dynamics->UpdateDynamics(Looptime);
}

void Controller::_updateDeltaTime(){
	_newTime = millis();
	deltaTime = (_newTime - _prevTime)/1000;
	_prevTime = _newTime;
}

void Initialize() {
	_updateDeltaTime();

	return;
}

