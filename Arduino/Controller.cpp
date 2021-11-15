#include "Controller.h"


// constructor for the Controller class. Here obejcts of the smaller classes used for
// comunication, calculations ect. will be initialized. 
Controller::Controller(){
	computerConnector = new ComputerConnector();
	dynamics = new Dynamics();
	kinematics = new Kinematics();
	dynamixelConnector = new DynamixelConnector();

	_initialize();
}


// Destructor for the Controller class
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


// Function used in main to log how long time has passed since last itteration.
// the function uses the arduino function milis() to get the time past since boot
// in miliseconds. milis() return an unsigned long, therefore the time data will be
// stored as such.
void Controller::_updateDeltaTime(){
	_newTime = millis();
	deltaTime = (_newTime - _prevTime)/1000;
	_prevTime = _newTime;
}

// The initialize() fuction is run as par of the setup() code. The main purpose of
// the function is to update the data of the CrustCrawler, such as it current position,
// state, etc.

void Initialize() {
	_updateDeltaTime();


