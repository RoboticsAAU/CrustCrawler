//#include "SimpleSerial.h"
#include "serialib.h"
#include "Filter.h"
#include "MyoBand.h"

#include <Windows.h>
#include <string>
#include <iostream>
#include <chrono>


class SerialLink {
public:
	SerialLink(char* comPort, DWORD baudRate, Filtering& FilterObject);

	void sendData();

#ifdef _DEBUG
	void print();
#endif

private:
	// Package functions
	void packageConstructor();
	void getEmergencyStop();
	void getSpeed();
	void getDirection(myo::Pose inputPose);
	void getMode();


	//Vector to hold the package variables as unsigned char. Please note the use of unsigned char which allows integer representation
	//from 0 to 255, as opposed to char which would restrain us to -128 to 127. Unsigned char is thus used as the speed to be sent can
	//have a max value of 150
	std::vector<unsigned char> package;
	
	//Package variables
	unsigned char HeaderByte = 255;
	unsigned char EmergencyStop = 0;
	unsigned char Mode = 0;
	unsigned char Direction = 0;
	unsigned char Speed = 0;

	///__LOGIC__///

	//Variables used to hold both current and previous poses
	myo::Pose currentPose = myo::Pose::unknown;
	myo::Pose previousPose = myo::Pose::unknown;
	myo::Pose lastControlPose = myo::Pose::unknown;
	
	//Enumerate list for the different control modes
	enum ControlMode {
		Grasp,     //0
		LeftRight, //1
		UpDown,    //2
		InOut,	   //3
		Lock	   //4
	};

	//Enumerate list for the different speed modes.
	//Used to define the mapping of moving average to a speed from 0 - 150 mm/s.
	enum SpeedMode {
		Gross = 1,
		Linear = 2,
		Precision = 3
	};

	//Variables used for keeping track of control mode and speed mode. Please note the default modes. 
	ControlMode controlMode = Grasp;
	ControlMode previousControlMode = Grasp;
	SpeedMode speedMode = Linear;

	//Declaration of pointers. These are initialized in the contructor's initialize list
	Filtering* pFilterObject;
	MyoBand* pMyoBand;

	//Declaring pointer to object of type serialib
	serialib* Serial;

	//Declaration of pointer to char array and variable to store baudrate. Both used to establish serial connection by serialib.h
	char* comPort;
	DWORD baudRate;
	std::chrono::time_point<std::chrono::steady_clock> serialDelay = std::chrono::steady_clock::now();

	//Variable used as a condition when sending data to arduino through serialib
	int isSent;

	//Variables to keep track of and managing print comments for the user
	bool calibration = false;
	bool firstCalibration = true;
	std::chrono::time_point<std::chrono::steady_clock> interfaceTimeStamp = std::chrono::steady_clock::now();
	
	//Upper moving average limit for waveOut and waveIn. Initialized with a low value, which shoftly after runtime is overwritten through calibration
	double waveInMaxSpeed = 0;
	double waveOutMaxSpeed = 0;

	//Lower moving average treshhold between rest and waveIn and waveOut pose. Initialized with a high value, which shortly after runtime is overwritten through calibration
	double waveInThreshold = 150;
	double waveOutThreshold = 150;
	
	//Variable used to keep track of previous speed
	double prevSpeed = 0;

	//Constants used limit the max speed and calculate thresholds
	const double maxSpeedCap = 150; 
	const double thresholdTolerance = 0.1; //Tolerance (in %) used for updating the upper/lower thresholds

	//Function used internally to define the threshold that corresponds to a given pose
	double Threshold(myo::Pose gesture);

	//Function used internally to map moving average to speed
	double speedMap(double& variable);
};
