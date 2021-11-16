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

	// Package variables
	std::vector<unsigned char> package;
	//std::string package;
	unsigned char HeaderByte = 255;
	unsigned char EmergencyStop = 0;
	unsigned char Mode = 0;
	unsigned char Direction = 0;
	unsigned char Speed = 0;

	// Logic
	myo::Pose currentPose = myo::Pose::unknown;
	myo::Pose previousPose = myo::Pose::unknown;
	myo::Pose lastControlPose = myo::Pose::unknown;
	double Threshold(myo::Pose gesture);

	enum ControlMode {
		Grasp,     //0
		LeftRight, //1
		UpDown,    //2
		InOut,	   //3
		Lock //4
	};

	enum SpeedMode {
		Gross,
		Linear,
		Precision
	};

	ControlMode controlMode = Grasp;
	ControlMode previousControlMode = Grasp;
	SpeedMode speedMode = Linear;


	Filtering* pFilterObject;
	MyoBand* pMyoBand;

	char* comPort;
	DWORD baudRate;

	serialib* Serial;
	int isSent;
	bool calibration = false;
	bool firstCalibration = true;

	std::chrono::time_point<std::chrono::steady_clock> interfaceTimeStamp = std::chrono::steady_clock::now();
	std::chrono::time_point<std::chrono::steady_clock> modeTimeStamp = std::chrono::steady_clock::now();

	//Upper moving average limit for waveOut and waveIn. Initialized with a low value, which shoftly after runtime is overwritten through calibration
	double waveInMaxSpeed = 0;
	double waveOutMaxSpeed = 0;
	//Lower moving average treshhold between rest and waveIn and waveOut pose. Initialized with a high value, which shortly after runtime is overwritten through calibration
	double waveInThreshold = 150;
	double waveOutThreshold = 150;
	
	double prevSpeed = 0;
	double maxSpeedCap = 150; 

	double thresholdTolerance = 0.1; //Tolerance (in %) used for updating the upper/lower thresholds

	double speedMap(double& variable);

};
