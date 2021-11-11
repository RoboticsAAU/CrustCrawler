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
	void getEmergencyStop(unsigned char& outStop);
	void getSpeed(unsigned char& outSpeed);
	void getDirection(unsigned char& outDirection);
	void getMode(unsigned char& outMode);
	
	// Package variables
	std::vector<unsigned char> package;
	//std::string package;
	unsigned char HeaderByte = 255;
	unsigned char EmergencyStop = 0;
	unsigned char Mode = 0;
	unsigned char Direction = 0;
	unsigned char Speed = 0;

	// Logic
	myo::Pose previousPose;

	enum ControlMode {
		Grasp,
		LeftRight,
		UpDown,
		InOut,
		LockUnlock
	};

	enum SpeedMode {
		Gross,
		Linear,
		Precision
	};

	ControlMode controlMode = Grasp;
	SpeedMode speedMode = Linear;
	

	Filtering* pFilterObject;
	MyoBand* pMyoBand;
	
	char* comPort;
	DWORD baudRate;

	serialib* Serial;
	int isSent;

	double waveInMaxSpeed = 1;
	double waveOutMaxSpeed = 1;
	double waveInThreshold = 0;
	double waveOutThreshold = 0;

	double threshold = 4;
	double speedCap = 150;

	double speedMap(double& variable);
	void configure();
};
