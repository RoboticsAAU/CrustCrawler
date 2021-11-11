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
	void packageConstructor();
	void getEmergencyStop(char& outStop);
	void getSpeed(char& outSpeed);
	void getDirection(char& outDirection);
	void getMode(char& outMode);

	std::string package;
	
	char HeaderByte = 255;
	char EmergencyStop = 0;
	char Mode = 0;
	char Direction = 0;
	char Speed = 0;
	myo::Pose previousPose;

	enum eMode {
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

	eMode currentMode = Grasp;
	SpeedMode speedMode = Linear;
	

	Filtering* pFilterObject;
	MyoBand* pMyoBand;
	
	char* comPort;
	DWORD baudRate;

	serialib* Serial;
	int isSent;



	int currentMaxSpeed = 0;
	int threshold = 0;
	int speedCap = 150;

	int speedMap(int& variable);

	void gain(int& variable);
};
