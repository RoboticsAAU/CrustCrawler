#include "SimpleSerial.h"
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
	
	char EmergencyStop = 0;
	char Mode = 0;
	char Direction = 0;
	char Speed = 0;
	char EndByte = 0xFF;
	myo::Pose previousPose;

	enum eMode {
		Grasp,
		LeftRight,
		UpDown,
		InOut
		
	};

	eMode currentMode = Grasp;

	Filtering* pFilterObject;
	MyoBand* pMyoBand;
	
	char* comPort;
	DWORD baudRate;

	SimpleSerial* Serial;

	bool isSent;

	std::chrono::time_point<std::chrono::system_clock> timeStamp = std::chrono::system_clock::now();



};


/*
#pragma once

#include <Windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <string.h>
#include <chrono>
#include <thread>
#include <time.h>
#include <fstream>

using namespace std;

class SerialLink
{

private:
	HANDLE io_handler_;
	COMSTAT status_;
	DWORD errors_;

	string syntax_name_;
	char front_delimiter_;
	char end_delimiter_;

	void CustomSyntax(string syntax_type);	

public:
	SerialLink(char* com_port, DWORD COM_BAUD_RATE);

	string ReadSerialPort(int reply_wait_time, string syntax_type);	
	bool WriteSerialPort(char *data_sent);
	bool CloseSerialPort();
	~SerialLink();
	bool connected_;
};
*/
