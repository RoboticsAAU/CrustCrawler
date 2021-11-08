#include "SimpleSerial.h"
#include "Filter.h"
#include "MyoBand.h"

#include <Windows.h>
#include <string>
#include <vector>
#include <iostream>

class SerialLink {
public:
    SerialLink(std::string& comPort, DWORD baudRate, Filtering& FilterObject);

	void sendData();
private:
	std::string packageConstructor();

	MyoBand* pMyoBand;
   
	std::string comPort;
	DWORD baudRate;

	SimpleSerial* Serial;

	bool isSent;
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
