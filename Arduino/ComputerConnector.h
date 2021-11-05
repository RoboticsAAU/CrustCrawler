#pragma once
#include "String.h"
#include "HardwareSerial.h"

#define DYNAMIXEL_SERIAL Serial
#define DEBUG_SERIAL Serial1
#define DATA_SERIAL Serial2
//const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN


class ComputerConnector
{

public:
	ComputerConnector();
	~ComputerConnector();

	//Print anything to the debug serial monitor with a line ending.
	template<typename T> 
	void debugPrintLine(T data) {
		DEBUG_SERIAL.println(data);
	}
	
	//Print anything to the debug serial monitor 
	template<typename T> 
	void debugPrint(T data) {
		DEBUG_SERIAL.print(data);
	}
	//Get the data from the computer
	int* getComputerData();


	

private:
	//Variables received from computer 
	bool emergencyStop;
	unsigned int controlMode;
	bool positiveDirection;
	unsigned int speed;


};

