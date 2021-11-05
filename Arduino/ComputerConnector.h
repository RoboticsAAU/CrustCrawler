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
	void updateComputerData();


	//Getters for use in controller
	bool getEmergencyStop() { return this->emergencyStop; };
	unsigned int getControlMode() { return this->controlMode; };
	bool getDirection() { return this->direction; };
	double getSpeed() { return this->speed; };



	

private:
	//Variables received from computer 
	bool emergencyStop;
	unsigned int controlMode;
	//bool positiveDirection;
	bool direction;
	unsigned int speed;
	String _newData;
	String _currentData;



};

