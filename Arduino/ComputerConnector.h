#pragma once
#include "String.h"
#include "HardwareSerial.h"

#define DEBUG_SERIAL Serial1
#define DATA_SERIAL Serial2


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
	bool getEmergencyStop() { return emergencyStop; };
	unsigned int getControlMode() { return controlMode; };
	bool getDirection() { return direction; };
	double getSpeed() { return speed; };



	

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

