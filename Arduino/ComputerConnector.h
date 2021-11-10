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
	bool getEmergencyStop() { return _emergencyStop; };
	unsigned int getControlMode() { return _controlMode; };
	bool getDirection() { return _direction; };
	double getSpeed() { return _speed; };



	

private:
	//Variables received from computer 
	bool _emergencyStop;
	uint8_t _controlMode;
	//bool positiveDirection;
	bool _direction;
	uint8_t _speed;
	//String _newData;
	//String _currentData;

	char _dataBuffer[4];
	int _incommingData;


};

