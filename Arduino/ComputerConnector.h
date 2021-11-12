#pragma once
#include "String.h"
#include "HardwareSerial.h"

#include "CrustCrawlerData.h"

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

	/*Getters for use in controller
	bool getEmergencyStop() { return _emergencyStop; };
	unsigned int getControlMode() { return _controlMode; };
	bool getDirection() { return _directionSign; };
	double getSpeed() { return _speed; };
	*/



	

private:
	//Variables received from computer 
	bool _emergencyStop;
	uint8_t _controlMode;
	bool _directionSign;
	uint8_t _speed_mm_s;

	//Variables for conversion from max linear speed to max angular speed
	const double _maxJointLength = CrustCrawler::Joint2.m_length + CrustCrawler::Joint3.m_length + CrustCrawler::Joint4.m_length;
	const double _maxLinVelocity = 0.15;
	const double _maxAngVelocity = _maxJointLength / _maxLinVelocity;
	const double _ratioLinToAng = _maxAngVelocity / _maxLinVelocity;

	//Variables for reading data
	char _dataBuffer[4];
	int _incommingData;

	void _ComputerDataToVelocity();

};

