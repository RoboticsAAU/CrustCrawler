#pragma once
// Arduino includes
#include "HardwareSerial.h"
#include "String.h"

// Library includes

// Custom includes
#include "CommonData.h"

// The computer connector can update the current data buffer with new instructions and makes them available to other classes
class ComputerConnector
{
public:
	// Starts the serial ports
	ComputerConnector();

	// Get the data from the computer
	void updateComputerData();

	// Getter functions
	bool getEmergencyStop() { return (bool)_dataBuffer[0]; };
	uint8_t getControlMode() { return (uint8_t)_dataBuffer[1]; };
	bool getDirection() { return (bool)_dataBuffer[2]; };
	uint8_t getSpeed() { return (uint8_t)_dataBuffer[3]; };

private:
	// The data buffer contains data in the following form:
	// EMERGENCY STOP | CONTROL MODE | DIRECTION | SPEED 
	char _dataBuffer[4];
};

