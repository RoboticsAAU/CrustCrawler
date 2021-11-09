//Include header files.
#include "DynamixelConnector.h"

//Include dynamixel libraries
#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

#define DYNAMIXEL_SERIAL Serial
const uint8_t DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
const uint16_t DYNAMIXEL_BAUDRATE = 57600;

/// <summary>
/// Connect to the dynamixel servos on the crust crawler arm. 
/// </summary>
DynamixelConnector::DynamixelConnector() {
	p_dynamixel = new Dynamixel2Arduino(DYNAMIXEL_SERIAL,DIRECTION_PIN);
	p_dynamixel->begin(DYNAMIXEL_BAUDRATE);
}


/// <summary>
/// Disable the dynamixel connection.
/// </summary>
DynamixelConnector::~DynamixelConnector() {
	delete p_dynamixel;
}
