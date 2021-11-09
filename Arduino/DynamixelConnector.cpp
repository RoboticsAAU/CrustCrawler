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

// ---------------------------PUBLIC--------------------------------
//void DynamixelConnector::setPWM(){
//
//}

JointAngles DynamixelConnector::getJointAngles(UnitType unitType) {
	_UpdateDynamixelAngles();
	_AngleConverter(unitType);
	return outputAngles;
}

// ---------------------------PRIVATE--------------------------------

/// <summary>
/// Directly read the servos raw position value. 
/// </summary>
void DynamixelConnector::_UpdateDynamixelAngles() {
	rawDynamixelAngles.m_Theta1 = p_dynamixel->getPresentPosition(1, UNIT_RAW);
	rawDynamixelAngles.m_Theta2 = p_dynamixel->getPresentPosition(2, UNIT_RAW);
	rawDynamixelAngles.m_Theta3 = p_dynamixel->getPresentPosition(3, UNIT_RAW);
	rawDynamixelAngles.m_Theta4 = p_dynamixel->getPresentPosition(4, UNIT_RAW);
	rawDynamixelAngles.m_Theta5 = p_dynamixel->getPresentPosition(5, UNIT_RAW);

	rawDynamixelAngles.currentUnitType = Raw;
}

/// <summary>
/// Convert the internal raw motor angles to a desired unit type.
/// </summary>
/// <param name="desiredUnit"> Convert to this unit type </param>
void DynamixelConnector::_AngleConverter(UnitType desiredUnit) {
	if (desiredUnit == rawDynamixelAngles.currentUnitType) {
		return;
	}

	double conversionConstant;

	switch (desiredUnit) {
	case Degree: {
		if (rawDynamixelAngles.currentUnitType == Radians) {
			conversionConstant = 360 / (2 * M_PI);
			break;
		}
		if (rawDynamixelAngles.currentUnitType == Raw) {
			conversionConstant = 360 / 4095;
			break;
		}
	}
	case Radians: {
		if (rawDynamixelAngles.currentUnitType == Degree) {
			conversionConstant = (2 * M_PI) / 360;
			break;
		}
		if (rawDynamixelAngles.currentUnitType == Raw) {
			conversionConstant = (2 * M_PI) / 4095;
			break;
		}
	}
	case Raw: {
		if (rawDynamixelAngles.currentUnitType == Degree) {
			conversionConstant = 4095 / 360;
			break;
		}
		if (rawDynamixelAngles.currentUnitType == Degree) {
			conversionConstant = 4095 / (2 * M_PI);
			break;
		}
	}
	}
	outputAngles.m_Theta1 = rawDynamixelAngles.m_Theta1 * conversionConstant;
	outputAngles.m_Theta2 = rawDynamixelAngles.m_Theta2 * conversionConstant;
	outputAngles.m_Theta3 = rawDynamixelAngles.m_Theta3 * conversionConstant;
	outputAngles.m_Theta4 = rawDynamixelAngles.m_Theta4 * conversionConstant;
	outputAngles.m_Theta5 = rawDynamixelAngles.m_Theta5 * conversionConstant;

	outputAngles.currentUnitType = desiredUnit;
}

