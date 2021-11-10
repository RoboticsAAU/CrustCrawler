
//Include header files.
#include "DynamixelConnector.h"

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

/// <summary>
/// Sets the pwm of the specified servo.
/// </summary>
/// <param name="id"> The id of the servo</param>
/// <param name="pwmData"> The pwm value to set the servo</param>
void DynamixelConnector::setPWM(uint8_t id,uint16_t pwmData){

	if (pwmData > _joints[id - 1].m_PWMlimit) {
		//throw "The pwm signal is above the PWM limit!";
		return;
	}
	p_dynamixel->setGoalPWM(id, pwmData);
}

/// <summary>
/// Returns the pwm of the specified servo
/// </summary>
/// <param name="id"> ID of the servo </param>
/// <returns> PWM in the range of: -PWM_limit ~ PWM_limit </returns>
float DynamixelConnector::getCurrentPWM(uint8_t id) {
	return p_dynamixel->getPresentPWM(id);
}

/// <summary>
/// Gets the present veloctiy of the servo.
/// </summary>
/// <param name="id"> ID of the servo </param>
/// <returns> Velocity in the range of: -VEL_limit ~ VEL_limit </returns>
float DynamixelConnector::getVelocity(uint8_t id) {
	return p_dynamixel->getPresentVelocity(id);
}

/// <summary>
/// Whether the joint is in angular motion or not. Uses the Moving threshold.
/// </summary>
/// <param name="id"> ID of the servo </param>
/// <returns> True if moving, False if not </returns>
bool DynamixelConnector::isJointMoving(uint8_t id) {
	return p_dynamixel->readControlTableItem(ControlTableItem::MOVING, id);
}

/// <summary>
/// Gets the current joint angles in the specified unit.
/// </summary>
/// <param name="unitType"> The desired unit in which the angles will be returned </param>
/// <returns> Joint angles in desired unit </returns>
JointAngles DynamixelConnector::getJointAngles(UnitType unitType) {
	_UpdateDynamixelAngles();

	AngleConverter(_internalJointAngles,unitType);

	return _internalJointAngles;
}

/// <summary>
/// Convert joint angles from the current joint angles to another.
/// </summary>
/// <param name="desiredUnit"> Convert to this unit type </param>
/// <param name="inputAngles"> Reference to the joint angles which is to be converted </param>
void DynamixelConnector::AngleConverter(JointAngles &inputAngles, UnitType desiredUnit) {
	if (desiredUnit == inputAngles.currentUnitType) {
		return;
	}

	double conversionConstant;

	switch (desiredUnit) {
		case Degree: {
			if (inputAngles.currentUnitType == Radians) {
				conversionConstant = 360 / (2 * M_PI);
				break;
			}
			if (inputAngles.currentUnitType == Raw) {
				conversionConstant = 360 / 4095;
				break;
			}
		}
		case Radians: {
			if (inputAngles.currentUnitType == Degree) {
				conversionConstant = (2 * M_PI) / 360;
				break;
			}
			if (inputAngles.currentUnitType == Raw) {
				conversionConstant = (2 * M_PI) / 4095;
				break;
			}
		}
		case Raw: {
			if (inputAngles.currentUnitType == Degree) {
				conversionConstant = 4095 / 360;
				break;
			}
			if (inputAngles.currentUnitType == Degree) {
				conversionConstant = 4095 / (2 * M_PI);
				break;
			}
		}
	}
	inputAngles.m_Theta1 = inputAngles.m_Theta1 * conversionConstant;
	inputAngles.m_Theta2 = inputAngles.m_Theta2 * conversionConstant;
	inputAngles.m_Theta3 = inputAngles.m_Theta3 * conversionConstant;
	inputAngles.m_Theta4 = inputAngles.m_Theta4 * conversionConstant;
	inputAngles.m_Theta5 = inputAngles.m_Theta5 * conversionConstant;

	inputAngles.currentUnitType = desiredUnit;
}

// ---------------------------PRIVATE--------------------------------

/// <summary>
/// Directly read the servos raw position value. 
/// </summary>
void DynamixelConnector::_UpdateDynamixelAngles() {
	_internalJointAngles.m_Theta1 = p_dynamixel->getPresentPosition(1, UNIT_RAW);
	_internalJointAngles.m_Theta2 = p_dynamixel->getPresentPosition(2, UNIT_RAW);
	_internalJointAngles.m_Theta3 = p_dynamixel->getPresentPosition(3, UNIT_RAW);
	_internalJointAngles.m_Theta4 = p_dynamixel->getPresentPosition(4, UNIT_RAW);
	_internalJointAngles.m_Theta5 = p_dynamixel->getPresentPosition(5, UNIT_RAW);

	_internalJointAngles.currentUnitType = Raw;
}

/// <summary>
/// Write to the controll table of each dynamixel, in order to specify its proporties.
/// </summary>
void DynamixelConnector::_SetupDynamixelServos() {

	for (uint8_t i = 0; i < 5; i++)
	{	
		p_dynamixel->torqueOff(_joints[i].m_id);
		p_dynamixel->writeControlTableItem(ControlTableItem::OPERATING_MODE, _joints[i].m_id, OperatingMode::OP_PWM);
		p_dynamixel->writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, _joints[i].m_id, _joints[i].m_maxTheta); 
		p_dynamixel->writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, _joints[i].m_id, _joints[i].m_maxTheta);
		p_dynamixel->writeControlTableItem(ControlTableItem::PWM_LIMIT, _joints[i].m_id, _joints[i].m_PWMlimit);
		p_dynamixel->writeControlTableItem(ControlTableItem::MOVING_THRESHOLD, _joints[i].m_id, _MovingThreshold);
		p_dynamixel->torqueOn(_joints[i].m_id);
	}

}

