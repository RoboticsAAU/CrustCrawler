#include "DynamixelConnection.h"

// Constructor for DynamixelConnection class. The servos are here configured to a baudrate of 1M bps and the correct operating mode
// Additionally, angles outside an Raw angle range of -2047 and 2047 is offset 
DynamixelConnection::DynamixelConnection(ComputerConnection* pointer) : dynamixel(DYNAMIXEL_SERIAL, DirectionPin), pComCon(pointer)
{
	// Communication with dynamixel is initially set to 57600 bps (default baudrate of Dynamixel servos)
	// This is done to configure the servos to a higher baudrate
	dynamixel.begin(57600);

	// We loop through each servo and set the baudrate to 1M bps in control table
	for (size_t i = 1; i < 6; i++) {
		// To configure controltable, the torque must be turned off (See DYNAMIXEL Protocol 2.0 documentation). 
		// This is only done if the torque is enabled. 
		if (dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			dynamixel.torqueOff(Joints[i]->ID);
		}
		// Baudrate is set to "3", corresponding to a baudrate of 1M bps 
		dynamixel.writeControlTableItem(ControlTableItem::BAUD_RATE, Joints[i]->ID, 3);
	}

	// Communication with dynamixel is set to 1M to match the new servo baudrate
	dynamixel.begin(1000000);

	// We loop through each servo to define their operating mode and check for angle range
	for (size_t i = 1; i < 6; i++)
	{	
		// Torque is turned off for each servo if it is enabled (just in case)
		if (dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			dynamixel.torqueOff(Joints[i]->ID);
		}

		// The servo's operating mode is configured depending on the defined macro. This macro is defined in ""DynamixelConnection.h"
		#ifdef VELOCITY_OPERATINGMODE
			bool ControlMode = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_VELOCITY);
		#endif // VELOCITY_OPERATINGMODE
		#ifdef PWM_OPERATINGMODE
			bool ControlMode = dynamixel.writeControlTableItem(ControlTableItem::OPERATING_MODE, Joints[i]->ID, OperatingMode::OP_PWM);
		#endif // PWM_OPERATINGMODE

		// NOTE: Alternative solution to offset is to manually set the limits in the control table of each servo motor
		//bool Maxtheta = dynamixel.writeControlTableItem(ControlTableItem::MAX_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MaxTheta);
		//bool Mintheta = dynamixel.writeControlTableItem(ControlTableItem::MIN_POSITION_LIMIT, Joints[i]->ID, Joints[i]->MinTheta);

		// If the torque is off, we turn it on
		if (!dynamixel.getTorqueEnableStat(Joints[i]->ID))
		{
			dynamixel.torqueOn(Joints[i]->ID);
		}
		
		// The current joint angle is temporarily stored (in unit Raw)
		currentAngle = dynamixel.getPresentPosition(Joints[i]->ID, ParamUnit::UNIT_RAW);
		
		// A value representing half a rotation in Raw is stored with the same sign as currentAngle. This is needed because we are interested in obtaining the angle 
		// in the interval -2047 to 2047, and not 0 to 4095
		double offset = copysign(2047.0, currentAngle);

		// The number of full rotation offsets (corresponding to 4095 Raw) for the particular joint is defined. If the angle is within range, rawOffset is set to 0.
			// NOTE: The negative sign makes sure that the offset goes towards the desired interval instead of away from it
		rawOffsets[i] = (int)(-(currentAngle + offset) / 4095.0);
		
		// The number of offsets is multiplied by a whole rotation in Raw
		rawOffsets[i] *= 4095;	
	} 
}

// Function used to retrieve all current joint angles
JointAngles DynamixelConnection::getJointAngles()
{
	// Temporary object is instantiated to store joint angles
	JointAngles returnJointAngles;

	// The current joint angle for each servo is fetched
	for (size_t i = 1; i < 6; i++)
	{
		returnJointAngles.thetas[i] = getJointAngle(*Joints[i]);
	}
	// The correct unit type is assigned
	returnJointAngles.currentUnitType = Raw;
	
	return returnJointAngles;
}

// Function that returns the current joint angle of the input joint
double DynamixelConnection::getJointAngle(Joint& joint)
{
	double readAngle = dynamixel.getPresentPosition(joint.ID, ParamUnit::UNIT_RAW);

	// Uses the previously found offsets (in Raw) and makes the angle be within the correct interval, i.e. -2047 to 2047
	readAngle += rawOffsets[joint.ID];
	
	return readAngle;	
}

// Function used to retrieve all current joint velocities
Velocities DynamixelConnection::getJointVelocities()
{
	// Temporary object is instantiated to store joint velocities
	Velocities returnJointVelocities;

	// The current joint velocity for each servo is fetched
	for (size_t i = 1; i < 6; i++)
	{
		returnJointVelocities.velocities[i] = getJointVelocity(*Joints[i]);
	}

	// The correct unit type and space is assigned
	returnJointVelocities.currentUnitType = RPM;
	returnJointVelocities.currentSpaceType = JointSpace;
	
	return returnJointVelocities;
}

// Function used to retrieve the current joint velocity of a single servo
double DynamixelConnection::getJointVelocity(Joint& joint)
{
	return dynamixel.getPresentVelocity(joint.ID, ParamUnit::UNIT_RPM);
}

// Function used for emergency stop
void DynamixelConnection::EmergencyStop()
{
	// In case of emergency stop, the torque for all servos is turned off
	for (size_t i = 1; i < 6; i++)
	{
		dynamixel.torqueOff(Joints[i]->ID);
	}
}

// Function used to set the joint velocity (used for velocity operating mode)
void DynamixelConnection::setJointVelocity(Velocities& goalVelocities)
{	
	// Velocities are converted to the expected unit (RPM)
	goalVelocities.ConvertTo(RPM);

	// Velocity is set for each individual servo
	for (size_t i = 1; i < 6; i++)
	{
		bool set = dynamixel.setGoalVelocity(Joints[i]->ID, (int)goalVelocities.velocities[i], ParamUnit::UNIT_RPM);
	}
}

// Function used to set the joint PWM (used for PWM operating mode)
void DynamixelConnection::setJointPWM(JointTorques& updateTorques, Velocities& currentVelocities)
{
	// Converting the velocity to SI-units for use in the function _torqueToPWM()
	currentVelocities.ConvertTo(RadiansPerSec);

	//  Determining and setting the PWM of each joint
	for (size_t i = 1; i < 6; i++)
	{
		double jointPWM = _torqueToPWM(updateTorques.torques[i], currentVelocities.velocities[i], Joints[i]->ServoType);
		bool set = dynamixel.setGoalPWM(Joints[i]->ID, jointPWM);
	}
}

// Function that converts a given torque (and corresponding current velocity) to PWM using the PWM formula
double DynamixelConnection::_torqueToPWM(double& variable, double& currentVel, ServoType& servoType)
{
	// Determining the constants used in the PWM formula
	_getPWMConstants(variable, currentVel, servoType);

	// Returning the calculated PWM
	return variable * torqueConstant + currentVel * velocityConstant;
}

// Function that determines the torque constant and velocity constant used in the PWM formula
void DynamixelConnection::_getPWMConstants(double& desiredTorque, double& currentVel, ServoType& servoType) {
	
	// Variable that is used to determine which torque constant is selected
	int constantPicker = 0;

	//If the value is not approximately equal to zero, the constant picker is updated depending on the direction of the torque and velocity
	if( !( abs(desiredTorque * currentVel) < 1e-6 ) ){
		constantPicker = (int)copysign(1, (desiredTorque * currentVel));
	}

	// The consants are determined depending on which servo type the servo at the joint is
		// NOTE: The selection tree for the constants can be seen in the report page 44 
	switch(servoType) {
	case MX28R: { // If the servo is MX28R (joint 4 and 5)
		if (constantPicker < 0) { torqueConstant = 211.7; }
		else if (constantPicker == 0) { torqueConstant = 427.4; }
		else if (constantPicker > 0) { torqueConstant = 642.0; }
		velocityConstant = 115.3;
		break;
	}
	case MX64R: { // If the servo is MX64R (joint 1 and 3)
		if (constantPicker < 0) { torqueConstant = 80.9; }
		else if (constantPicker == 0) { torqueConstant = 152.7; }
		else if (constantPicker > 0) { torqueConstant = 224.5; }
		velocityConstant = 105.3;
		break;
	}
	case MX106R: { // If the servo is MX106R (joint 2)
		if (constantPicker < 0) { torqueConstant = 40.4; }
		else if (constantPicker == 0) { torqueConstant = 83.9; }
		else if (constantPicker > 0) { torqueConstant = 127.5; }
		velocityConstant = 160.6;
		break;
	}
	}
}

/////////////////////////////////////////////////////////////////////////////
// THE FOLLOWING FUNCTIONS ARE NOT IMPLEMENTED, I.E. THEY ARE NEVER CALLED //
/////////////////////////////////////////////////////////////////////////////

double DynamixelConnection::getJointLoad(Joint& joint) {
	//getPresentCurrent returns the load percentage 
	return dynamixel.getPresentCurrent(joint.ID, ParamUnit::UNIT_PERCENT);
}
