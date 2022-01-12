#pragma once
// Generel includes

// Library includes
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <actuator.h>

// Custom includes
#include "DataStructures.h"
#include "JointConfigs.h"
#include "ComputerConnection.h"

// Defining which operating mode we want to run
//#define VELOCITY_OPERATINGMODE
#define PWM_OPERATINGMODE 

// Defining the serial port for communication between Arduino and the servos
#define DYNAMIXEL_SERIAL Serial

class DynamixelConnection
{
public:
	DynamixelConnection(ComputerConnection* pointer);

	void EmergencyStop();

	// Getting values from the CrustCrawler servos
	JointAngles getJointAngles();
	double getJointAngle(Joint& joint);

	Velocities getJointVelocities();
	double getJointVelocity(Joint& joint);

	double getJointLoad(Joint& joint); // THE FUNCTION IS NOT IMPLEMENTED, I.E. IT IS NEVER CALLED

	// Setting values on the CrustCrawler servos
	void setJointVelocity(Velocities& goalVelocities);
	void setJointPWM(JointTorques& updateTorques, Velocities& currentVelocities);

private:
	Dynamixel2Arduino dynamixel; // Object used to communicate with the servos
	ComputerConnection* pComCon; // Pointer to store the address of ComputerConnection object that is passed to the constructor

	// Determining the required PWM
	double _torqueToPWM(double& variable, double& currentVel, ServoType& servoType);
	void _getPWMConstants(double& desiredTorque, double& currentVel, ServoType& servoType);
	double torqueConstant;   // Variable to store the dynamic torque constant 
	double velocityConstant; // Variable to store the dynamic velocity constant

	// Variables used for angle interpretation
	int rawOffsets[6] = { 0, 0, 0, 0, 0, 0 }; // Array for determining the offset that must be added to each joint angle to get the angle in the correct interval
	double currentAngle;	 // Variable to store the current angle
};

