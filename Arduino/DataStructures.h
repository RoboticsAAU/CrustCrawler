#pragma once
#include <Arduino.h> // Defines the uint8_t

enum AngleUnitType
{
	Degree,
	Radians,
	Raw
};

enum OutputType
{
	Torque,
	PWM
};

enum SpaceType
{
	JointSpace,
	CartesianSpace
};

enum ServoType {
	MX28R,
	MX64R,
	MX106R
};

enum ControlMode
{
	Gripper,
	Base,
	InOut,
	UpDown,
	Lock
};

struct Package
{
	bool EmergencyStop;
	ControlMode Mode;
	bool Sign;
	uint8_t Speed;
	bool isUpdated;
};

struct JointAngles
{
	double thetas[6] = { 0,0,0,0,0,0 };
	AngleUnitType currentUnitType;
};

struct JointTorques
{
	double torques[6] = { 0,0,0,0,0,0 };
	OutputType type;
};

struct Velocities
{
	double velocities[6] = { 0,0,0,0,0,0 };
	SpaceType currentSpaceType;
};

struct Accelerations {
	double accelerations[6] = { 0,0,0,0,0,0 };
};


struct Joint
{
	unsigned int ID;
	unsigned int Mass;
	unsigned int Length;
	int MinTheta, MaxTheta;
	int PWMlimit;
	ServoType ServoType;
};

struct eePosition
{
	double x, y, z;
};