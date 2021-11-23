#pragma once
#include <Arduino.h> // Defines the uint8_t

enum AngleUnitType
{
	Degree,
	Radians,
	Raw
};

enum VelocityUnitType
{
	DegreesPerSec,
	RadiansPerSec,
	RawsPerSec
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

struct JointTorques
{
	double torques[6] = { 0,0,0,0,0,0 };

	JointTorques operator+=(const JointTorques& addequal);
	JointTorques operator+(const JointTorques& add);
	JointTorques operator-=(const JointTorques& subtractequal);
	JointTorques operator-(const JointTorques& subtract);
};

struct JointAngles
{
	double thetas[6] = { 0,0,0,0,0,0 };
	AngleUnitType currentUnitType;

	void CovertTo(AngleUnitType newType);

	JointAngles operator+=(const JointAngles& addequal);
	JointAngles operator+(const JointAngles& add);
	JointAngles operator-=(const JointAngles& subtractequal);
	JointAngles operator-(const JointAngles& subtract);

private:
	void _typeConverter(JointAngles* inputAngles, AngleUnitType desiredUnit);
};


struct Velocities
{
	double velocities[6] = { 0,0,0,0,0,0 };
	VelocityUnitType currentUnitType;
	SpaceType currentSpaceType;

	void ConvertTo(VelocityUnitType newType);

	// Doesn't handle spaceconversions - the two objects "Velocities" must be of same SpaceType
	Velocities operator+=(const Velocities& addequal);
	Velocities operator+(const Velocities& add);
	Velocities operator-=(const Velocities& subtractequal);
	Velocities operator-(const Velocities& subtract);
private:
	void _typeConverter(Velocities* inputVelocities, VelocityUnitType desiredUnit);
};

struct Accelerations {
	double accelerations[6] = { 0,0,0,0,0,0 };
};

struct Joint
{
	unsigned int ID;
	double Length;
	int MinTheta, MaxTheta;
	int PWMlimit;
	ServoType ServoType;
};

struct eePosition
{
	double x, y, z;
};