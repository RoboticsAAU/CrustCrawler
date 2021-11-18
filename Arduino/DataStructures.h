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

struct JointAngles
{
	double thetas[6] = { 0,0,0,0,0,0 };
	AngleUnitType currentUnitType;

	JointAngles operator+=(const JointAngles& addequal) {
		for (size_t i = 1; i < 6; i++)
		{
			this->thetas[i] += addequal.thetas[i];
		}
	}
	JointAngles operator+(const JointAngles& add) {
		JointAngles returnAngles;
		for (size_t i = 1; i < 6; i++)
		{
			returnAngles.thetas[i] = this->thetas[i] + add.thetas[i];
		}
		return returnAngles;
	}
	JointAngles operator-=(const JointAngles& subtractequal) {
		for (size_t i = 1; i < 6; i++)
		{
			this->thetas[i] -= subtractequal.thetas[i];
		}
	}
	JointAngles operator-(const JointAngles& subtract) {
		JointAngles returnAngles;
		for (size_t i = 1; i < 6; i++)
		{
			returnAngles.thetas[i] = this->thetas[i] - subtract.thetas[i];
		}
		return returnAngles;
	}
};

struct JointTorques
{
	double torques[6] = { 0,0,0,0,0,0 };
};

struct Velocities
{
	double velocities[6] = { 0,0,0,0,0,0 };
	VelocityUnitType currentUnitType;
	SpaceType currentSpaceType;

	// Doesn't handle spaceconversions - the two objects "Velocities" must be of same SpaceType
	Velocities operator+=(const Velocities& addequal) {
		for (size_t i = 1; i < 6; i++)
		{
			this->velocities[i] += addequal.velocities[i];
		}
	}
	Velocities operator+(const Velocities& add) {
		Velocities returnVelocities;
		for (size_t i = 1; i < 6; i++)
		{
			returnVelocities.velocities[i] = this->velocities[i] + add.velocities[i];
		}
		return returnVelocities;
	}
	Velocities operator-=(const Velocities& subtractequal) {
		for (size_t i = 1; i < 6; i++)
		{
			this->velocities[i] -= subtractequal.velocities[i];
		}
	}
	Velocities operator-(const Velocities& subtract) {
		Velocities returnVelocities;
		for (size_t i = 1; i < 6; i++)
		{
			returnVelocities.velocities[i] = this->velocities[i] - subtract.velocities[i];
		}
		return returnVelocities;
	}
};

struct Accelerations {
	double accelerations[6] = { 0,0,0,0,0,0 };


};

struct Joint
{
	unsigned int ID;
	double Mass;
	double Length;
	int MinTheta, MaxTheta;
	int PWMlimit;
	ServoType ServoType;
};

struct eePosition
{
	double x, y, z;
};