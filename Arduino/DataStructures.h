#pragma once
#include <Arduino.h> // Defines the uint8_t

enum UnitType
{
	Degree,
	Radians,
	Raw
};

enum ServoType {
	MX28R,
	MX64R,
	MX106R
};

struct Package
{
	bool EmergencyStop;
	uint8_t Mode;
	bool Sign;
	uint8_t Speed;
	bool isUpdated;
};

struct JointAngles
{
	double thetas[6] = { 0,0,0,0,0,0 };
	UnitType currentUnitType;
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