#include "DataStructures.h"

JointAngles JointAngles::operator+(const JointAngles& add) {
	if (this->currentUnitType != add.currentUnitType)
	{
		_typeConverter(this, add.currentUnitType);
	}
	JointAngles returnAngles;
	for (size_t i = 1; i < 6; i++)
	{
		returnAngles.thetas[i] = this->thetas[i] + add.thetas[i];
	}
	return returnAngles;
}

JointAngles JointAngles::operator+=(const JointAngles& addequal) {
	if (this->currentUnitType != addequal.currentUnitType)
	{
		_typeConverter(this, addequal.currentUnitType);
	}
	for (size_t i = 1; i < 6; i++)
	{
		this->thetas[i] += addequal.thetas[i];
	}
}

JointAngles JointAngles::operator-(const JointAngles& subtract) {
	if (this->currentUnitType != subtract.currentUnitType)
	{
		_typeConverter(this, subtract.currentUnitType);
	}
	JointAngles returnAngles;
	for (size_t i = 1; i < 6; i++)
	{
		returnAngles.thetas[i] = this->thetas[i] - subtract.thetas[i];
	}
	return returnAngles;
}

JointAngles JointAngles::operator-=(const JointAngles& subtractequal) {
	if (this->currentUnitType != subtractequal.currentUnitType)
	{
		_typeConverter(this, subtractequal.currentUnitType);
	}
	for (size_t i = 1; i < 6; i++)
	{
		this->thetas[i] -= subtractequal.thetas[i];
	}
}

void JointAngles::CovertTo(AngleUnitType newType)
{
	_typeConverter(this, newType);
}

void JointAngles::_typeConverter(JointAngles* inputAngles, AngleUnitType desiredUnit)
{
	if (desiredUnit == inputAngles->currentUnitType) {
		return;
	}

	double conversionConstant;

	switch (desiredUnit) {
	case Degree: {
		if (inputAngles->currentUnitType == Radians) {
			conversionConstant = 360 / (2 * M_PI);
			break;
		}
		if (inputAngles->currentUnitType == Raw) {
			conversionConstant = 360 / 4095;
			break;
		}
	}
	case Radians: {
		if (inputAngles->currentUnitType == Degree) {
			conversionConstant = (2 * M_PI) / 360;
			break;
		}
		if (inputAngles->currentUnitType == Raw) {
			conversionConstant = (2 * M_PI) / 4095;
			break;
		}
	}
	case Raw: {
		if (inputAngles->currentUnitType == Degree) {
			conversionConstant = 4095 / 360;
			break;
		}
		if (inputAngles->currentUnitType == Radians) {
			conversionConstant = 4095 / (2 * M_PI);
			break;
		}
	}
	}
	for (size_t i = 1; i < 6; i++)
	{
		inputAngles->thetas[i] *= conversionConstant;
	}
	inputAngles->currentUnitType = desiredUnit;
}

Velocities Velocities::operator+(const Velocities& add) {
	if (this->currentUnitType != add.currentUnitType)
	{
		_typeConverter(this, add.currentUnitType);
	}
	Velocities returnVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnVelocities.velocities[i] = this->velocities[i] + add.velocities[i];
	}
	return returnVelocities;
}

Velocities Velocities::operator+=(const Velocities& addequal) {
	if (this->currentUnitType != addequal.currentUnitType)
	{
		_typeConverter(this, addequal.currentUnitType);
	}
	for (size_t i = 1; i < 6; i++)
	{
		this->velocities[i] += addequal.velocities[i];
	}
}

Velocities Velocities::operator-(const Velocities& subtract) {
	if (this->currentUnitType != subtract.currentUnitType)
	{
		_typeConverter(this, subtract.currentUnitType);
	}
	Velocities returnVelocities;
	for (size_t i = 1; i < 6; i++)
	{
		returnVelocities.velocities[i] = this->velocities[i] - subtract.velocities[i];
	}
	return returnVelocities;
}

Velocities Velocities::operator-=(const Velocities& subtractequal) {
	if (this->currentUnitType != subtractequal.currentUnitType)
	{
		_typeConverter(this, subtractequal.currentUnitType);
	}
	for (size_t i = 1; i < 6; i++)
	{
		this->velocities[i] -= subtractequal.velocities[i];
	}
}

void Velocities::ConvertTo(VelocityUnitType newType)
{
	_typeConverter(this, newType);
}

void Velocities::_typeConverter(Velocities* inputVelocities, VelocityUnitType desiredUnit)
{
	if (desiredUnit == inputVelocities->currentUnitType) {
		return;
	}

	double conversionConstant;

	switch (desiredUnit) {
	case DegreesPerSec: {
		if (inputVelocities->currentUnitType == RadiansPerSec) {
			conversionConstant = 360 / (2 * M_PI);
			break;
		}
		if (inputVelocities->currentUnitType == RawsPerSec) {
			conversionConstant = 360 / 4095;
			break;
		}
	}
	case RadiansPerSec: {
		if (inputVelocities->currentUnitType == DegreesPerSec) {
			conversionConstant = (2 * M_PI) / 360;
			break;
		}
		if (inputVelocities->currentUnitType == RawsPerSec) {
			conversionConstant = (2 * M_PI) / 4095;
			break;
		}
	}
	case RawsPerSec: {
		if (inputVelocities->currentUnitType == DegreesPerSec) {
			conversionConstant = 4095 / 360;
			break;
		}
		if (inputVelocities->currentUnitType == RadiansPerSec) {
			conversionConstant = 4095 / (2 * M_PI);
			break;
		}
	}
	}
	for (size_t i = 1; i < 6; i++)
	{
		inputVelocities->velocities[i] *= conversionConstant;
	}
	inputVelocities->currentUnitType = desiredUnit;
}

