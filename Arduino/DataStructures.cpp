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

void JointAngles::ConvertTo(AngleUnitType newType)
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
		switch (inputVelocities->currentUnitType)
		{
		case RadiansPerSec: conversionConstant = 360 / (2 * M_PI); break;
		case RawsPerSec: conversionConstant = 360 / 4095; break;
		case RPM: conversionConstant = 360 / 60; break;
		}
		break;
	}
	case RadiansPerSec: {
		switch (inputVelocities->currentUnitType)
		{
		case DegreesPerSec: conversionConstant = (2 * M_PI) / 360; break;
		case RawsPerSec: conversionConstant = (2 * M_PI) / 4095; break;
		case RPM: conversionConstant = (2 * M_PI) / 60; break;
		}
		break;
	}
	case RawsPerSec: {
		switch (inputVelocities->currentUnitType)
		{
		case DegreesPerSec: conversionConstant = 4095 / 360; break;
		case RadiansPerSec: conversionConstant = 4095 / (2 * M_PI); break;
		case RPM: conversionConstant = 4095 / 60; break;
		}
		break;
	}
	case RPM:
		switch (inputVelocities->currentUnitType)
		{
		case DegreesPerSec: conversionConstant = 60 / 360; break;
		case RadiansPerSec: conversionConstant = 60 / (2 * M_PI); break;
		case RawsPerSec: conversionConstant = 60 / 4095; break;
		}
		break;
	}
	for (size_t i = 1; i < 6; i++)
	{
		inputVelocities->velocities[i] *= conversionConstant;
	}
	inputVelocities->currentUnitType = desiredUnit;
}

JointTorques JointTorques::operator+=(const JointTorques& addequal)
{
	JointTorques returnTorques;
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] += addequal.torques[i];
	}
	return returnTorques;
}

JointTorques JointTorques::operator+(const JointTorques& add)
{
	JointTorques returnTorques;
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] = this->torques[i] + add.torques[i];
	}
	return returnTorques;
}

JointTorques JointTorques::operator-=(const JointTorques& subtractequal)
{
	JointTorques returnTorques;
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] -= subtractequal.torques[i];
	}
	return returnTorques;
}

JointTorques JointTorques::operator-(const JointTorques& subtract)
{
	JointTorques returnTorques;
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] = this->torques[i] - subtract.torques[i];
	}
	return returnTorques;
}
