#include "DataStructures.h"

// Operator for addition between two objects of type JointAngles
JointAngles JointAngles::operator+(const JointAngles& add) {
	// Convert to same unit if they are different
	if (this->currentUnitType != add.currentUnitType)
	{
		_typeConverter(this, add.currentUnitType);
	}
	
	JointAngles returnAngles;

	// Element-wise addition for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnAngles.thetas[i] = this->thetas[i] + add.thetas[i];
	}
	returnAngles.currentUnitType = add.currentUnitType;
	
	return returnAngles;
}

// Operator for add equal between two objects of type JointAngles
JointAngles JointAngles::operator+=(const JointAngles& addequal) {
	// Convert to same unit if they are different
	if (this->currentUnitType != addequal.currentUnitType)
	{
		_typeConverter(this, addequal.currentUnitType);
	}
	
	// Element-wise add equal for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		this->thetas[i] += addequal.thetas[i];
	}
	
	this->currentUnitType = addequal.currentUnitType;
}

// Operator for subtraction between two objects of type JointAngles
JointAngles JointAngles::operator-(const JointAngles& subtract) {
	// Convert to same unit if they are different
	if (this->currentUnitType != subtract.currentUnitType)
	{
		_typeConverter(this, subtract.currentUnitType);
	}

	JointAngles returnAngles;

	// Element-wise subtraction for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnAngles.thetas[i] = this->thetas[i] - subtract.thetas[i];
	}
	returnAngles.currentUnitType = subtract.currentUnitType;
	
	return returnAngles;
}

// Operator for subtract equal between two objects of type JointAngles
JointAngles JointAngles::operator-=(const JointAngles& subtractequal) {
	// Convert to same unit if they are different
	if (this->currentUnitType != subtractequal.currentUnitType)
	{
		_typeConverter(this, subtractequal.currentUnitType);
	}
	
	// Element-wise subtract equal for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		this->thetas[i] -= subtractequal.thetas[i];
	}
	
	this->currentUnitType = subtractequal.currentUnitType;
}

// Convert an object of type JointAngles to the unit "newType" - the object is the one corresponding to the input "newType"
void JointAngles::ConvertTo(AngleUnitType newType)
{
	_typeConverter(this, newType);
}

// Function that converts the unit of "inputAngles" to a desired unit
void JointAngles::_typeConverter(JointAngles* inputAngles, AngleUnitType desiredUnit)
{
	// Return if the unit is already the desired unit
	if (desiredUnit == inputAngles->currentUnitType) {
		return;
	}

	// The variable "conversionConstant" is assigned to the constant that corresponds to the desired conversion
	double conversionConstant;

	// Depending on the desired unit, the "conversionConsant" is assigned fittingly
	switch (desiredUnit) {
	case Degree: { // If the desired unit is degrees
		if (inputAngles->currentUnitType == Radians) {
			conversionConstant = 360 / (2 * M_PI);
			break;
		}
		if (inputAngles->currentUnitType == Raw) {
			conversionConstant = 360 / 4095;
			break;
		}
	}
	case Radians: { // If the desired unit is radians
		if (inputAngles->currentUnitType == Degree) {
			conversionConstant = (2 * M_PI) / 360;
			break;
		}
		if (inputAngles->currentUnitType == Raw) {
			conversionConstant = (2 * M_PI) / 4095;
			break;
		}
	}
	case Raw: { // If the desired unit is raw
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
	
	// Multiplying the found constant onto the five joint angles
	for (size_t i = 1; i < 6; i++)
	{
		inputAngles->thetas[i] *= conversionConstant;
	}
	
	// Updating the current unit type to the desired unit
	inputAngles->currentUnitType = desiredUnit;
}

// Operator for addition between two objects of type Velocities
Velocities Velocities::operator+(const Velocities& add) {
	// Convert to same unit if they are different
	if (this->currentUnitType != add.currentUnitType)
	{
		_typeConverter(this, add.currentUnitType);
	}

	Velocities returnVelocities;

	// Element-wise addition for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnVelocities.velocities[i] = this->velocities[i] + add.velocities[i];
	}
	returnVelocities.currentUnitType = add.currentUnitType;
	
	return returnVelocities;
}

// Operator for add equal between two objects of type Velocities
Velocities Velocities::operator+=(const Velocities& addequal) {
	// Convert to same unit if they are different
	if (this->currentUnitType != addequal.currentUnitType)
	{
		_typeConverter(this, addequal.currentUnitType);
	}

	// Element-wise addition for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		this->velocities[i] += addequal.velocities[i];
	}

	this->currentUnitType = addequal.currentUnitType;
}

// Operator for subtraction between two objects of type Velocities
Velocities Velocities::operator-(const Velocities& subtract) {
	// Convert to same unit if they are different
	if (this->currentUnitType != subtract.currentUnitType)
	{
		_typeConverter(this, subtract.currentUnitType);
	}

	Velocities returnVelocities;
	
	// Element-wise subtraction for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnVelocities.velocities[i] = this->velocities[i] - subtract.velocities[i];
	}
	returnVelocities.currentUnitType = subtract.currentUnitType;
	
	return returnVelocities;
}

// Operator for subtract equal between two objects of type Velocities
Velocities Velocities::operator-=(const Velocities& subtractequal) {
	// Convert to same unit if they are different
	if (this->currentUnitType != subtractequal.currentUnitType)
	{
		_typeConverter(this, subtractequal.currentUnitType);
	}

	// Element-wise subtract equal for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		this->velocities[i] -= subtractequal.velocities[i];
	}

	this->currentUnitType = subtractequal.currentUnitType;
}

// Convert an object of type Velocities to the unit "newType" - the object is the one corresponding to the input "newType"
void Velocities::ConvertTo(VelocityUnitType newType)
{
	_typeConverter(this, newType);
}

// Function that converts the unit of "inputVelocities" to a desired unit
void Velocities::_typeConverter(Velocities* inputVelocities, VelocityUnitType desiredUnit)
{
	// Return if the unit is already the desired unit
	if (desiredUnit == inputVelocities->currentUnitType) {
		return;
	}

	// The variable "conversionConstant" is assigned to the constant that corresponds to the desired conversion
	double conversionConstant;

	// Depending on the desired unit, the "conversionConsant" is assigned fittingly
	switch (desiredUnit) {
	case DegreesPerSec: { // If the desired unit is degrees/second
		switch (inputVelocities->currentUnitType)
		{
		case RadiansPerSec: conversionConstant = 360 / (2 * M_PI); break;
		case RawsPerSec: conversionConstant = 360 / 4095; break;
		case RPM: conversionConstant = 360 / 60; break;
		}
		break;
	}
	case RadiansPerSec: { // If the desired unit is radians/second
		switch (inputVelocities->currentUnitType)
		{
		case DegreesPerSec: conversionConstant = (2 * M_PI) / 360; break;
		case RawsPerSec: conversionConstant = (2 * M_PI) / 4095; break;
		case RPM: conversionConstant = (2 * M_PI) / 60; break;
		}
		break;
	}
	case RawsPerSec: { // If the desired unit is raw/second
		switch (inputVelocities->currentUnitType)
		{
		case DegreesPerSec: conversionConstant = 4095 / 360; break;
		case RadiansPerSec: conversionConstant = 4095 / (2 * M_PI); break;
		case RPM: conversionConstant = 4095 / 60; break;
		}
		break;
	}
	case RPM: { // If the desired unit is revolutions/minute
		switch (inputVelocities->currentUnitType)
		{
		case DegreesPerSec: conversionConstant = 60 / 360; break;
		case RadiansPerSec: conversionConstant = 60 / (2 * M_PI); break;
		case RawsPerSec: conversionConstant = 60 / 4095; break;
		}
		break;
	}
	}

	// Multiplying the found constant onto the five joint velocities
	for (size_t i = 1; i < 6; i++)
	{
		inputVelocities->velocities[i] *= conversionConstant;
	}

	// Updating the current unit type to the desired unit
	inputVelocities->currentUnitType = desiredUnit;
}

// Operator for addition between two objects of type JointTorques
JointTorques JointTorques::operator+(const JointTorques& add)
{
	JointTorques returnTorques;

	// Element-wise addition for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] = this->torques[i] + add.torques[i];
	}

	return returnTorques;
}

// Operator for add equal between two objects of type JointTorques
JointTorques JointTorques::operator+=(const JointTorques& addequal)
{
	JointTorques returnTorques;

	// Element-wise add equal for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] += addequal.torques[i];
	}

	return returnTorques;
}

// Operator for subtraction between two objects of type JointTorques
JointTorques JointTorques::operator-(const JointTorques& subtract)
{
	JointTorques returnTorques;

	// Element-wise add equal for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] = this->torques[i] - subtract.torques[i];
	}

	return returnTorques;
}

// Operator for subtract equal between two objects of type JointTorques
JointTorques JointTorques::operator-=(const JointTorques& subtractequal)
{
	JointTorques returnTorques;

	// Element-wise subtract equal for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		returnTorques.torques[i] -= subtractequal.torques[i];
	}

	return returnTorques;
}
