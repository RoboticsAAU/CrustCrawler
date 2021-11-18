#include "Controller.h"

Controller::Controller() : conSys(&comCon), dynCon(&comCon), dyn(&comCon) {
	previousTime = millis(); 

	_maxJointLength = Joints[2]->Length + Joints[3]->Length + Joints[4]->Length;
	_maxAngularVelocity = MaxLinearVelocity / _maxJointLength;
	_LinearToAngularRatio = _maxAngularVelocity / MaxLinearVelocity;

	JointAngles startPosition = dynCon.getJointAngles();
	dyn.SetStartPos(startPosition);
}

void Controller::run()
{
	_updateDeltaTime();

	// Get package
	Package currentInstructions = comCon.getPackage();
	if (currentInstructions.isUpdated)
	{
		if (currentInstructions.EmergencyStop)
		{
			dynCon.EmergencyStop();
			return;
		}
		// 
		JointAngles currentJointAngles = _getJointAngles(currentInstructions.Mode);
		_typeConverter(currentJointAngles, Radians);
		Velocities currentJointVelocities = _getJointVelocities(currentInstructions.Mode);

		//currentJointVelocities = _typeConverter(currentJointVelocities, RadiansPerSec);

		
		// We convert our instructions to joint velocities
		Velocities desiredJointVelocities = _toJointVel(currentJointAngles, currentInstructions);
		_typeConverter(desiredJointVelocities, RadiansPerSec);
				
		// We take our desired and current joint velocities and calculate a correction velocity
		Velocities correctionVelocities = conSys.Control(currentJointVelocities, desiredJointVelocities, deltaTime);

		// Must be the same space type
		Velocities goalVelocities = correctionVelocities + currentJointVelocities;

		// Compute torques
		JointTorques jointTorqueCorrections = dyn.InverseDynamics(goalVelocities, deltaTime);

		// Send torque to joints
		dynCon.setJointPWM(jointTorqueCorrections, correctionVelocities, currentJointAngles);
	}
}

void Controller::_updateDeltaTime()
{
	unsigned long currentTime = millis();
	deltaTime = (currentTime - previousTime) / 1000.0;
	previousTime = currentTime;
}

JointAngles Controller::_getJointAngles(ControlMode controlMode)
{
	JointAngles returnJointAngles;
	switch (controlMode) {
	case Gripper: {
		returnJointAngles = dynCon.getJointAngle(Joints[4]->ID) + dynCon.getJointAngle(Joints[5]->ID);
		break;
	}
	case Base: case InOut: case UpDown: {
		returnJointAngles = dynCon.getJointAngle(Joints[1]->ID) + dynCon.getJointAngle(Joints[2]->ID) + dynCon.getJointAngle(Joints[3]->ID);
		break;
	}
	case Lock: {
		returnJointAngles = dynCon.getJointAngles();
		break;
	}
	}
	return returnJointAngles;
}

Velocities Controller::_getJointVelocities(ControlMode controlMode)
{
	Velocities returnJointVelocities;
	switch (controlMode) {
	case Gripper: {
		returnJointVelocities = dynCon.getJointVelocity(Joints[4]->ID) + dynCon.getJointVelocity(Joints[5]->ID);
		break;
	}
	case Base: case InOut: case UpDown: {
		returnJointVelocities = dynCon.getJointVelocity(Joints[1]->ID) + dynCon.getJointVelocity(Joints[2]->ID) + dynCon.getJointVelocity(Joints[3]->ID);
		break;
	}
	case Lock: {
		returnJointVelocities = dynCon.getJointVelocities();
		break;
	}
	}
	return returnJointVelocities;
}

Velocities Controller::_toJointVel(JointAngles& jointAngles, Package& instructions)
{
	Velocities instructionVelocities = _toVel(instructions);
	Velocities instructionJointVelocities = _spaceConverter(jointAngles, instructionVelocities, JointSpace);
	instructionJointVelocities.currentUnitType = RadiansPerSec;
	return instructionJointVelocities;
}

// Remember that in JointSpace the .velocities refer to the velocities of the individual joints, 
// where as in CartesianSpace only .velocities[1-3] are only used to refer to cartesian x,y,z- velocities
Velocities Controller::_toVel(Package& instructions)
{
	Velocities returnVelocities;
	
	int directionSign = instructions.Sign ? -1 : 1;
	double speedMS = instructions.Speed / 1000.0;

	switch (instructions.Mode)
	{
	case Gripper: {
		// Should probably map to the instruction speed
		returnVelocities.velocities[4] = directionSign;
		returnVelocities.velocities[5] = -1*directionSign;
		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	case Base: {
		returnVelocities.velocities[1] = directionSign * (speedMS * _LinearToAngularRatio);
		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	case InOut: {
		returnVelocities.velocities[1] = directionSign * speedMS;
		returnVelocities.currentSpaceType = CartesianSpace;
		break;
	}
	case UpDown: {
		returnVelocities.velocities[3] = directionSign * speedMS;
		returnVelocities.currentSpaceType = CartesianSpace;
		break;
	}
	case Lock: {
		// Return velocities are initialised with 0
		break;
	}
	default:
		// Invalid Control mode
		break;
	}
	return returnVelocities;
}

void Controller::_typeConverter(JointAngles& inputAngles, AngleUnitType desiredUnit) {
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
		if (inputAngles.currentUnitType == Radians) {
			conversionConstant = 4095 / (2 * M_PI);
			break;
		}
	}
	}
	for (size_t i = 1; i < 6; i++)
	{
		inputAngles.thetas[i] *= conversionConstant;
	}
	inputAngles.currentUnitType = desiredUnit;
}

void Controller::_typeConverter(Velocities& inputVelocities, VelocityUnitType desiredUnit) {
	if (desiredUnit == inputVelocities.currentUnitType) {
		return;
	}

	double conversionConstant;

	switch (desiredUnit) {
	case DegreesPerSec: {
		if (inputVelocities.currentUnitType == RadiansPerSec) {
			conversionConstant = 360 / (2 * M_PI);
			break;
		}
		if (inputVelocities.currentUnitType == RawsPerSec) {
			conversionConstant = 360 / 4095;
			break;
		}
	}
	case RadiansPerSec: {
		if (inputVelocities.currentUnitType == DegreesPerSec) {
			conversionConstant = (2 * M_PI) / 360;
			break;
		}
		if (inputVelocities.currentUnitType == RawsPerSec) {
			conversionConstant = (2 * M_PI) / 4095;
			break;
		}
	}
	case RawsPerSec: {
		if (inputVelocities.currentUnitType == DegreesPerSec) {
			conversionConstant = 4095 / 360;
			break;
		}
		if (inputVelocities.currentUnitType == RadiansPerSec) {
			conversionConstant = 4095 / (2 * M_PI);
			break;
		}
	}
	}
	for (size_t i = 1; i < 6; i++)
	{
		inputVelocities.velocities[i] *= conversionConstant;
	}
	inputVelocities.currentUnitType = desiredUnit;
}

// This does not convert joint 4 and 5
Velocities Controller::_spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace)
{
	if (instructionVelocities.currentSpaceType == desiredSpace)
	{
		return instructionVelocities;
	}
	Velocities returnVelocities;
	_typeConverter(jointAngles, Radians);

	// Jacobian:
	BLA::Matrix<3, 3> jacobian; 
	jacobian(0,0) = sin(jointAngles.thetas[1]) * (Joints[2]->Length * sin(jointAngles.thetas[2]) + Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(0,1) = -cos(jointAngles.thetas[1]) * (Joints[2]->Length * cos(jointAngles.thetas[2]) + Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(0,2) = -cos(jointAngles.thetas[1]) * Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]);
	jacobian(1,0) = -cos(jointAngles.thetas[1]) * (Joints[2]->Length * sin(jointAngles.thetas[2]) + Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(1,1) = -sin(jointAngles.thetas[1]) * (Joints[2]->Length * cos(jointAngles.thetas[2]) + Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(1,2) = -sin(jointAngles.thetas[1]) * Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]);
	jacobian(2,0) = 0;
	jacobian(2,1) = -Joints[2]->Length * sin(jointAngles.thetas[2]) - Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]);
	jacobian(2,2) = -Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]);

	BLA::Matrix<3,1> velocityVectorFrame1W;
	// Should probably be done be reference - for optimisation
	for (size_t i = 1; i < 4; i++)
	{
		velocityVectorFrame1W(i-1, 0) = instructionVelocities.velocities[i];
	}

	BLA::Matrix<3, 3> rotationMatrixFrame01;
	rotationMatrixFrame01(0, 0) = cos(jointAngles.thetas[1]);
	rotationMatrixFrame01(0, 1) = -sin(jointAngles.thetas[1]);
	rotationMatrixFrame01(0, 2) = 0;
	rotationMatrixFrame01(1, 0) = sin(jointAngles.thetas[1]);
	rotationMatrixFrame01(1, 1) = cos(jointAngles.thetas[1]);
	rotationMatrixFrame01(1, 2) = 0;
	rotationMatrixFrame01(2, 0) = 0;
	rotationMatrixFrame01(2, 1) = 0;
	rotationMatrixFrame01(2, 2) = 1;

	BLA::Matrix<3,1> velocityVectorFrame0W = rotationMatrixFrame01 * velocityVectorFrame1W;
	switch (desiredSpace)
	{
	case JointSpace: {
		//Inverse Jacobian:
		BLA::Matrix<3, 3> jacobianInverse = jacobian;
		bool isNonSingular = Invert(jacobianInverse);

		//If the jacobian is singular
		if (!isNonSingular) {
			// Some error message
		}
		velocityVectorFrame0W = jacobianInverse * velocityVectorFrame0W;
		break;
	}
	case CartesianSpace: {
		velocityVectorFrame0W = jacobian * velocityVectorFrame0W;
		break;
	}
	default:
		// Invalid desiredSpace
		break;
	}

	for (size_t i = 1; i < 4; i++)
	{
		returnVelocities.velocities[i] = velocityVectorFrame0W(i - 1, 0);
	}
	returnVelocities.currentSpaceType = desiredSpace;
	return returnVelocities;
}
