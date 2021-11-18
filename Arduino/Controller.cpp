#include "Controller.h"

Controller::Controller() : conSys(this->conSys) {
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
	comCon.Print<char*>("\nDelta time: ");
	comCon.Print<double>(deltaTime*1000.0);

	// Get package
	Package currentInstructions = comCon.getPackage();
	if (currentInstructions.isUpdated)
	{
		if (currentInstructions.EmergencyStop)
		{
			dynCon.EmergencyStop();
			return;
		}
		// We get our current motion from the crustcrawler
		JointAngles currentJointAngles = dynCon.getJointAngles();
		currentJointAngles = _angleConverter(currentJointAngles, Radians);
		Velocities currentJointVelocities = dynCon.getJointVelocities();

		// We convert our instructions to joint velocities
		Velocities desiredJointVelocities = _toJointVel(currentJointAngles, currentInstructions);
		comCon.Print<char*>("\nDesired Joint Velocities: ");
		comCon.Print<double>(desiredJointVelocities.velocities[1]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(desiredJointVelocities.velocities[2]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(desiredJointVelocities.velocities[3]);
		comCon.Print<char*>(" ");

		// We take our desired and current joint velocities and calculate a correction velocity
		Velocities correctionVelocities = conSys.Control(currentJointVelocities, desiredJointVelocities, deltaTime);
		comCon.Print<char*>("\nCorrection velocities: ");
		comCon.Print<double>(correctionVelocities.velocities[1]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(correctionVelocities.velocities[2]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(correctionVelocities.velocities[3]);
		comCon.Print<char*>(" ");

		// Compute torques
		JointTorques jointTorqueCorrections = dyn.InverseDynamics(correctionVelocities, deltaTime);

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

Velocities Controller::_toJointVel(JointAngles& jointAngles, Package& instructions)
{
	Velocities instructionVelocities = _toVel(instructions);
	Velocities instructionJointVelocities = _spaceConverter(jointAngles, instructionVelocities, JointSpace);
	return instructionJointVelocities;
}

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
		returnVelocities.velocities[5] = -directionSign;
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
		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	default:
		// Invalid Control mode
		break;
	}
	return returnVelocities;
}

JointAngles Controller::_angleConverter(JointAngles& inputAngles, AngleUnitType desiredUnit) {
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
		if (inputAngles.currentUnitType == Degree) {
			conversionConstant = 4095 / (2 * M_PI);
			break;
		}
	}
	}
	for (size_t i = 1; i < 6; i++)
	{
		inputAngles.thetas[i] = inputAngles.thetas[i] * conversionConstant;
	}
	inputAngles.currentUnitType = desiredUnit;
}

Velocities Controller::_spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace)
{
	if (instructionVelocities.currentSpaceType == desiredSpace)
	{
		return instructionVelocities;
	}
	Velocities returnVelocities;
	jointAngles = _angleConverter(jointAngles, Radians);

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

	//Inverse Jacobian:
	BLA::Matrix<3, 3> jacobianInverse = jacobian;
	bool isNonSingular = Invert(jacobianInverse);

	//If the jacobian is singular
	if(!isNonSingular){
		// Some error message
	}

	BLA::Matrix<3,1> velocityVectorFrame1W;
	// Should probably be done by reference - for optimisation
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
		returnVelocities.velocities[i] = velocityVectorFrame1W(i - 1, 0);
	}
	returnVelocities.currentSpaceType = desiredSpace;
	return returnVelocities;
}
