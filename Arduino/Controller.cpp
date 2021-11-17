#include "Controller.h"

Controller::Controller() {
	previousTime = millis(); 

	_maxJointLength = Joints[2]->Length + Joints[3]->Length + Joints[4]->Length;
	_maxAngularVelocity = _maxJointLength / MaxLinearVelocity;
	_LinearToAngularRatio = _maxAngularVelocity / MaxLinearVelocity;
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
		// We get our current motion from the crustcrawler
		JointAngles currentJointAngles = dynCon.getJointAngles();
		Velocities currentJointVelocities = dynCon.getJointVelocities();

		// We convert our instructions to joint velocities
		Velocities desiredJointVelocities = _toJointVel(currentJointAngles, currentInstructions);

		// We take our desired and current joint velocities and calculate a correction velocity
		Velocities correctionVelocities = conSys.Control(currentJointVelocities, desiredJointVelocities);

		// Compute torques
		JointTorques jointTorqueCorrections = dyn.InverseDynamics(correctionVelocities);

		// Send torque to joints
		dynCon.setJointPWM(jointTorqueCorrections);
	}
}

void Controller::_updateDeltaTime()
{
	static unsigned long currentTime = millis();
	deltaTime = (currentTime - previousTime)/1000;
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
	double speedMS = instructions.Speed / 1000;

	switch (instructions.Mode)
	{
	case Gripper: {
		// Should probably map to the instruction speed
		returnVelocities.velocities[4] = directionSign * Joints[4]->PWMlimit;
		returnVelocities.velocities[5] = -directionSign * Joints[5]->PWMlimit;
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
	if (instructionVelocities.currentSpaceType = desiredSpace)
	{
		return instructionVelocities;
	}
	Velocities returnVelocities;
	jointAngles = _angleConverter(jointAngles, Radians);

	//Forward Jacobian:
	BLA::Matrix<3, 3> jacobian; 
	jacobian(0,0) = sin(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(0,1) = -cos(inputAngles.m_Theta1) * (m_Joint2.m_length * cos(inputAngles.m_Theta2) + m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(0,2) = -cos(inputAngles.m_Theta1) * m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(1,0) = -cos(inputAngles.m_Theta1) * (m_Joint2.m_length * sin(inputAngles.m_Theta2) + m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(1,1) = -sin(inputAngles.m_Theta1) * (m_Joint2.m_length * cos(inputAngles.m_Theta2) + m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	jacobian(1,2) = -sin(inputAngles.m_Theta1) * m_Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(2,0) = 0;
	jacobian(2,1) = -m_Joint2.m_length * sin(inputAngles.m_Theta2) - m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);
	jacobian(2,2) = -m_Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3);

	//Inverse Jacobian:
	BLA::Matrix<3, 3> jacobianInverse = jacobian;
	bool is_nonsingular = Invert(jacobianInverse);

	//If the jacobian is singular - leave function 
	if(!is_nonsingular){
		
	}


	return returnVelocities;
}
