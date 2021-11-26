#include "Controller.h"

Controller::Controller() : conSys(&comCon), dynCon(&comCon), dyn(&comCon) {
	previousTime = millis(); 

	_maxJointLength = Joints[2]->Length + Joints[3]->Length + Joints[4]->Length;
	_maxAngularVelocity = MaxLinearVelocity / _maxJointLength;
	_LinearToAngularRatio = _maxAngularVelocity / MaxLinearVelocity;
}

void Controller::run()
{
	#ifdef DYNAMICS_TEST

	JointAngles desiredAngles;
	desiredAngles.thetas[2] = 20;
	desiredAngles.currentUnitType = Degree;
	desiredAngles.ConvertTo(Radians);

	//desiredAngles = dynCon.getJointAngles();

	Velocities desiredJointVelocities;

	Accelerations desiredAcceleration;

	JointTorques torques;

	//torques = dyn.InverseDynamics(desiredAngles, desiredJointVelocities, desiredAcceleration);
	
	dynCon.setJointPWM(torques, desiredJointVelocities);


	return;
	#endif // DYNAMICS_TEST

	_updateDeltaTime();

	// Get package sent from the computer 
	Package currentInstructions = comCon.getPackage();
	if (currentInstructions.isUpdated)
	{
		// Call emergency stop function if spacebar is pressed on the computer.
		if (currentInstructions.EmergencyStop)
		{
			dynCon.EmergencyStop();
			return;
		}

		// We read the data once per loop from the CrustCrawler
		JointAngles currentJointAngles = _getJointAngles(currentInstructions.Mode);		
		comCon.Print<char*>("\nCurrent Joint Angles:");
		comCon.Print<double>(currentJointAngles.thetas[1]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointAngles.thetas[2]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointAngles.thetas[3]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointAngles.thetas[4]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointAngles.thetas[5]);
		comCon.Print<char*>(" ");

		// We convert our instructions to joint velocities
		currentJointAngles.ConvertTo(Radians);
		Velocities desiredJointVelocities = _toJointVel(currentJointAngles, currentInstructions);

		Velocities currentJointVelocities = _getJointVelocities(currentInstructions.Mode);
		comCon.Print<char*>("\nCurrent joint velocities:");
		comCon.Print<double>(currentJointVelocities.velocities[1]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointVelocities.velocities[2]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointVelocities.velocities[3]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointVelocities.velocities[4]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(currentJointVelocities.velocities[5]);
		comCon.Print<char*>(" ");

#ifdef VELOCITY_CONTROL
		// If we control our robot by velocity, we can then just set the joint velocities now,
		// since the joints have their own control system
		desiredJointVelocities.ConvertTo(RPM);
		comCon.Print<char*>("\nDesired joint velocities:");
		comCon.Print<double>(desiredJointVelocities.velocities[1]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(desiredJointVelocities.velocities[2]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(desiredJointVelocities.velocities[3]);
		comCon.Print<char*>(" ");
		dynCon.setJointVelocity(desiredJointVelocities);
#endif // VELOCITY_CONTROL

#ifdef PWM_CONTROL

		// If we control our robot by PWM, then our robot has no internal control systems.
		// We therefore need to control/regulate them ourselves.

		// To control our robot we also need our current joint velocities
		Velocities currentJointVelocities = _getJointVelocities(currentInstructions.Mode);
		
		// We take our desired and current joint velocities and calculate our correction/error velocities
		Velocities errorVelocities = desiredJointVelocities - currentJointVelocities;
		
		// We calculate our torques from the control system
		errorVelocities.ConvertTo(RadiansPerSec);
		JointTorques controlTorques = conSys.Control(errorVelocities, currentJointAngles, deltaTime);

		// We calculate our static torques.
		Accelerations zeroAcceleration;
		JointTorques currentTorques = dyn.InverseDynamics(currentJointAngles, currentJointVelocities, zeroAcceleration);


		JointTorques goalTorques = currentTorques;

		// We then send this goal torque to the joints
		dynCon.setJointPWM(goalTorques, currentJointVelocities);

#endif // PWM_CONTROL
		// Add delay to get fixed loop time
		comCon.Print<char*>("\n");
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
		returnJointAngles.thetas[4] = dynCon.getJointAngle(*Joints[4]);
		returnJointAngles.thetas[5] = dynCon.getJointAngle(*Joints[5]);
		break;											   
	}													   
	case Base: case InOut: case UpDown: {				   
		returnJointAngles.thetas[1] = dynCon.getJointAngle(*Joints[1]);
		returnJointAngles.thetas[2] = dynCon.getJointAngle(*Joints[2]);
		returnJointAngles.thetas[3] = dynCon.getJointAngle(*Joints[3]);
		break;
	}
	case Lock: {
		returnJointAngles = dynCon.getJointAngles();
		return returnJointAngles;
	}
	}
	returnJointAngles.currentUnitType = Raw;
	return returnJointAngles;
}

Velocities Controller::_getJointVelocities(ControlMode controlMode)
{
	Velocities returnJointVelocities;
	switch (controlMode) {
	case Gripper: {
		returnJointVelocities.velocities[4] = dynCon.getJointVelocity(Joints[4]->ID);
		returnJointVelocities.velocities[5] = dynCon.getJointVelocity(Joints[5]->ID);
		break;
	}
	case Base: case InOut: case UpDown: {
		returnJointVelocities.velocities[1] = dynCon.getJointVelocity(Joints[1]->ID);
		returnJointVelocities.velocities[2] = dynCon.getJointVelocity(Joints[2]->ID);
		returnJointVelocities.velocities[3] = dynCon.getJointVelocity(Joints[3]->ID);
		break;
	}
	case Lock: {
		returnJointVelocities = dynCon.getJointVelocities();
		return returnJointVelocities;
	}
	}
	returnJointVelocities.currentUnitType = RawsPerSec;
	returnJointVelocities.currentSpaceType = JointSpace;
	return returnJointVelocities;
}

Velocities Controller::_toJointVel(JointAngles& jointAngles, Package& instructions)
{
	Velocities instructionVelocities = _toVel(instructions);
	Velocities instructionJointVelocities = _spaceConverter(jointAngles, instructionVelocities, JointSpace);
	instructionJointVelocities.currentUnitType = RadiansPerSec;

	breakVelocitiesAtLimit(jointAngles, instructionJointVelocities);

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
		returnVelocities.velocities[4] = -1*directionSign * speedMS;
		returnVelocities.velocities[5] = directionSign * speedMS;
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
		// Return velocities is initialised with 0
		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	default:
		// Invalid Control mode
		break;
	}
	return returnVelocities;
}

// This does not convert joint 4 and 5
Velocities Controller::_spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace)
{
	if (instructionVelocities.currentSpaceType == desiredSpace)
	{
		return instructionVelocities;
	}
	Velocities returnVelocities;
	jointAngles.ConvertTo(Radians);

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

	if (0.33 < returnVelocities.velocities[2] && 0.34 > returnVelocities.velocities[2] ||
		0.33 < returnVelocities.velocities[3] && 0.34 > returnVelocities.velocities[3]) {
		double _determinant = getDeterminant(jacobian);
	}
	returnVelocities.currentSpaceType = desiredSpace;
	return returnVelocities;
}

double Controller::getDeterminant(BLA::Matrix<3, 3> matrix) {
	return matrix(0, 0) * (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1)) - 
		   matrix(0, 1) * (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0)) + 
		   matrix(0, 2) * (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
}

void Controller::breakVelocitiesAtLimit(JointAngles& jointAngles, Velocities& instructionJointVelocities) {
	double angleDiff = 0;
	bool flag[6] = { 0,0,0,0,0,0 };
	jointAngles.ConvertTo(Raw);

	for (int i = 1; i < 6; i++) {
		// If the i'th joint is close to the lower angle limit and we are going towards the limit
		if ((jointAngles.thetas[i] < (Joints[i]->MinTheta + limitBoundary)) &&
			(instructionJointVelocities.velocities[i] < -1e-6))
		{
			angleDiff = jointAngles.thetas[i] - Joints[i]->MinTheta;

			// We break the velocity of the i'th joint
			breakVelocity(instructionJointVelocities.velocities[i], angleDiff);
			flag[i] = true;
		}
		// If the i'th joint is close to the upper angle limit and we are going towards the limit
		if ((jointAngles.thetas[i] > (Joints[i]->MaxTheta - limitBoundary)) &&
			(instructionJointVelocities.velocities[i] > 1e-6))
		{
			angleDiff = Joints[i]->MaxTheta - jointAngles.thetas[i];

			// We break the velocity of the i'th joint
			breakVelocity(instructionJointVelocities.velocities[i], angleDiff);
			flag[i] = true;
		}

	}

	// If the flag of the i'th velocity is true, meaning that joint is breaking
	for (int i = 1; i < 6; i++) {
		if (flag[i]) {
			switch (i) {
			case 1: break;
			case 2: case 4:
				if (!flag[i + 1]) {
					breakVelocity(instructionJointVelocities.velocities[i + 1], angleDiff);
					break;
				}
			case 3: case 5:
				if (!flag[i - 1]) {
					breakVelocity(instructionJointVelocities.velocities[i - 1], angleDiff);
					break;
				}
			}
		}
	}
}

void Controller::breakVelocity(double& velocity, double angleDiff) {
	// Breaking the velocity - as angleDiff goes to 0, so does the velocity.
	velocity = (velocity / limitBoundary) * angleDiff;

	// We set the velocity to 0 if it is under - happens when we have crossed the angle limit
	if (velocity < 0) velocity = 0;
}
