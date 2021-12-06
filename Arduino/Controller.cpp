#include "Controller.h"

Controller::Controller() : conSys(&comCon), dynCon(&comCon), dyn(&comCon) {
	previousTime = micros(); 

	_maxJointLength = Joints[2]->Length + Joints[3]->Length + Joints[4]->Length;
	_maxAngularVelocity = MaxLinearVelocity / _maxJointLength;
	_LinearToAngularRatio = _maxAngularVelocity / MaxLinearVelocity;
}

void Controller::run()
{
	comCon.Print<char*>("\n");
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
	//comCon.Print<char*>("\nDeltatime: ");
	//comCon.Print<unsigned long>(deltaTime);
	
	// Get currently read package 
	Package currentInstructions = comCon.getPackage();

	// Call emergency stop function if spacebar is pressed on the computer.
	if (currentInstructions.EmergencyStop)
	{
		dynCon.EmergencyStop();
		return;
	}

	// We read the data once per loop from the CrustCrawler
	JointAngles currentJointAngles = dynCon.getJointAngles();
	//_getJointAngles(currentInstructions.Mode);
	

	/* {
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
	}*/
	

	// We convert our instructions to joint velocities
	Velocities desiredJointVelocities = _toJointVel(currentJointAngles, currentInstructions);

	Velocities currentJointVelocities = dynCon.getJointVelocities();
	//_getJointVelocities(currentInstructions.Mode);
	/*
	{
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
	}
	*/

#ifdef VELOCITY_CONTROL
	// If we control our robot by velocity, we can then just set the joint velocities now,
	// since the joints have their own control system
	desiredJointVelocities.ConvertTo(RPM);
	/*
	{
		comCon.Print<char*>("\nDesired joint velocities:");
		comCon.Print<double>(desiredJointVelocities.velocities[1]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(desiredJointVelocities.velocities[2]);
		comCon.Print<char*>(" ");
		comCon.Print<double>(desiredJointVelocities.velocities[3]);
		comCon.Print<char*>(" ");
	}
	*/

	// Ensures a fixed send time
	if (timeToNextSend < fixedSendTime) {
		timeToNextSend += deltaTime;
	}
	else {
		dynCon.setJointVelocity(desiredJointVelocities);
		comCon.Print<char*>("\nSent");
		timeToNextSend = 0;
	}
#endif // VELOCITY_CONTROL

#ifdef PWM_CONTROL

	// If we control our robot by PWM, then our robot has no internal control systems.
	// We therefore need to control/regulate them ourselves.

	// To control our robot we also need our current joint velocities
	//Velocities currentJointVelocities = _getJointVelocities(currentInstructions.Mode);
	
	//desiredJointVelocities.ConvertTo(RadiansPerSec);
	//desiredJointVelocities.velocities[1] = 1;
	currentJointVelocities.ConvertTo(RadiansPerSec);

	// We take our desired and current joint velocities and calculate our correction/error velocities
	Velocities errorVelocities = desiredJointVelocities - currentJointVelocities;
	
	// We calculate our torques from the control system
	JointTorques controlTorques = conSys.Control(errorVelocities, currentJointAngles, deltaTime);
	
	// We calculate our static torques.
	Accelerations zeroAcceleration;
	Velocities zeroVelocity;
	JointTorques currentTorques = dyn.InverseDynamics(currentJointAngles, currentJointVelocities, zeroAcceleration);
	//JointTorques currentTorques = dyn.InverseDynamics(currentJointAngles, zeroVelocity, zeroAcceleration);

	JointTorques goalTorques = controlTorques + currentTorques;
	
	accumulatedTime += deltaTime;
	// We check if another loop can be achieved with the same deltaTime. If not then instructions are sent
	if (accumulatedTime + deltaTime < fixedSendTime) {
		return;
	}

	else if(accumulatedTime + deltaTime > fixedSendTime){
		delayMicroseconds(fixedSendTime - accumulatedTime);
		dynCon.setJointPWM(goalTorques, currentJointVelocities);
		/*
		comCon.Print<char*>("\nErrorTime: ");
		comCon.Print<unsigned long>((fixedSendTime-deltaTime)/1000);

		unsigned long errorTime = (fixedSendTime - accumulatedTime)/1000;


		comCon.Print<char*>("\nIDTorque1: ");
		comCon.Print<double>(currentTorques.torques[1]);

		comCon.Print<char*>("\nControlTorque1: ");
		comCon.Print<double>(controlTorques.torques[1]);

		comCon.Print<char*>("\nGoalTorque1: ");
		comCon.Print<double>(goalTorques.torques[1]);

		comCon.Print<char*>("\nJointVelocity1: ");
		comCon.Print<double>(currentJointVelocities.velocities[1]);


		comCon.Print<char*>("\nIDTorque2: ");
		comCon.Print<double>(currentTorques.torques[2]);

		comCon.Print<char*>("\nControlTorque2: ");
		comCon.Print<double>(controlTorques.torques[2]);

		comCon.Print<char*>("\nGoalTorque2: ");
		comCon.Print<double>(goalTorques.torques[2]);

		comCon.Print<char*>("\nJointVelocity2: ");
		comCon.Print<double>(currentJointVelocities.velocities[2]);


		comCon.Print<char*>("\nIDTorque3: ");
		comCon.Print<double>(currentTorques.torques[3]);

		comCon.Print<char*>("\nControlTorque3: ");
		comCon.Print<double>(controlTorques.torques[3]);

		comCon.Print<char*>("\nGoalTorque3: ");
		comCon.Print<double>(goalTorques.torques[3]);

		comCon.Print<char*>("\nJointVelocity3: ");
		comCon.Print<double>(currentJointVelocities.velocities[3]);


		
		*/

		comCon.Print<char*>("\nJointVelocity1: ");
		comCon.Print<double>(currentJointVelocities.velocities[1]);

		comCon.Print<char*>("\nIDTorque1: ");
		comCon.Print<double>(100000*currentTorques.torques[1]);

		accumulatedTime = 0;
	}

#endif // PWM_CONTROL

}

void Controller::_updateDeltaTime()
{
	unsigned long currentTime = micros();
	//comCon.Print<char*>("\nCurrent time: ");
	//comCon.Print<unsigned long>(currentTime);
	deltaTime = currentTime - previousTime;
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
		returnJointVelocities.velocities[4] = dynCon.getJointVelocity(*Joints[4]);
		returnJointVelocities.velocities[5] = dynCon.getJointVelocity(*Joints[5]);
		break;														  
	}																  
	case Base: case InOut: case UpDown: {							  
		returnJointVelocities.velocities[1] = dynCon.getJointVelocity(*Joints[1]);
		returnJointVelocities.velocities[2] = dynCon.getJointVelocity(*Joints[2]);
		returnJointVelocities.velocities[3] = dynCon.getJointVelocity(*Joints[3]);
		break;
	}
	case Lock: {
		returnJointVelocities = dynCon.getJointVelocities();
		return returnJointVelocities;
	}
	}
	returnJointVelocities.currentUnitType = RPM;
	returnJointVelocities.currentSpaceType = JointSpace;
	return returnJointVelocities;
}

Velocities Controller::_toJointVel(JointAngles& jointAngles, Package& instructions)
{
	jointAngles.ConvertTo(Radians);
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
	
	directionSign = instructions.Sign ? -1 : 1;
	double speedMS = instructions.Speed / 1000.0;

	switch (instructions.Mode)
	{
	case Gripper: {
		// Should probably map to the instruction speed
		returnVelocities.velocities[4] = -3 * directionSign * speedMS;
		returnVelocities.velocities[5] = 3 * directionSign * speedMS;
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
		//if (!isNonSingular) {
		//	// Some error message
		//}
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

	double determinant = 1000 * getDeterminant(jacobian);
	//comCon.Print<char*>("\nDeterminant: ");
	//comCon.Print<double>(determinant);

	for (size_t i = 1; i < 4; i++)
	{	
		returnVelocities.velocities[i] = velocityVectorFrame0W(i - 1, 0);
		breakVelocityAtSingularity(returnVelocities.velocities[i], determinant);
	}

	returnVelocities.currentSpaceType = desiredSpace;
	return returnVelocities;
}

void Controller::breakVelocityAtSingularity(double& velocity, double determinant) {
	if ((abs(determinant) - determinantShift) < determinantThreshold) {
		if (directionSign == prevDirectionSign) {
			velocity *= pow((abs(determinant) - determinantShift) / determinantThreshold, exp(1));
		}
	}
	else {
		prevDirectionSign = directionSign;
	}
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
		if (((jointAngles.thetas[i] < (Joints[i]->MinTheta + limitBoundary)) &&
			(instructionJointVelocities.velocities[i] < -1e-6)))
		{
			angleDiff = jointAngles.thetas[i] - Joints[i]->MinTheta;

			// We break the velocity of the i'th joint
			breakVelocityAtLimit(instructionJointVelocities.velocities[i], angleDiff);
			flag[i] = true;
		}
		
		// If the i'th joint is close to the upper angle limit and we are going towards the limit
		if ((jointAngles.thetas[i] > (Joints[i]->MaxTheta - limitBoundary)) &&
			(instructionJointVelocities.velocities[i] > 1e-6))
		{
			angleDiff = Joints[i]->MaxTheta - jointAngles.thetas[i];

			// We break the velocity of the i'th joint
			breakVelocityAtLimit(instructionJointVelocities.velocities[i], angleDiff);
			flag[i] = true;
		}

	}

	// If the flag of the i'th velocity is true, meaning that joint is breaking
	for (int i = 1; i < 6; i++) {
		if (flag[i]) {
			switch (i) {
			case 1: break;
			case 2:
				if (!flag[i + 1]) {
					breakVelocityAtLimit(instructionJointVelocities.velocities[i + 1], angleDiff);
					break;
				}
			case 3:
				if (!flag[i - 1]) {
					breakVelocityAtLimit(instructionJointVelocities.velocities[i - 1], angleDiff);
					break;
				}
			}
		}
	}
}

void Controller::breakVelocityAtLimit(double& velocity, double angleDiff) {
	double sign = copysign(1.0, velocity);
	// Breaking the velocity - as angleDiff goes to 0, so does the velocity.
	velocity = (velocity / limitBoundary) * angleDiff;

	// We set the velocity to 0 if it is under - happens when we have crossed the angle limit
	if (sign*velocity < 0) velocity = 0;
}
