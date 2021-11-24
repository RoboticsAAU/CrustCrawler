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

	breakVelocityAtLimit(jointAngles, instructionJointVelocities);

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
	returnVelocities.currentSpaceType = desiredSpace;
	return returnVelocities;
}

void Controller::breakVelocityAtLimit(JointAngles& jointAngles, Velocities& instructionJointVelocities){

	jointAngles.ConvertTo(Raw);
	bool minFlags[6];
	bool maxFlags[6];
	for (int i = 1; i < 6; i++) {
		if (abs(instructionJointVelocities.velocities[i]) < 1e-6) {
			continue;
		}
		if (jointAngles.thetas[i] < (Joints[i]->MinTheta + BreakingThreshold)) {
			minFlags[i] = true;
		}
		if (jointAngles.thetas[i] > (Joints[i]->MaxTheta - BreakingThreshold)) {
			maxFlags[i] = true;
		}
	}

	if ( minFlags[1] || maxFlags[1] ) {
		// We can either have a max or min flag, never both 
		if (minFlags[1]){
			// The difference is positive when the angle is within boundary, and negative otherwise. 
			double minDiff = jointAngles.thetas[1] - Joints[1]->MinTheta;

			// If minDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits      
			if ( minDiff < 0){
				instructionJointVelocities.velocities[1] = 10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[1] *= minDiff / (minDiff + decelerationConstant);
			}
		}
		else if(maxFlags[1]) {
			//The difference is positive when the angle is within boundary, and negative otherwise. 
			double maxDiff = Joints[1]->MaxTheta - jointAngles.thetas[1];

			// If maxDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits     
			if (maxDiff < 0) {
				instructionJointVelocities.velocities[1] = -10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[1] *= maxDiff / (maxDiff + decelerationConstant);
			}
		}
	}

	if (minFlags[2] || maxFlags[2]) {
		// We can either have a max or min flag, never both 
		if (minFlags[2]) {
			// The difference is positive when the angle is within boundary, and negative otherwise. 
			double minDiff = jointAngles.thetas[2] - Joints[2]->MinTheta;

			// If minDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits      
			if (minDiff < 0) {
				instructionJointVelocities.velocities[2] = 10;
				jointAngles.ConvertTo(Radians);
			}
			else {
				instructionJointVelocities.velocities[2] *= minDiff / (minDiff + decelerationConstant);
				instructionJointVelocities.velocities[3] *= minDiff / (minDiff + decelerationConstant);
			}
		}
		else if (maxFlags[2]) {
			//The difference is positive when the angle is within boundary, and negative otherwise. 
			double maxDiff = Joints[2]->MaxTheta - jointAngles.thetas[2];

			// If maxDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits     
			if (maxDiff < 0) {
				instructionJointVelocities.velocities[2] = -10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[2] *= maxDiff / (maxDiff + decelerationConstant);
				instructionJointVelocities.velocities[3] *= maxDiff / (maxDiff + decelerationConstant);
			}
		}
	}

	if (minFlags[3] || maxFlags[3]) {
		// We can either have a max or min flag, never both 
		if (minFlags[3]) {
			// The difference is positive when the angle is within boundary, and negative otherwise. 
			double minDiff = jointAngles.thetas[3] - Joints[3]->MinTheta;

			// If minDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits      
			if (minDiff < 0) {
				instructionJointVelocities.velocities[3] = 10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[3] *= minDiff / (minDiff + decelerationConstant);
				instructionJointVelocities.velocities[2] *= minDiff / (minDiff + decelerationConstant);
			}
		}
		else if (maxFlags[3]) {
			//The difference is positive when the angle is within boundary, and negative otherwise. 
			double maxDiff = Joints[3]->MaxTheta - jointAngles.thetas[3];

			// If maxDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits     
			if (maxDiff < 0) {
				instructionJointVelocities.velocities[3] = -10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[3] *= maxDiff / (maxDiff + decelerationConstant);
				instructionJointVelocities.velocities[2] *= maxDiff / (maxDiff + decelerationConstant);
			}
		}
	}
	
		
	if (minFlags[4] || maxFlags[4]) {
		// We can either have a max or min flag, never both 
		if (minFlags[4]) {
			// The difference is positive when the angle is within boundary, and negative otherwise. 
			double minDiff = jointAngles.thetas[4] - Joints[4]->MinTheta;

			// If minDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits      
			if (minDiff < 0) {
				instructionJointVelocities.velocities[4] = 10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[4] *= minDiff / (minDiff + decelerationConstant);
			}
		}
		else if (maxFlags[4]) {
			//The difference is positive when the angle is within boundary, and negative otherwise. 
			double maxDiff = Joints[4]->MaxTheta - jointAngles.thetas[4];

			// If maxDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits     
			if (maxDiff < 0) {
				instructionJointVelocities.velocities[4] = -10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[4] *= maxDiff / (maxDiff + decelerationConstant);
			}
		}
	}

	if (minFlags[5] || maxFlags[5]) {
		// We can either have a max or min flag, never both 
		if (minFlags[5]) {
			// The difference is positive when the angle is within boundary, and negative otherwise. 
			double minDiff = jointAngles.thetas[5] - Joints[5]->MinTheta;

			// If minDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits      
			if (minDiff < 0) {
				instructionJointVelocities.velocities[5] = 10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[5] *= minDiff / (minDiff + decelerationConstant);
			}
		}
		else if (maxFlags[5]) {
			//The difference is positive when the angle is within boundary, and negative otherwise. 
			double maxDiff = Joints[5]->MaxTheta - jointAngles.thetas[5];

			// If maxDiff was negative, then we are outside the limits and need to regulate by a constant value till we are back inside the limits     
			if (maxDiff < 0) {
				instructionJointVelocities.velocities[5] = -10;
				jointAngles.ConvertTo(Radians);
				return;
			}
			else {
				instructionJointVelocities.velocities[5] *= maxDiff / (maxDiff + decelerationConstant);
			}
		}
	}
	
	jointAngles.ConvertTo(Radians);
}

