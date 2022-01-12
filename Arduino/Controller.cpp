#include "Controller.h"

Controller::Controller() : conSys(&comCon), dynCon(&comCon), dyn(&comCon) {
	// As the very first thing, the "previousTime" is set to the current time in microseconds (run once upon startup)
	previousTime = micros();

	// Variables later used in the function "toVel", where the parameters are converted to velocity
	// The first two variables ("_maxJointLength" and "_maxAngularVelocity") are only used to determine the third ("_LinearToAngularRatio")
	_maxJointLength = Joints[2]->Length + Joints[3]->Length + Joints[4]->Length;
	_maxAngularVelocity = MaxLinearVelocity / _maxJointLength;
	_LinearToAngularRatio = _maxAngularVelocity / MaxLinearVelocity;
}

// Main function that represent the loop structure.
void Controller::run()
{

	// Updating the difference in time between current loop iteration and last
	_updateDeltaTime();

	// Get currently read package 
	Package currentInstructions = comCon.getPackage();

	// Call emergency stop function if spacebar is pressed on the computer
	if (currentInstructions.EmergencyStop)
	{
		dynCon.EmergencyStop();
		return;
	}

	// We get and assign the current joint angles and velocities of the individual servos of the CrustCrawler
	JointAngles currentJointAngles = dynCon.getJointAngles();
	Velocities currentJointVelocities = dynCon.getJointVelocities();

	// We convert our instructions (the four retrieved parameters from the computer) to joint velocities
	Velocities desiredJointVelocities = _toJointVel(currentJointAngles, currentInstructions);

	// If macro is defined, we operate only by using the built-in controllers of the servos (controller / gravity compensation is not required in code).
	// This means that we just find the desired velocity of each servo
	#ifdef VELOCITY_OPERATINGMODE
		desiredJointVelocities.ConvertTo(RPM);
	#endif // VELOCITY_OPERATINGMODE

	// If macro is defined, the servos bypass built-in controllers  (controller / gravity compensation is required in code).
	// This means that we find the desired torque of each servo
	#ifdef PWM_OPERATINGMODE
		currentJointVelocities.ConvertTo(RadiansPerSec);

		// We calculate the torques required to keep maintain the CrustCrawler's current motion (model-based compensation)
		Accelerations zeroAcceleration;
		JointTorques currentTorques = dyn.InverseDynamics(currentJointAngles, currentJointVelocities, zeroAcceleration);

		// We calculate the additional torques as defined by the control system
		Velocities errorVelocities = desiredJointVelocities - currentJointVelocities; // Finding the error velocities
		JointTorques controlTorques = conSys.Control(errorVelocities, currentJointAngles, deltaTime);

		// The torques of model-based compensation and control system are added together to find the overall torques
		JointTorques goalTorques = controlTorques + currentTorques;
	#endif // PWM_OPERATINGMODE
	

	// The previous runtime is added to the accumulated runtime.
	// This is used below to ensure that the latest data is fetched if discrete-time differentiation is used
	accumulatedTime += deltaTime;

	// We check if another loop can be achieved with the same deltaTime. If it can, another loop is run before sending new values
	if (accumulatedTime + deltaTime < fixedSendTime) {
		return;
	}
	// If it cannot, the program is delayed to until achieving the fixed send time, and the new values are sent
	else if (accumulatedTime + deltaTime > fixedSendTime) {
		delayMicroseconds(fixedSendTime - accumulatedTime);

		// Instructions to joint servos are sent depending on which macro is defined for control
		#ifdef VELOCITY_OPERATINGMODE
			dynCon.setJointVelocity(desiredJointVelocities);
		#endif		
		#ifdef PWM_OPERATINGMODE
			dynCon.setJointPWM(goalTorques, currentJointVelocities);
		#endif

		//Since intructions are sent, the accumulated time is reset
		accumulatedTime = 0;
	}
}

// Function used to determine the last runtime
void Controller::_updateDeltaTime()
{
	unsigned long currentTime = micros();

	deltaTime = currentTime - previousTime;
	previousTime = currentTime;
}

// Function that converts the package intructions to joint velocity
Velocities Controller::_toJointVel(JointAngles& jointAngles, Package& instructions)
{
	jointAngles.ConvertTo(Radians);

	// The package instructions are converted to velocity (represented either in joint space or Cartesian space
	// depending on the instruction control mode)
	Velocities instructionVelocities = _toVel(instructions);
	
	// The intruction velocities are converted to joint space
	Velocities instructionJointVelocities = _spaceConverter(jointAngles, instructionVelocities, JointSpace);
	
	// Unit is assigned to rad/s, which is the same type returned by "_spaceConverter"
	instructionJointVelocities.currentUnitType = RadiansPerSec;

	// If the joints of the CrustCrawler are approaching an angle limit, the velocities are linearly decreased to zero
	brakeVelocitiesAtLimit(jointAngles, instructionJointVelocities);

	return instructionJointVelocities;
}



// The velocity in either Cartesian- or joint space is defined based on the instructions
	// NOTE: in JointSpace, the .velocities refer to the joint velocities of the individual joints, whereas in CartesianSpace, 
	// only .velocities[1-3] are used to refer to Cartesian x-, y-, and z- velocities, i.e. velocities[4-5] are zero
Velocities Controller::_toVel(Package& instructions)
{
	Velocities returnVelocities;

	// The boolean variable "Sign" is converted to a signed integer that represent movement direction. This is done for arithmetic purposes 
	directionSign = instructions.Sign ? 1 : -1;

	// The speed is converted to SI-unit (from mm/s to m/s)
	double speedMS = instructions.Speed / 1000.0;

	// If the last gripper direction corresponded to closing, we make the fingers close at a constant speed
	// This way, fingers can still close while in other modes
	if (_isClosing) {
		returnVelocities.velocities[4] = -_GripperCloseConstant;
		returnVelocities.velocities[5] = _GripperCloseConstant;
	}

	// Velocity is in the following defined based on the instruction mode
	switch (instructions.Mode)
	{
	case Gripper: {
		// We determine whether or not the current direction sign corresponds to closing
		if (instructions.Speed > 0) _isClosing = !instructions.Sign;
		
		// In velocity operating mode, we don't use the _isClosing feature - instead we just close like we open (done to avoid overload)
		#ifdef VELOCITY_OPERATINGMODE
			_isClosing = false;
		#endif

		// If we are not closing, we open with the user input velocity (times 3 for faster motion)
		if (!_isClosing) {
			returnVelocities.velocities[4] = 3 * directionSign * speedMS;
			returnVelocities.velocities[5] = -3 * directionSign * speedMS;
		}

		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	case Base: {
		// Base: theta1 correponding to [1] in joint space (multiplying by "_LinearToAngularRatio" to go from linear velocity to angular velocity)
		returnVelocities.velocities[1] = -directionSign * (speedMS * _LinearToAngularRatio); 
		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	case InOut: {
		// InOut: x-axis correponding to [1] in Cartesian space
		returnVelocities.velocities[1] = directionSign * speedMS;
		returnVelocities.currentSpaceType = CartesianSpace;
		break;
	}
	case UpDown: {
		// UpDown: z-axis correponding to [3] in Cartesian space
		returnVelocities.velocities[3] = directionSign * speedMS;
		returnVelocities.currentSpaceType = CartesianSpace;
		break;
	}
	case Lock: {
		// "returnVelocities" is initialised with 0, so no need to reassign
		returnVelocities.currentSpaceType = JointSpace;
		break;
	}
	default:
		// Invalid Control mode
		break;
	}

	return returnVelocities;
}

// Converts to a desired space (either joint space or Cartesian space) using Jacobians
	// NOTE: This does not convert joint 4 and 5
Velocities Controller::_spaceConverter(JointAngles& jointAngles, Velocities& instructionVelocities, SpaceType desiredSpace)
{
	// If the velocitities are already in the desired space, we simply return them
	if (instructionVelocities.currentSpaceType == desiredSpace)
	{
		return instructionVelocities;
	}
	
	Velocities returnVelocities;
	
	// Converting the joint angles to radians for use in the Jacobian
	jointAngles.ConvertTo(Radians);

	// Jacobian matrix (found from the CrustCrawler using Maple and subsequently inserted here):
	BLA::Matrix<3, 3> jacobian;
	jacobian(0, 0) = sin(jointAngles.thetas[1]) * (Joints[2]->Length * sin(jointAngles.thetas[2]) + Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(0, 1) = -cos(jointAngles.thetas[1]) * (Joints[2]->Length * cos(jointAngles.thetas[2]) + Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(0, 2) = -cos(jointAngles.thetas[1]) * Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]);
	jacobian(1, 0) = -cos(jointAngles.thetas[1]) * (Joints[2]->Length * sin(jointAngles.thetas[2]) + Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(1, 1) = -sin(jointAngles.thetas[1]) * (Joints[2]->Length * cos(jointAngles.thetas[2]) + Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]));
	jacobian(1, 2) = -sin(jointAngles.thetas[1]) * Joints[3]->Length * cos(jointAngles.thetas[2] + jointAngles.thetas[3]);
	jacobian(2, 0) = 0;
	jacobian(2, 1) = -Joints[2]->Length * sin(jointAngles.thetas[2]) - Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]);
	jacobian(2, 2) = -Joints[3]->Length * sin(jointAngles.thetas[2] + jointAngles.thetas[3]);

	// The velocities that were obtained from the instructions are seen from frame {1} to {W}. 
	// Such a vector is defined and assigned with the instruction velocities
	BLA::Matrix<3, 1> velocityVectorFrame1W;
	
		// NOTE: Should probably be done by reference - for optimisation
	for (size_t i = 1; i < 4; i++)
	{
		velocityVectorFrame1W(i - 1, 0) = instructionVelocities.velocities[i];
	}

	// A rotation matrix from frame {0} to {1} is defined for conversion of the velocity vector
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

	// The velocity vector is converted to be defined globally, i.e. from frame {0} to {W} using the found rotation matrix
	BLA::Matrix<3, 1> velocityVectorFrame0W = rotationMatrixFrame01 * velocityVectorFrame1W;
	
	// Switch for converting to the desired space
	switch (desiredSpace)
	{
	case JointSpace: { // If we want to convert from Cartesian space to joint space
		// Finding the inverse Jacobian
		BLA::Matrix<3, 3> jacobianInverse = jacobian;
		bool isNonSingular = Invert(jacobianInverse); // NOTE: The boolean "isNonSingular" is currently unused

		// The global velocity vector in joint space
		velocityVectorFrame0W = jacobianInverse * velocityVectorFrame0W;
		break;
	}
	case CartesianSpace: { // If we want to convert from joint space to Cartesian space
		// The global velocity vector in Cartesian space
		velocityVectorFrame0W = jacobian * velocityVectorFrame0W;
		break;
	}
	default:
		// Invalid "desiredSpace"
		break;
	}

	// Assigning the determinant of the Jacobian for use in determining limits when near singularities
		// NOTE: Multiplied by 1000 to get more usable values (otherwise the value is very small)
	double determinant = 1000 * getDeterminant(jacobian);

	// Turning the global velocity vector back into an object of the Velocities class
	for (size_t i = 1; i < 4; i++)
	{
		returnVelocities.velocities[i] = velocityVectorFrame0W(i - 1, 0);

		// If the configuration approaches a singularity, the velocity for each joint is non-linearly decreased to zero
		brakeVelocityAtSingularity(returnVelocities.velocities[i], determinant);
	}

	// We remember to insert the gripper velocities
	returnVelocities.velocities[4] = instructionVelocities.velocities[4];
	returnVelocities.velocities[5] = instructionVelocities.velocities[5];
	
	// We update the current space type to the one we have just converted to
	returnVelocities.currentSpaceType = desiredSpace;
	return returnVelocities;
}

// Function used to brake velocity depending on the size of the Jacobian's determinant. 
void Controller::brakeVelocityAtSingularity(double& velocity, double determinant) {
	// We check whether the current determinant is below a certain threshold. If true, the velocity is reduced by multiplying with a gain expressed through a power function.
	// NOTE: The shift ensures that joints stop before the singularity, while absolute determinant allows for both positive and negative approaches.
	if ((abs(determinant) - determinantShift) < determinantThreshold) { 
		// If current direction sign is equal the previous direction sign, it is assumed that the user continues to move towards a singularity and the velocity is thus reduced
		if (directionSign == prevDirectionSign) {
			velocity *= pow((abs(determinant) - determinantShift) / determinantThreshold, exp(1));
		}
	}
	// If not, the variable "prevDirectionSign" is assigned to current direction sign
	else {
		prevDirectionSign = directionSign;
	}
}

// Function to calculate the determinant of a 3x3 matrix
double Controller::getDeterminant(BLA::Matrix<3, 3> matrix) {
	return matrix(0, 0) * (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1)) -
		matrix(0, 1) * (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0)) +
		matrix(0, 2) * (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
}

// Function used to brake the velocity when joint angle limits are approached
void Controller::brakeVelocitiesAtLimit(JointAngles& jointAngles, Velocities& instructionJointVelocities) {
	// Local variable used to store angle difference
	double angleDiff = 0;
	// Local variable used to flag a joint if braking has occured. 
	// This is used to brake joint 2 and 3 as a pair during Cartesian movements whenever one of them reaches a limit. 
	bool flag[6] = { 0,0,0,0,0,0 };

	jointAngles.ConvertTo(Raw);

	// We check for each joint if they are within their limits 
	for (int i = 1; i < 6; i++) {

		// We check if the i'th joint is close to the lower angle limit and we are going towards the limit
		if (((jointAngles.thetas[i] < (Joints[i]->MinTheta + limitBoundary)) &&
			(instructionJointVelocities.velocities[i] < -1e-6)))
		{
			// The angle difference is calculated
			angleDiff = jointAngles.thetas[i] - Joints[i]->MinTheta;

			// We brake the velocity of the i'th joint
			brakeVelocityAtLimit(instructionJointVelocities.velocities[i], angleDiff);

			// The particular joint is flagged. 
			flag[i] = true;
		}

		// We check if the i'th joint is close to the upper angle limit and we are going towards the limit
		if ((jointAngles.thetas[i] > (Joints[i]->MaxTheta - limitBoundary)) &&
			(instructionJointVelocities.velocities[i] > 1e-6))
		{
			angleDiff = Joints[i]->MaxTheta - jointAngles.thetas[i];

			// We brake the velocity of the i'th joint
			brakeVelocityAtLimit(instructionJointVelocities.velocities[i], angleDiff);

			// The particular joint is flagged. 
			flag[i] = true;
		}

	}

	// We loop through each joint to check if the flag of the i'th velocity is true (meaning that joint braking has occurred)
	for (int i = 1; i < 6; i++) {
		// If braking has occurred
		if (flag[i]) {
			// Switch case is used to perform specific checks based on which joint has been flagged (currently only necessary for joint 2 and 3)
			switch (i) {
			case 1: break;
			case 2:
				// If braking has occurred for joint 2, we know that a Cartesian movement is being performed
				// The other joint used for this movement (joint 3) must thus also brake. 
				if (!flag[i + 1]) {
					brakeVelocityAtLimit(instructionJointVelocities.velocities[i + 1], angleDiff);
					break;
				}
			case 3:
				// If braking has occurred for joint 3, we know that a Cartesian movement is being performed
				// The other joint used for this movement (joint 2) must thus also brake
				if (!flag[i - 1]) {
					brakeVelocityAtLimit(instructionJointVelocities.velocities[i - 1], angleDiff);
					break;
				}
			}
		}
	}
}

// Function used to brake velocity when a joint limit has occurred.
void Controller::brakeVelocityAtLimit(double& velocity, double angleDiff) {
	// The sign of current velocity is stored. This is used to 
	double sign = copysign(1.0, velocity);
	// Braking the velocity - as angleDiff goes to 0, so does the velocity.
	velocity = (velocity / limitBoundary) * angleDiff;

	// We set the velocity to 0 if the expression is below zero. This is necessary to check as the angleDiff is negative when the limit has been crossed, 
	// which results in a velocity of opposite sign.
	if (sign * velocity < 0) velocity = 0;
}