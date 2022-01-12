#include "ControlSystem.h"
ControlSystem::ControlSystem(ComputerConnection* pointer) : pComCon(pointer) {} // The pointer is only for debugging purposes

JointTorques ControlSystem::Control(Velocities& errorVelocities, JointAngles& currentAngles, unsigned long& deltaTime)
{
	// We make sure our angles and velocities are in the correct units
	currentAngles.ConvertTo(Radians); 
	errorVelocities.ConvertTo(RadiansPerSec); 

	JointTorques returnJointTorques;

	// From the error velocities, we can create the regulation torques. This is done for all five joints
	for (size_t i = 1; i < 6; i++)
	{
		// Regulation torques are found based on the error velocities
			// NOTE: While the function is a PID-controller, the constants Ki and Kp are set to zero making it only a P-controller in this case
		returnJointTorques.torques[i] = _PID(errorVelocities.velocities[i], i, deltaTime);
	}

	return returnJointTorques;
}

// The function determines each of the three PID terms depending on the error and returns the sum of them
double ControlSystem::_PID(double& error, int&& iterator, unsigned long& deltaTime){
	// The proportional term
	double proportional = _P(Kp[iterator], error);

	// The derivative term
	double derivative = _D(Kd[iterator], error, lastError[iterator], deltaTime);

	// The integral term
	// If statement avoids integral windup by only activating the integral term when the response begins to settle. First part of the if statement
	// is true when reaching a sufficiently small error and second part is true when the  slope (derivative) is sufficiently small
		// NOTE: If we want to expand on this, we could limit it further by thresholding the derivative of the step reponse. 
	double slopeConstant = 1;
	if (abs(error) < 0.1 && abs(derivative / Kd[iterator]) < slopeConstant) {
		integral[iterator] = _I(Ki[iterator], error, integral[iterator], deltaTime);
	}
	else { 
		integral[iterator] = 0; 
	}

	// We update the last error to the current error
	lastError[iterator] = error;

	// We return the sum of the three terms
	return (proportional + integral[iterator] + derivative);
}

// The function returns the proportional term depending on the Kp value and the error
double ControlSystem::_P(double& Kp, double& error)
{
	return Kp * error;
}

// The function returns the integral term depending on the Ki value and the accumulated error
double ControlSystem::_I(double& Ki, double& error, double& integral, unsigned long& deltaTime)
{
	return Ki * Integrate(error, integral, deltaTime);
}

// The function returns the derivative term depending on the Kd value and the difference in error
double ControlSystem::_D(double& Kd, double& error, double& lastError, unsigned long& deltaTime)
{
	return  Kd * Differentiate(error, lastError, deltaTime);
}


/////////////////////////////////////////////////////////////////////////////
// THE FOLLOWING FUNCTIONS ARE NOT IMPLEMENTED, I.E. THEY ARE NEVER CALLED //
/////////////////////////////////////////////////////////////////////////////

double ControlSystem::_PD(double& error, int&& iterator, unsigned long& deltaTime)
{
	double proportional = _P(Kp[iterator], error);

	double derivative = _D(Kd[iterator], error, lastError[iterator], deltaTime);
	lastError[iterator] = error;

	return proportional + derivative;
}

void ControlSystem::_handleJointLimitations(Velocities& errorVelocities, JointAngles& currentAngles)
{
	bool correctionFlags[6] = { 0,0,0,0,0,0 };
	// We correct for joint limitations
	for (size_t i = 1; i < 6; i++)
	{
		// If the current joint have been corrected, we dont want to recorrect it. As this could cause an infinite loop.
		if (correctionFlags[i]) { continue; }

		switch (i) {
			// For joint 3 and 5, if the previous joint was corrected, we must also correct the current.
		case 3: case 5: if (correctionFlags[i - 1]) { errorVelocities.velocities[i] = _velocityBraker(i, currentAngles.thetas[i]); continue; }
			  // If joint 3 and 5 were corrected later down the chain, we are returned here, so we can correct the previous joint.
		case 2: case 4: if (correctionFlags[i + 1]) { errorVelocities.velocities[i] = _velocityBraker(i, currentAngles.thetas[i]); continue; }
		}

		// If the joint is not within our set boundaries, then we set the velocity to zero.
		// We could also restore it back to the limit here.
		if (!_isWithinAngleBoundaries(*Joints[i], currentAngles.thetas[i])) {
			errorVelocities.velocities[i] = 0;
			continue;
		}

		// Here we must be within the boundaries, so we check if the joint is close to the limits. If it is, then we must brake it. 
		else if (_isWithinBrakingThreshold(*Joints[i], currentAngles.thetas[i])) {
			_velocityBraker(i, currentAngles.thetas[i]);
			correctionFlags[i] = true;

			// If have corrected joint 3 or 5, we must return to return to correct joint 2 or 4.
			if (i == 3 || i == 5) { i -= 2; }
		}

		// If the joint has been determined to not be outside or close to its limit, we also need to check if the error velocity 
		//if (_isAboveSpeedLimit())
		//{
		//	errorVelocities.velocities[i] = jointSpeedLimit;
		//	correctionFlags[i] = true;
		//}
	}
}

double ControlSystem::_velocityBraker(int&& iterator, double& inputAngle)
{
	double angleDiff;
	double _boundaryMidPoint = (Joints[iterator]->MaxTheta + Joints[iterator]->MinTheta) / 2;

	// Max case
	if (inputAngle > _boundaryMidPoint) { angleDiff = inputAngle - Joints[iterator]->MinTheta; }
	// Min case
	else if (inputAngle < _boundaryMidPoint) { angleDiff = Joints[iterator]->MaxTheta - inputAngle; }
	
	// The braking formula
	return angleDiff / (angleDiff - brakingConstant);
}

bool ControlSystem::_isWithinAngleBoundaries(Joint& inputJoint, double inputAngle) {
	return (inputAngle > inputJoint.MinTheta) && (inputAngle < inputJoint.MaxTheta);
}

bool ControlSystem::_isWithinBrakingThreshold(Joint& inputJoint, double inputAngle) {
	return (inputAngle < (inputJoint.MinTheta + brakingThreshold) && inputAngle > inputJoint.MinTheta) || // Min braking zone
		   (inputAngle > (inputJoint.MaxTheta - brakingThreshold) && inputAngle < inputJoint.MaxTheta) ;  // Max braking zone
}

void ControlSystem::_gripperSynchronisation(Velocities& errorVelocities, JointAngles& currentAngles)
{
	// The zeropoints of the fingers are moved to be orthogonal from the link by the pi multiplications. 
	// Then the differences from each of the zero points are added together. 
	// If they are an equal distance from the moved zero points, this is zero
	double gripperError = currentAngles.thetas[5] - M_PI_2 + currentAngles.thetas[4] - 3 * M_PI_2;
	if (gripperError > 1e-6)
	{
		// Now we p regulate the error on each joint taking in to account their rotation direction
		errorVelocities.velocities[4] -= _P(gripperSyncGain, gripperError);
		errorVelocities.velocities[5] += _P(gripperSyncGain, gripperError);
	}
}


