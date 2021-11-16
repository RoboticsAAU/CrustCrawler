#include "Controller.h"

Controller::Controller() {
	previousTime = millis(); 

	for (size_t i = 0; i < 6; i++)
	{
		Joints[i].ID = i;
	}
}

void Controller::run()
{
	_updateDeltaTime();

	// Get package
	Package currentInstructions = comCon.getPackage();
	if (currentInstructions.isUpdated)
	{
		// We get our current motion from the crustcrawler
		JointAngles currentJointAngles = dynCon.getJointAngles();
		eePosition currentEEPos = kin.ForwardKinematics(currentJointAngles);
		JointVelocities currentJointVelocities = dynCon.getJointVelocities();
		eeVelocities currentEEVel = func();
		eeMotion currentMotion = _toMotion(currentEEPos, currentEEPVel);

		// We get our desired motion from our current instructions
		eeMotion desiredMotion = _toMotion(currentInstructions);

		// Control system
		eeMotion correctionMotion = conSys.Control(currentMotion, desiredMotion);

		// Compute torques
		JointTorques desiredJointTorques = dyn.InverseDynamics(correctionMotion);

		// Send torque to joints
		dynCon.setJointPWM(desiredJointTorques);
	}
}

void Controller::_updateDeltaTime()
{
	static unsigned long currentTime = millis();
	deltaTime = (currentTime - previousTime)/1000;
	previousTime = currentTime;
}
