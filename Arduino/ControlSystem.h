#pragma once

// Arduino includes

// Library includes

// Custom includes
#include "CommonData.h"
#include "Methods.h"


class ControlSystem
{
public:
	ControlSystem();
	~ControlSystem();

	void RunControlsystem();

private:

	double _proportional{ 0.0 }, _integral{ 0.0 }, _derivative{ 0.0 }, _lastError{ 100.0 }, samplingTime = 1.0 / 200.0; //Sampling time should be changed

	const int _constantGripperPWM = 150;

	double _PID(double desiredValue, double currentValue, unsigned long Looptime);
	void _GetJointPWMConstants(Joint& inputJoint);
	bool _IsWithinAngleBoundaries(Joint& inputJoint, double inputAngle);
	void _TorqueToPWM(Joint& inputJoint, unsigned long looptime);

};

