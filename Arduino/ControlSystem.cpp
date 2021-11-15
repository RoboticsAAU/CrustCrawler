#include "ControlSystem.h"


ControlSystem::ControlSystem() {

}

ControlSystem::~ControlSystem() {

}

void ControlSystem::RunControlsystem() {

}

void ControlSystem::_GetJointPWMConstants(Joint& inputJoint) {
	SpaceConverter(JointSpace);


	int torque_sign = inputJoint.m_torque;
	int vel_sign = inputJoint.m_vel;

	int constantPicker = torque_sign * vel_sign;

	// if(constantPicker > 0) a
	// else if (constantPicker < 0) c
	// else b

	switch (inputJoint.m_servoType) {
	case MX28R: {

		if (constantPicker < 0) { inputJoint.m_constantC1 = 211.7; }
		else if (constantPicker = 0) { inputJoint.m_constantC1 = 427.4; }
		else if (constantPicker > 0) { inputJoint.m_constantC1 = 642.0; }

		inputJoint.m_constantC2 = 115.3;

	}
	case MX64R: {

		if (constantPicker < 0) { inputJoint.m_constantC1 = 80.9; }
		else if (constantPicker = 0) { inputJoint.m_constantC1 = 152.7; }
		else if (constantPicker > 0) { inputJoint.m_constantC1 = 224.5; }

		inputJoint.m_constantC2 = 105.3;

	}
	case MX106R: {

		if (constantPicker < 0) { inputJoint.m_constantC1 = 40.4; }
		else if (constantPicker = 0) { inputJoint.m_constantC1 = 83.9; }
		else if (constantPicker > 0) { inputJoint.m_constantC1 = 127.5; }

		inputJoint.m_constantC2 = 160.6;

	}
	}
}

bool ControlSystem::_IsWithinAngleBoundaries(Joint& inputJoint, double inputAngle) {
	return inputAngle >= inputJoint.m_minTheta && inputAngle <= inputJoint.m_maxTheta;
}

void ControlSystem::_TorqueToPWM(Joint& inputJoint, unsigned long looptime) {
	SpaceConverter(JointSpace);

	if (!_IsWithinAngleBoundaries(inputJoint, AngleData.m_currentThetas[inputJoint.m_id - 1])) {
		double _boundaryMidPoint = (inputJoint.m_maxTheta + inputJoint.m_minTheta) / 2;
		double _outputTheta = AngleData.m_currentThetas[inputJoint.m_id - 1] > _boundaryMidPoint ? inputJoint.m_maxTheta : inputJoint.m_minTheta;

		inputJoint.m_PWM = _PID(_outputTheta, AngleData.m_currentThetas[inputJoint.m_id - 1], looptime);
		return;
	}

	if (inputJoint.m_servoType == MX28R) { //If the joint is a gripper servo
		inputJoint.m_PWM = _constantGripperPWM * inputJoint.m_vel; //In reality the velocity is just a sign (positive or negative 1, or 0 if not in gripper mode)
	}
	else {
		// Getting the joint constants for the joint
		_GetJointPWMConstants(inputJoint);

		inputJoint.m_PWM = inputJoint.m_torque * inputJoint.m_constantC1 + inputJoint.m_vel * inputJoint.m_constantC2;
	}
}


double ControlSystem::_PID(double desiredValue, double currentValue, unsigned long Looptime) {
	double Kp{ 0.7 }, Ki{ 0.01 }, Kd{ 0.1 };
	double error = desiredValue - currentValue;

	_proportional = Kp * error;
	_integral = Ki * IntegrationOperator(error, _integral, Looptime);
	_derivative = Kd * DifferentiationOperator(error, _lastError, Looptime);

	//m_integral += Ki * (error * samplingTime);
	//m_derivative = Kd * ((error - m_lastError) / samplingTime);

	_lastError = error;

	return (_proportional + _integral + _derivative);
}

