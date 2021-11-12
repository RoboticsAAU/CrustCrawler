#include "Methods.h"

double IntegrationOperator(double currentValue, double inputIntegrationVal, unsigned long& looptime) {
	inputIntegrationVal += currentValue * (1 / looptime);
	return inputIntegrationVal;
}

double DifferentiationOperator(double currentValue, double previousValue, unsigned long& looptime) {
	return (currentValue - previousValue) / (1 / looptime);
}

void SpaceConverter(SpaceType desiredSpace) {
	//_AngleConverter(Radians);

	if (desiredSpace == MotionData.currentSpaceType) {
		return;
	}

	//Forward Jacobian:
	BLA::Matrix<3, 3> jacobian;
	jacobian(0, 0) = sin(AngleData.m_currentThetas[0]) * (Joint2.m_length * sin(AngleData.m_currentThetas[1]) + Joint3.m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	jacobian(0, 1) = -cos(AngleData.m_currentThetas[0]) * (Joint2.m_length * cos(AngleData.m_currentThetas[1]) + Joint3.m_length * cos(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	jacobian(0, 2) = -cos(AngleData.m_currentThetas[0]) * Joint3.m_length * cos(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]);
	jacobian(1, 0) = -cos(AngleData.m_currentThetas[0]) * (Joint2.m_length * sin(AngleData.m_currentThetas[1]) + Joint3.m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	jacobian(1, 1) = -sin(AngleData.m_currentThetas[0]) * (Joint2.m_length * cos(AngleData.m_currentThetas[1]) + Joint3.m_length * cos(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	jacobian(1, 2) = -sin(AngleData.m_currentThetas[0]) * Joint3.m_length * cos(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]);
	jacobian(2, 0) = 0;
	jacobian(2, 1) = -Joint2.m_length * sin(AngleData.m_currentThetas[1]) - Joint3.m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]);
	jacobian(2, 2) = -Joint3.m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]);

	//Inverse Jacobian:
	BLA::Matrix<3, 3> jacobianInverse = jacobian;
	bool is_nonsingular = Invert(jacobianInverse);

	//for (int i = 0; i < 3; i++) {
	//	for (int j = 0; j < 3; j++) {
	//		Serial.print(jacobianInverse(i, j));
	//		Serial.print(" ");
	//	}
	//	Serial.println(" ");
	//}
	//Serial.println(" ");


	//If the jacobian is singular. Wait implementation until further understanding.
	/*if(!is_nonsingular)
	{

	}*/

	BLA::Matrix<3, 1> velocityVector;
	velocityVector(0, 0) = Joint1.m_vel;
	velocityVector(1, 0) = Joint2.m_vel;
	velocityVector(2, 0) = Joint3.m_vel;

	switch (desiredSpace) {
	case JointSpace: {
		velocityVector = jacobianInverse * velocityVector;
	}
	case CartesianSpace: {
		velocityVector = jacobian * velocityVector;
	}
	}

	Joint1.m_vel = velocityVector(0, 0);
	Joint2.m_vel = velocityVector(1, 0);
	Joint3.m_vel = velocityVector(2, 0);

	MotionData.currentSpaceType = desiredSpace;
}