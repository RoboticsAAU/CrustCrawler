#include "Methods.h"

double IntegrationOperator(double currentValue, double inputIntegrationVal, double& looptime) {
	inputIntegrationVal += currentValue * (1 / looptime);
	return inputIntegrationVal;
}

double DifferentiationOperator(double currentValue, double previousValue, double& looptime) {
	return (currentValue - previousValue) / (1 / looptime);
}

void SpaceConverter(SpaceType desiredSpace) {
	//_AngleConverter(Radians);

	if (desiredSpace == CrustCrawler::MotionData.currentSpaceType) {
		return;
	}

	//Forward Jacobian:
	BLA::Matrix<3, 3> jacobian;
	jacobian(0, 0) = sin(CrustCrawler::AngleData.m_Theta1) * (CrustCrawler::Joint2.m_length * sin(CrustCrawler::AngleData.m_Theta2) + CrustCrawler::Joint3.m_length * sin(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3));
	jacobian(0, 1) = -cos(CrustCrawler::AngleData.m_Theta1) * (CrustCrawler::Joint2.m_length * cos(CrustCrawler::AngleData.m_Theta2) + CrustCrawler::Joint3.m_length * cos(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3));
	jacobian(0, 2) = -cos(CrustCrawler::AngleData.m_Theta1) * CrustCrawler::Joint3.m_length * cos(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3);
	jacobian(1, 0) = -cos(CrustCrawler::AngleData.m_Theta1) * (CrustCrawler::Joint2.m_length * sin(CrustCrawler::AngleData.m_Theta2) + CrustCrawler::Joint3.m_length * sin(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3));
	jacobian(1, 1) = -sin(CrustCrawler::AngleData.m_Theta1) * (CrustCrawler::Joint2.m_length * cos(CrustCrawler::AngleData.m_Theta2) + CrustCrawler::Joint3.m_length * cos(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3));
	jacobian(1, 2) = -sin(CrustCrawler::AngleData.m_Theta1) * CrustCrawler::Joint3.m_length * cos(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3);
	jacobian(2, 0) = 0;
	jacobian(2, 1) = -CrustCrawler::Joint2.m_length * sin(CrustCrawler::AngleData.m_Theta2) - CrustCrawler::Joint3.m_length * sin(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3);
	jacobian(2, 2) = -CrustCrawler::Joint3.m_length * sin(CrustCrawler::AngleData.m_Theta2 + CrustCrawler::AngleData.m_Theta3);

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
	velocityVector(0, 0) = CrustCrawler::Joint1.m_vel;
	velocityVector(1, 0) = CrustCrawler::Joint2.m_vel;
	velocityVector(2, 0) = CrustCrawler::Joint3.m_vel;

	switch (desiredSpace) {
	case JointSpace: {
		velocityVector = jacobianInverse * velocityVector;
	}
	case CartesianSpace: {
		velocityVector = jacobian * velocityVector;
	}
	}

	CrustCrawler::Joint1.m_vel = velocityVector(0, 0);
	CrustCrawler::Joint2.m_vel = velocityVector(1, 0);
	CrustCrawler::Joint3.m_vel = velocityVector(2, 0);

	CrustCrawler::MotionData.currentSpaceType = desiredSpace;
}