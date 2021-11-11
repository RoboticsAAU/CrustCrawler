#include "Kinematics.h"


Kinematics::Kinematics() {}

Kinematics::~Kinematics() {}

void Kinematics::UpdateForwardkinematics() {
	eePositionData.x = -cos(AngleData.m_Theta1) * (Joint2.m_length * sin(AngleData.m_Theta2) + Joint3.m_length * sin(AngleData.m_Theta2 + AngleData.m_Theta3));
	eePositionData.y = -sin(AngleData.m_Theta1) * (Joint2.m_length * sin(AngleData.m_Theta2) + Joint3.m_length * sin(AngleData.m_Theta2 + AngleData.m_Theta3));
	eePositionData.z = Joint1.m_length + Joint2.m_length * cos(AngleData.m_Theta2) + Joint3.m_length * cos(AngleData.m_Theta2 + AngleData.m_Theta3);
}
