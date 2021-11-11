#include "Kinematics.h"


Kinematics::Kinematics() {

}

Kinematics::~Kinematics() {

}

void Kinematics::UpdateForwardkinematics() {
	m_eePosition.x = -cos(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.y = -sin(inputAngles.m_Theta1) * (Joint2.m_length * sin(inputAngles.m_Theta2) + Joint3.m_length * sin(inputAngles.m_Theta2 + inputAngles.m_Theta3));
	m_eePosition.z = Joint1.m_length + Joint2.m_length * cos(inputAngles.m_Theta2) + Joint3.m_length * cos(inputAngles.m_Theta2 + inputAngles.m_Theta3);
}
