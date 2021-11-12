#include "Kinematics.h"


Kinematics::Kinematics() {}

Kinematics::~Kinematics() {}

void Kinematics::UpdateForwardkinematics() {
	eePositionData.x = -cos(AngleData.m_currentThetas[0]) * (Joint2.m_length * sin(AngleData.m_currentThetas[1]) + Joint3.m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	eePositionData.y = -sin(AngleData.m_currentThetas[0]) * (Joint2.m_length * sin(AngleData.m_currentThetas[1]) + Joint3.m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	eePositionData.z = Joint1.m_length + Joint2.m_length * cos(AngleData.m_currentThetas[1]) + Joint3.m_length * cos(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]);
}
