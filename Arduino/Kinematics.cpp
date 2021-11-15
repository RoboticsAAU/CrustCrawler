#include "Kinematics.h"


Kinematics::Kinematics() {}

Kinematics::~Kinematics() {}

void Kinematics::UpdateForwardKinematics() {
	eePositionData.x = -cos(AngleData.m_currentThetas[0]) * (Joints[2]->m_length * sin(AngleData.m_currentThetas[1]) + Joints[3]->m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	eePositionData.y = -sin(AngleData.m_currentThetas[0]) * (Joints[2]->m_length * sin(AngleData.m_currentThetas[1]) + Joints[3]->m_length * sin(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]));
	eePositionData.z = Joints[1]->m_length + Joints[2]->m_length * cos(AngleData.m_currentThetas[1]) + Joints[3]->m_length * cos(AngleData.m_currentThetas[1] + AngleData.m_currentThetas[2]);
}
