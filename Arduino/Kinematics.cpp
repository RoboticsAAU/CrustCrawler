#include "Kinematics.h"


Kinematics::Kinematics() {}

Kinematics::~Kinematics() {}

void Kinematics::UpdateForwardkinematics() {
	CrustCrawler::eePositionData.x = -cos(CrustCrawler::AngleData.m_currentThetas[0]) * (CrustCrawler::Joint2.m_length * sin(CrustCrawler::AngleData.m_currentThetas[1]) + CrustCrawler::Joint3.m_length * sin(CrustCrawler::AngleData.m_currentThetas[1] + CrustCrawler::AngleData.m_currentThetas[2]));
	CrustCrawler::eePositionData.y = -sin(CrustCrawler::AngleData.m_currentThetas[0]) * (CrustCrawler::Joint2.m_length * sin(CrustCrawler::AngleData.m_currentThetas[1]) + CrustCrawler::Joint3.m_length * sin(CrustCrawler::AngleData.m_currentThetas[1] + CrustCrawler::AngleData.m_currentThetas[2]));
	CrustCrawler::eePositionData.z = CrustCrawler::Joint1.m_length + CrustCrawler::Joint2.m_length * cos(CrustCrawler::AngleData.m_currentThetas[1]) + CrustCrawler::Joint3.m_length * cos(CrustCrawler::AngleData.m_currentThetas[1] + CrustCrawler::AngleData.m_currentThetas[2]);
}
