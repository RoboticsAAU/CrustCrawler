#pragma once


enum UnitType {
	Degree,
	Radians,
	Raw
};

struct Joint {

	unsigned int m_id;
	unsigned int m_mass;
	unsigned int m_length;
	double m_minTheta, m_maxTheta;
	double m_PWMlimit;

	Joint() {};

	Joint(unsigned int id, unsigned int mass, unsigned int length, double minTheta, double maxTheta, double PWMlimit) {
		m_id = id;
		m_mass = mass;
		m_length = length;
		m_minTheta = minTheta;
		m_maxTheta = maxTheta;
		m_PWMlimit = PWMlimit;
	};
};

struct JointAngles {
	double m_Theta1{ 0 }, m_Theta2{ 0 }, m_Theta3{ 0 }, m_Theta4{ 0 }, m_Theta5{ 0 };

	UnitType currentUnitType;

	JointAngles() {};

	JointAngles(UnitType inputUnitType) {
		currentUnitType = inputUnitType;
	};
};

struct eePosition {
	double x{ 0 }, y{ 0 }, z{ 0 };
};



