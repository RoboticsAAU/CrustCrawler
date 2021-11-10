#pragma once

enum UnitType {
	Degree,
	Radians,
	Raw
};

enum SpaceType {
	JointSpace,
	CartesianSpace
};

enum ServoType {
	MX28R,
	MX64R,
	MX106R
};

struct Joint { 
	unsigned int m_id;
	unsigned int m_mass; 
	unsigned int m_length;
	int16_t m_minTheta, m_maxTheta;
	uint16_t m_PWMlimit;
	ServoType m_servoType;
	//InertiaTensor inertiaTensor;

	double m_costantC1{ 0 }, m_constantC2{ 0 };

	Joint() {};

	Joint(unsigned int id, unsigned int mass, unsigned int length, int16_t minTheta, int16_t maxTheta, uint16_t PWMlimit, ServoType servoType) {
		m_id = id;
		m_mass = mass;
		m_length = length;
		m_minTheta = minTheta;
		m_maxTheta = maxTheta;
		m_PWMlimit = PWMlimit;
		m_servoType = servoType;
	};

	double m_torque{ 0 };
};

struct CrustCrawlerData
{
	Joint _Joint1;
	Joint _Joint2;
	Joint _Joint3;
	Joint _Joint4;
	Joint _Joint5;

	
	CrustCrawlerData() {
		//Joint Data
	}
};

struct JointAngles {
	double m_Theta1{ 0 }, m_Theta2{ 0 }, m_Theta3{ 0 }, m_Theta4{ 0 }, m_Theta5{ 0 };

	UnitType currentUnitType = Degree;

	JointAngles() {};

	JointAngles(UnitType inputUnitType) {
		currentUnitType = inputUnitType;
	};
};

struct Motion {
	double m_Pos1{ 0 }, m_Pos2{ 0 }, m_Pos3{ 0 };
	double m_Vel1{ 0 }, m_Vel2{ 0 }, m_Vel3{ 0 };
	double m_Acc1{ 0 }, m_Acc2{ 0 }, m_Acc3{ 0 };

	SpaceType currentSpaceType;

	Motion() {};

	Motion(SpaceType inputSpaceType) {
		currentSpaceType = inputSpaceType;
	};
};

struct eePosition {
	double x{ 0 }, y{ 0 }, z{ 0 };
};

