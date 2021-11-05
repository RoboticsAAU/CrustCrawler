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

enum OperationType {
	Differentiation,
	Integration
};

struct Joint { 
	unsigned int m_id;
	unsigned int m_mass; 
	unsigned int m_length;
	double m_minTheta, m_maxTheta;
	double m_PWMlimit;
	//InertiaTensor inertiaTensor;

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

struct Velocities {
	double m_Vel1{ 0 }, m_Vel2{ 0 }, m_Vel3{ 0 };

	SpaceType currentSpaceType;

	Velocities() {};

	Velocities(SpaceType inputSpaceType) {
		currentSpaceType = inputSpaceType;
	};
};

struct eePosition {
	double x{ 0 }, y{ 0 }, z{ 0 };
};

//struct InertiaTensor {
//	double Ixx, Iyy, Izz;
//};


