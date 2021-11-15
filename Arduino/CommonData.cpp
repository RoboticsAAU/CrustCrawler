
/*
	This file creates all the common data objects that used between all classes
*/

#pragma once
// Arduino includes

// Library includes

// Custom includes
#include "DataStructures.h"


//----------------------------Configurable data------------------------

// ID | MASS | LENGTH | MINTHETA | MAXTHETA | PWMLIMIT | SERVOTYPE //

Joint Joint1(1, 10, 0.066, -180, 180, 10, MX64R);
Joint Joint2(2, 10, 0.22, 10, 10, 10, MX106R);
Joint Joint3(3, 10, 0.147, 10, 10, 10, MX64R);
Joint Joint4(4, 10, 0.115, 2179, 3072, 10, MX28R); //maxTheta is 0.75*4095, minTheta is 10 from MID on blackboard (where grippers touch)
Joint Joint5(5, 10, 0.115, 1024, 1917, 10, MX28R); //minTheta is 0.25*4095, maxTheta is 10 from MID on blackboard (where grippers touch)


#define DEBUG_SERIAL Serial1
#define DATA_SERIAL Serial2
	
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DYNAMIXEL_SERIAL Serial
const unsigned int DIRECTION_PIN = 2; // DYNAMIXEL Shield DIR PIN
const unsigned long DYNAMIXEL_BAUDRATE = 57600;
#endif



//Variables for conversion from max linear speed to max angular speed
const unsigned int maxReachableLength = Joint1.m_length + Joint2.m_length + Joint3.m_length;
const double maxLinVelocity = 0.15;
const double maxAngVelocity = maxReachableLength / maxLinVelocity;
const double ratioLinToAng = maxAngVelocity / maxLinVelocity;

//----------------------------Active data------------------------------
Joint* Joints[6] = { nullptr, &Joint1, &Joint2, &Joint3, &Joint4, &Joint5 }; 

Motion MotionData;
JointAngles AngleData;
eePosition eePositionData;

double timeData;
