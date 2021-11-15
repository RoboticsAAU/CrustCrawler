/*
	This file allows for the definitions to be found in all translation units
*/

#pragma once
// Arduino includes

// Library includes

// Custom includes
#include "DataStructures.h"


//----------------------------Configurable data------------------------
extern Joint Joint1;
extern Joint Joint2;
extern Joint Joint3;
extern Joint Joint4;
extern Joint Joint5;

#define DEBUG_SERIAL Serial1
#define DATA_SERIAL Serial2
	
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
#define DYNAMIXEL_SERIAL Serial
const unsigned int DIRECTION_PIN; // DYNAMIXEL Shield DIR PIN
const unsigned long DYNAMIXEL_BAUDRATE;
#endif

extern const double maxLinVelocity;


//Variables for conversion from max linear speed to max angular speed
extern const unsigned int maxReachableLength;
extern const double maxAngVelocity;
extern const double ratioLinToAng;


//----------------------------Active data------------------------------
extern Joint* Joints[6]; 

extern Motion MotionData;
extern JointAngles AngleData;
extern eePosition eePositionData;

extern double timeData;
