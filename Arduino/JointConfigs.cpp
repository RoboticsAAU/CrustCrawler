#include "JointConfigs.h"

// Input variables:
// ID | LENGTH | MINTHETA | MAXTHETA | PWMLIMIT | SERVOTYPE 
Joint Joint1 = { 1, 0.066, -2047, 2047, 885, MX64R };
Joint Joint2 = { 2, 0.220, -1150, 1150, 885, MX106R };
Joint Joint3 = { 3, 0.147, -1200, 1200, 885, MX64R };
Joint Joint4 = { 4, 0.115, -2047, -1024, 885, MX28R }; //maxTheta is 0.75*4095, minTheta is 10 from MID on blackboard (where grippers touch)
Joint Joint5 = { 5, 0.115, 1024, 2047, 885, MX28R }; //minTheta is 0.25*4095, maxTheta is 10 from MID on blackboard (where grippers touch)

// Makes array indexing easier
Joint* Joints[6] = { nullptr, &Joint1, &Joint2, &Joint3, &Joint4, &Joint5 };

// Defines the threshold before we recognise that the joint is moving in range 0 ~ 1023 (unit = 0.229 rpm)
const int MovingThreshold = 5;

// Unit = m/s
const double MaxLinearVelocity = 0.15;
