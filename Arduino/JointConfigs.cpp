#include "JointConfigs.h"

// Input variables:
// ID | MASS | LENGTH | MINTHETA | MAXTHETA | PWMLIMIT | SERVOTYPE 
Joint Joint1 = { 1, 10, 0.066, -180, 180, 10, MX64R };
Joint Joint2 = { 2, 10, 0.22, 10, 10, 10, MX106R };
Joint Joint3 = { 3, 10, 0.147, 10, 10, 10, MX64R };
Joint Joint4 = { 4, 10, 0.115, 2179, 3072, 10, MX28R }; //maxTheta is 0.75*4095, minTheta is 10 from MID on blackboard (where grippers touch)
Joint Joint5 = { 5, 10, 0.115, 1024, 1917, 10, MX28R }; //minTheta is 0.25*4095, maxTheta is 10 from MID on blackboard (where grippers touch)

// Makes array indexing easier
Joint* Joints[6] = { nullptr, &Joint1, &Joint2, &Joint3, &Joint4, &Joint5 };

// Defines the threshold before we recognise that the joint is moving (unit = ??)
int MovingThreshold = 5;

// Unit = m/s
const double MaxLinearVelocity = 0.15;
