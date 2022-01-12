#include "JointConfigs.h"

// Input variables:
// ID | LINKLENGTH | MINANGLE | MAXANGLE | PWMLIMIT | SERVOTYPE 
Joint Joint1 = { 1, 0.066, -2047, 2047, 885, MX64R };
Joint Joint2 = { 2, 0.220, -1250, 1250, 885, MX106R };
Joint Joint3 = { 3, 0.147, -1250, 1250, 885, MX64R };
Joint Joint4 = { 4, 0.115, -2047, -1024, 885, MX28R };
Joint Joint5 = { 5, 0.115, 1024, 2047, 885, MX28R };

// Makes array indexing easier (avoids zero indexing, meaning it is possible to simply use the joints' ID when iterating through)
Joint* Joints[6] = { nullptr, &Joint1, &Joint2, &Joint3, &Joint4, &Joint5 };

// Unit = m/s
const double MaxLinearVelocity = 0.15;
