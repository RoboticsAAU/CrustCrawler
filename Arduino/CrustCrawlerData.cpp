#include "CrustCrawlerData.h"

//Initialising the joints with their corresponding physical proporties.
Joint Joint1 = { 1, 10, 0.066, -180, 180, 10, MX64R};
Joint Joint2 = { 2, 10, 0.22, 10, 10, 10, MX106R};
Joint Joint3 = {3, 10, 0.147, 10, 10, 10, MX64R};
Joint Joint4 = {4, 10, 0.115, 10, 10, 10, MX28R};
Joint Joint5 = {5, 10, 0.115, 10, 10, 10, MX28R};

Motion MotionData;
JointAngles AngleData;
eePosition eePositionData;

double timeData;
