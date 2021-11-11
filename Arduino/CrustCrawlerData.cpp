#include "CrustCrawlerData.h"

//Initialising the joints with their corresponding physical proporties.
Joint Joint1 { 1, 10, 6.6, -180, 180, 10, MX64R};
Joint Joint2 = { 2, 10, 22.0, 10, 10, 10, MX106R};
Joint Joint3 = {3, 10, 14.7, 10, 10, 10, MX64R};
Joint Joint4 = {4, 10, 10, 10, 10, 10, MX28R};
Joint Joint5 = {5, 10, 10, 10, 10, 10, MX28R};

Motion MotionData;
JointAngles AngleData;
eePosition eePositionData;

double timeData;
