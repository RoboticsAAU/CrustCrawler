#include "CrustCrawlerData.h"


//Initialising the joints with their corresponding physical proporties.
Joint CrustCrawler::Joint1 = {1, 10, 0.066, -180, 180, 10, MX64R};
Joint CrustCrawler::Joint2 = {2, 10, 0.22, 10, 10, 10, MX106R};
Joint CrustCrawler::Joint3 = {3, 10, 0.147, 10, 10, 10, MX64R};
Joint CrustCrawler::Joint4 = {4, 10, 0.115, 2179, 3072, 10, MX28R}; //maxTheta is 0.75*4095, minTheta is 10 from MID on blackboard (where grippers touch)
Joint CrustCrawler::Joint5 = {5, 10, 0.115, 1024, 1917, 10, MX28R}; //minTheta is 0.25*4095, maxTheta is 10 from MID on blackboard (where grippers touch)

Motion CrustCrawler::MotionData;
JointAngles CrustCrawler::AngleData;
eePosition CrustCrawler::eePositionData;

double CrustCrawler::timeData;
