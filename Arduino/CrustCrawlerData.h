#pragma once
#include "DataStructures.h"

struct CrustCrawler
{
	//Implying that these objects will be initialized in another translation unit. (CrustCrawlerData.cpp)
	static Joint Joint1; 
	static Joint Joint2;
	static Joint Joint3;
	static Joint Joint4;
	static Joint Joint5;

	static Motion MotionData;
	static JointAngles AngleData;
	static eePosition eePositionData;

	static double timeData;
};


