#pragma once
#include "CrustCrawlerData.h"
#include "math.h"


class Dynamics
{
public:
	Dynamics();
	~Dynamics();

	void UpdateDynamics(double& Looptime);

private:
	double _prevVel1{0}, _prevVel2{0}, _prevVel3{0};
	double _DifferentiationOperator(double currentValue, double previousValue, double& looptime);
	double _IntegrationOperator(double currentValue, double inputIntegrationVal, double& looptime);
	

};

