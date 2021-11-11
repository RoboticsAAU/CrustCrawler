#pragma once
#include "CrustCrawlerData.h"
#include "math.h"
#include "Methods.h"


class Dynamics
{
public:
	void UpdateDynamics(double& Looptime);

private:
	double _prevVel1{0}, _prevVel2{0}, _prevVel3{0};
	/*double _DifferentiationOperator(double currentValue, double previousValue, double& looptime);
	double _IntegrationOperator(double currentValue, double inputIntegrationVal, double& looptime);
	*/

};

