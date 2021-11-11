#pragma once
#include "CrustCrawlerData.h"

#include <BasicLinearAlgebra.h>
#include <math.h>

extern double IntegrationOperator(double currentValue, double inputIntegrationVal, double& looptime);
extern double DifferentiationOperator(double currentValue, double previousValue, double& looptime);
extern void SpaceConverter(SpaceType desiredSpace);