#pragma once
#include "CrustCrawlerData.h"

#include <BasicLinearAlgebra.h>
#include <math.h>

extern double IntegrationOperator(double currentValue, double inputIntegrationVal, unsigned long& looptime);
extern double DifferentiationOperator(double currentValue, double previousValue, unsigned long& looptime);
extern void SpaceConverter(SpaceType desiredSpace);