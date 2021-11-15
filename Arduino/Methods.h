#pragma once
// Arduino includes
#include <math.h>

// Library includes
#include <BasicLinearAlgebra.h>

// Custom includes
#include "DataStructures.h"
#include "CommonData.h"

double IntegrationOperator(double currentValue, double inputIntegrationVal, unsigned long& looptime);
double DifferentiationOperator(double currentValue, double previousValue, unsigned long& looptime);
void SpaceConverter(SpaceType desiredSpace);