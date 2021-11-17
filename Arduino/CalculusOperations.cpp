#include "CalculusOperations.h"

double CalculusOperations::Integrate(double changeInValue, double currentValue, double& deltaTime)
{	
	currentValue += changeInValue * deltaTime;
	return currentValue;
}

double CalculusOperations::Differentiate(double currentValue, double previousValue, double& deltaTime)
{
	return (currentValue - previousValue) / deltaTime;
}
