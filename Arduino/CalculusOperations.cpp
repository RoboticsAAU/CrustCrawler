#include "CalculusOperations.h"

double CalculusOperations::Integrate(double newValue, double currentValue, double& deltaTime)
{	
	currentValue += newValue * deltaTime;
	return currentValue;
}

double CalculusOperations::Differentiate(double currentValue, double previousValue, double& deltaTime)
{
	return (currentValue - previousValue) / deltaTime;
}
