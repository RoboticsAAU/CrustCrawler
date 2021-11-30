#include "CalculusOperations.h"

double CalculusOperations::Integrate(double newValue, double currentValue, unsigned long& deltaTime)
{	
	currentValue += newValue * deltaTime;
	return currentValue;
}

double CalculusOperations::Differentiate(double currentValue, double previousValue, unsigned long& deltaTime)
{
	return (currentValue - previousValue) / deltaTime;
}
