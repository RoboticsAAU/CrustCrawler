#include "CalculusOperations.h"

double CalculusOperations::Integrate(double changeInValue, double currentValue, unsigned long& deltaTime)
{
	currentValue += changeInValue / deltaTime;
	return currentValue;
}

double CalculusOperations::Differentiate(double currentValue, double previousValue, unsigned long& deltaTime)
{
	return (currentValue - previousValue) / deltaTime;
}
