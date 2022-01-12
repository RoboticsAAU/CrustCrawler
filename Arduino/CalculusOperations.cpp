#include "CalculusOperations.h"

// Function that determines the accumulated sum over time (discrete-time integral)
double CalculusOperations::Integrate(double newValue, double currentValue, unsigned long& deltaTime)
{	
	currentValue += newValue * deltaTime;
	return currentValue;
}

// Function that determines the slope of two values, i.e. "currentValue" and "previousValue", w.r.t time (discrete-time derivative)
double CalculusOperations::Differentiate(double currentValue, double previousValue, unsigned long& deltaTime)
{
	return (currentValue - previousValue) / deltaTime;
}
