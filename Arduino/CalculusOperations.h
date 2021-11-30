#pragma once
class CalculusOperations
{
public:
	double Integrate(double newValue, double currentValue, unsigned long& deltaTime);
	double Differentiate(double currentValue, double previousValue, unsigned long& deltaTime);
};

