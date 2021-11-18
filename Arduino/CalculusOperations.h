#pragma once
class CalculusOperations
{
public:
	double Integrate(double newValue, double currentValue, double& deltaTime);
	double Differentiate(double currentValue, double previousValue, double& deltaTime);
};

