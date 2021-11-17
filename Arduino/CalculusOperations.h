#pragma once
class CalculusOperations
{
public:
	double Integrate(double changeInValue, double currentValue, double& deltaTime);
	double Differentiate(double currentValue, double previousValue, double& deltaTime);
};

