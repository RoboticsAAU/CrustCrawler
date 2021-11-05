#pragma once
class ComputerConnector
{

public:
	ComputerConnector();
	~ComputerConnector();

	//Print something to the serial monitor
	void debugPrintLine();
	
	//Get the data from the computer
	int* getComputerData();


	

private:
	//Variables received from computer 
	bool emergencyStop;
	unsigned int controlMode;
	bool positiveDirection;
	unsigned int speed;

	int dataArray[4];

};

