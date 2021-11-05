#ifndef FILTER
#define FILTER

#include "MyoBand.h"
#include <vector>
#include <cmath> 

class Filtering
{
public:
	Filtering(int sample_size, MyoBand& MyoBand);
	void Update(); //Should be called with fixed interval to emulate sampling, 
	double MoveAvg();
private:

	//Used in constructor and Update() function//
	MyoBand* pMyoBand;
	std::vector<double> samples;
	int sample_size; //Desired sample size for moving average - defined in constructor
	int current_sample_avg = 0;  //Variable to hold the current sample average - average of 8 emg signals
	int counter = 0; //Counter used for indexing the vector "samples"
	
};


#endif