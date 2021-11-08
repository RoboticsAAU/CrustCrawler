#ifndef FILTER
#define FILTER

#include "MyoBand.h"
#include <vector>
#include <numeric>
#include <cmath> 

class Filtering
{
public:
	// The constructor fills the samples vector with EMG data
	Filtering(int sample_size, MyoBand& MyoBand);
	
	// Calculates a moving average and return our signal magnitude
	double MoveAvg();

	// Used by SerialLink to get the same MyoBand pointer as the filter object
	MyoBand* getMyoBandPointer() { return pMyoBand; }

private:
	// Assigned in constructor and used to call new EMG data
	MyoBand* pMyoBand;

	// Holds the current EMG samples
	std::vector<double> samples;

	// Desired sample size for moving average - defined in constructor
	int m_sampleSize; 

	// Private functions
	// Calculates the average of the 8 channel EMG data and returns a sample size
	double averageEMG(std::vector<int8_t>& emg_sample);

	// Calls averageEMG and pushes it back into samples, while also removing the first element. Effectively making queue system. 
	void UpdateSamples(); 
	
};


#endif