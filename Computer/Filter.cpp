#include "Filter.h"

Filtering::Filtering(int sampleSize, MyoBand &MyoBand) : pMyoBand(&MyoBand), samples(sampleSize,0), m_sampleSize(sampleSize){
	//Assign zeros to the array when the program starts, in order to have the moving average begin at zero
	for(int i = 0; i < sampleSize; i++){
		samples.at(i) = 0.0;
	}
}

//Function that returns the average of the absolute values of the 8 EMG channels. 
double Filtering::averageEMG(std::vector<int8_t> &emgSample){
	int sum = 0;
	for(int i = 0; i < 8; i++){
		sum += abs(emgSample.at(i));
	}
	return (sum / emgSample.size());
}

//Function that inserts a sample into the moving average sample container. If we decelerate, the variable addZero is true,
//and the sample inserted is simply zero. Otherwise we calculate the average EMG of the 8 channels and insert in sample container. 
void Filtering::UpdateSamples(){
	
	if (addZero) {
		samples.erase(samples.begin());
		samples.push_back(0.0);
	}
	else{
		std::vector<int8_t> rawEMGdata = pMyoBand->getEMGdata();
		double sample = averageEMG(rawEMGdata);

		samples.erase(samples.begin());
		samples.push_back(sample);
	}
}

//Function that is called whenever the moving average must be calculated. The input parameter used to determined whether or not the 
//container should be updated with a new sample (Used for print() in this class and getSpeed() in SerialLink.cpp)
double Filtering::MoveAvg(bool updateSamples){
	//The moving average is calculated for the 8-channel sample averages contained in the vector "samples"
	if (updateSamples) { UpdateSamples(); }
	double sum = std::accumulate(samples.begin(), samples.end(), 0);
	return (sum / m_sampleSize);
}

//Function for deceleration of the moving average / speed, which turns a boolean variable true/false. It is used in UpdateSamples() to
//assign 0's in the array (if true) or actual values (if false).
void Filtering::Decelerate(bool input){
	addZero = input;
}

//Debug function that is defined through include guard whenever the "Solution Configuration" is set to Debug-mode. We check this through the macro "_DEBUG"
#ifdef _DEBUG
void Filtering::print(){
	printf("MovAvg: %5.2f ", MoveAvg(false));
}
#endif
