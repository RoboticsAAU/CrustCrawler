#include "Filter.h"

Filtering::Filtering(int sampleSize, MyoBand &MyoBand) : pMyoBand(&MyoBand), samples(sampleSize,0), m_sampleSize(sampleSize){
	for(int i = 0; i < sampleSize; i++){
		std::vector<int8_t> rawEMGdata = pMyoBand->getEMGdata();
		double sample = averageEMG(rawEMGdata);
		samples.at(i) = sample;
	}
}

double Filtering::averageEMG(std::vector<int8_t> &emgSample){
	int sum = 0;
	for(int i = 0; i < 8; i++){
		sum += abs(emgSample.at(i));
	}
	return (sum / emgSample.size());
}

void Filtering::UpdateSamples(){
	std::vector<int8_t> rawEMGdata = pMyoBand->getEMGdata();
	double sample = averageEMG(rawEMGdata);

	samples.erase(samples.begin());
	samples.push_back(sample);
}	

double Filtering::MoveAvg(){
	//The moving average is calculated for the 8-channel sample averages contained in the vector "samples"
	UpdateSamples();
	double sum = std::accumulate(samples.begin(), samples.end(), 0);

	return (sum / m_sampleSize);
}

#ifdef _DEBUG
void Filtering::print(){
	printf("MovAvg: %.3f ", MoveAvg());
	//std::cout << "Moving average: [" << Filter.MoveAvg() << "]" << std::flush;
}
#endif