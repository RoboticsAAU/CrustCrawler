#include "Filter.h"

Filtering::Filtering(int sample_size, MyoBand &MyoBand) : pMyoBand(&MyoBand), samples(sample_size,0), sample_size(sample_size){
	//Currently does nothing
}

void Filtering::Update(){
	std::vector<int8_t> tmpRawEMG = pMyoBand->getEMGdata();
	
	if(counter == sample_size){
		counter = 0;
	}

	int sum = 0;
	for(int i = 0; i < tmpRawEMG.size(); i++){
		sum += std::abs(tmpRawEMG.at(i));
	}

	current_sample_avg = sum / tmpRawEMG.size(); //the size of tmpRawEMG would in reality always be 8
	
	samples.at(counter) = current_sample_avg;
	counter++;

}	


double Filtering::MoveAvg(){
	//The moving average is calculated for the 8-channel sample averages contained in the vector "samples"
	Update();
	int sum = 0;
	for(int i = 0; i<sample_size; i++){
		sum += samples[i];
	}

	double avg = sum/sample_size;

	return avg; 
}