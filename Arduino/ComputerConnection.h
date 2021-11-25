#pragma once
// Generel includes
#include <Arduino.h>

// Library includes

// Custom includes
#include "DataStructures.h"

#define DEBUG_SERIAL Serial2
#define DATA_SERIAL Serial1

class ComputerConnection
{
public:
	ComputerConnection();
	Package getPackage();

	template <class dataType>
	void Print(dataType data) {
		DEBUG_SERIAL.print(data);		
	}
private:
	byte databuffer[4];
};

