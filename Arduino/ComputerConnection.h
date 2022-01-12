#pragma once
// Generel includes
#include <Arduino.h>

// Library includes

// Custom includes
#include "DataStructures.h"

// Defining the serial ports for the data transfer from computer to Arduino and for debug purposes
#define DATA_SERIAL Serial1
#define DEBUG_SERIAL Serial2

class ComputerConnection
{
public:
	ComputerConnection();
	
	Package getPackage();

	// Function template used for debug purposes. Allows any datatype to be passed as parameter and printed through debug serial 
	template <class dataType>
	void Print(dataType data) {
		DEBUG_SERIAL.print(data);		
	}

private:
	// Array of bytes to temporarily store the package data
	byte dataBuffer[4];
	
	// Object of the class Package used for storing the four retrieved parameters from the computer
	Package returnPackage;
};

