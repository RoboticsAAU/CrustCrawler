#pragma once
// Generel includes
#include <Arduino.h>

// Library includes

// Custom includes
#include "DataStructures.h"

#define DEBUG_SERIAL Serial1
#define DATA_SERIAL Serial2

class ComputerConnection
{
public:
	ComputerConnection();
	Package getPackage();
};

