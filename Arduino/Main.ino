#include "ComputerConnector.h"

#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

int counter = 0;

ComputerConnector* computerConnector;

void setup() {
	computerConnector = new ComputerConnector();
}

void loop() {
	computerConnector->debugPrint("This is the value: ");
	computerConnector->debugPrintLine(20);

	counter++;
	delay(1000);
}

