#include "Controller.h"

#include "Dynamixel2Arduino.h"
#include "DynamixelShield.h"

void setup() {

	Controller controller;

}

void loop() {
	
	controller.main();


	//computerConnector->debugPrint("This is the value: ");
	//computerConnector->debugPrintLine(20);
	computerConnector->updateComputerData();

}

