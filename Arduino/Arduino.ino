#include "Controller.h"

// Defining a object pointer of the Controller class
Controller* controller;

// The setup function runs once when you press reset or power the board
void setup() {
	
	// Delay to manually place the robot in a starting configuration before torque is turned on
	delay(1000);
	
	// A new object of the Controller class is instantiated (heap allocation) and its address is assigned to the global pointer "controller".
	controller = new Controller();
}

// The loop function runs over and over again until power down or reset
void loop() {
	controller->run();
}
