#include "Controller.h"
#include "Dynamixel2Arduino.h"

Controller* controller;

void setup() {
	controller = new Controller();
}

void loop() {
	controller->main();
}


