#include "Controller.h"

Controller* controller;

void setup() {
	Controller* controller = new Controller();
}

void loop() {
	controller->main();
}


