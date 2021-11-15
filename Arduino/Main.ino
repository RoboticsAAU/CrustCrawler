#include "Controller.h"

int counter = 0;
Controller* controller;

void setup() {
	controller = new Controller();
}

void loop() {
	counter++;
	controller->main();
}


