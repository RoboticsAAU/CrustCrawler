/*
 Name:		Arduino.ino
 Created:	11/15/2021 3:36:53 PM
 Author:	P-363
*/
// Generel includes

// Library includes

// Custom includes
#include "Controller.h"

Controller* controller;

// the setup function runs once when you press reset or power the board
void setup() {
	controller = new Controller();
}

// the loop function runs over and over again until power down or reset
void loop() {
	controller->run();
}
