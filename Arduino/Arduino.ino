/*
 Name:		Arduino.ino
 Created:	15/11/2021 15:36:53
 Author:	P-363
*/
// Generel includes

// Library includes

// Custom includes
#include "Controller.h"

Controller* controller;

// The setup function runs once when you press reset or power the board
void setup() {
	delay(1000);
	controller = new Controller();
}

// The loop function runs over and over again until power down or reset
void loop() {
	controller->run();
}
