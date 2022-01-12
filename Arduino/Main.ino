#include "ComputerConnector.h"

ComputerConnector* computerConnector;

void setup() {
	computerConnector = new ComputerConnector();


}

void loop() {
	computerConnector->debugPrintLine("Hello World!");
	delay(1000);
}

