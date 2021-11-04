
int led = 13;

// the setup function runs once when you press reset or power the board
void setup() {

	pinMode(led, OUTPUT);

}

// the loop function runs over and over again until power down or reset
void loop() {
	digitalWrite(led, HIGH);
	delay(500);
	digitalWrite(led, LOW);
	delay(500);
}
