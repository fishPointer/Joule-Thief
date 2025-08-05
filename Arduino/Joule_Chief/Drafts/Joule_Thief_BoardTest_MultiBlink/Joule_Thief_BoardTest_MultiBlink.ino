void setup() {
  // initialize digital pins 13 down through 5 as outputs
  for (int pin = 5; pin <= 13; pin++) {
    pinMode(pin, OUTPUT);
  }
}

// the loop function runs over and over again forever
void loop() {
  // turn all LEDs on
  for (int pin = 5; pin <= 13; pin++) {
    digitalWrite(pin, HIGH);
  }
  delay(5000);  // wait for a second
  
  // turn all LEDs off
  for (int pin = 5; pin <= 13; pin++) {
    digitalWrite(pin, LOW);
  }
  delay(5000);  // wait for a second
}