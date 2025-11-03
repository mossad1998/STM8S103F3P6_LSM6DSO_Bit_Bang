// Arduino Uno: Receive STM8 bit-banged serial on pin 0 (RX)
void setup() {
  Serial.begin(1200);
  Serial.println("Arduino listening on RX...");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.write(c);  // echo to Serial Monitor
  }
}
