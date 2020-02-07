// Manages communication with the RoboRIO

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    byte value = Serial.read();

    if(value == 0x12) {
      Serial.println("Arduino says: 'Received code 0x12.'");
    }
  }

  delay(50);
}
